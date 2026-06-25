#include "ex_client.h"
#include "log.h"
#include "task/task_manager.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>
#include <base_switch_map/Base_switch_map_srv.h>
#include <nav_msgs/LoadMap.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <fstream>

namespace bridgex {

ExClient::ExClient(ros::NodeHandle &nh) : nh_(nh), context_(1), is_running_(false) {
  nh_.param("ex_endpoint", rep_endpoint_, std::string("tcp://*:4564"));
  nh_.param("robot_type", robot_type_, std::string("unknown"));
  if (robot_type_ == "m20") {
    std::string m20_ip;
    int m20_port;
    nh_.param("m20_ip", m20_ip, std::string("10.21.31.103"));
    nh_.param("m20_port", m20_port, 30001);
    m20_controller_ = std::make_unique<M20Controller>(m20_ip, m20_port);
  } else if (robot_type_ == "lite3") {
    std::string lite3_ip;
    int lite3_port;
    nh_.param("lite3_ip", lite3_ip, std::string("192.168.1.120"));
    nh_.param("lite3_port", lite3_port, 43893);
    lite3_controller_ = std::make_unique<Lite3Controller>(lite3_ip, lite3_port);
  } else if (robot_type_ == "agv") {
	agv_controller_ = std::make_unique<AGVController>();
  }
  std::string home_dir;
  nh_.param("home_dir", home_dir, std::string("/home/cetc21"));
  std::string config_path;
  nh_.param("config_path", config_path, home_dir + "/CETCRobot/data/map/config.txt");
  file_storage_ = std::make_unique<FileStorage>(config_path, robot_type_);

  // 提取地图基础路径 (config.txt 的父目录)
  size_t last_slash = config_path.find_last_of("/\\");
  map_base_path_ = (last_slash != std::string::npos) ? config_path.substr(0, last_slash) : config_path;

  task_manager_ = std::make_unique<TaskManager>(*this);

  camera_ = std::make_unique<Camera>(std::string("tcp://127.0.0.1:5333"));
  camera_->Connect();

  bag_recording_api_ = std::make_unique<BagRecordingApi>(std::string("tcp://127.0.0.1:15510"));
  bag_recording_api_->Init();

  mapping_api_ = std::make_unique<MappingApi>(std::string("tcp://127.0.0.1:15520"));
  mapping_api_->Init();
}

ExClient::~ExClient() { Stop(); }

bool ExClient::InitZMQ() {
  try {
    reply_socket_ = std::make_unique<zmq::socket_t>(context_, zmq::socket_type::rep);
    reply_socket_->bind(rep_endpoint_);
    LOG(INFO) << "ExClient ZMQ REP bound to: " << rep_endpoint_;
    return true;
  } catch (const zmq::error_t &e) {
    LOG(ERROR) << "ExClient ZMQ initialization error: " << e.what();
    return false;
  }
}

void ExClient::InitROS() {
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  initialpose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
  z_filter_min_pub_ = nh_.advertise<std_msgs::Float32>("z_filter_min", 1);
  clear_costmap_client_ = nh_.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
  move_base_client_ =
      std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> >("move_base", true);

  loc_map_change_client_ = nh_.serviceClient<base_switch_map::Base_switch_map_srv>("map_switch_service", true);
  nav_map_change_client_ = nh_.serviceClient<nav_msgs::LoadMap>("change_map", true);

  // 梯控服务
  lift_call_client_ = nh_.serviceClient<lift_comm::LiftCall>("/lift/call");
  lift_state_client_ = nh_.serviceClient<lift_comm::StateInquiry>("/lift/state_inquiry");
  lift_hodor_client_ = nh_.serviceClient<lift_comm::Hodor>("/lift/hodor");
  // 目标点被占据状态
  wp_occupied_sub_ = nh_.subscribe("wp_occupied", 1, &ExClient::WpOccupiedCallback, this);

  loc_sub_ = nh_.subscribe("odom", 1, &ExClient::LocCallback, this);
  amcl_sub_ = nh_.subscribe("amcl_pose", 1, &ExClient::AmclCallback, this);
  pointcloud_sub_ = nh_.subscribe("lidar_fusion_pcl", 1, &ExClient::PointCloudCallback, this);

  // 新的任务链上传成功后，发布bool状态值，以支持重置里程计路程mileage为0
  task_upload_status_pub_ = nh_.advertise<std_msgs::Bool>("clear_mileage", 10);

  robot_motion_sub_ = nh_.subscribe("stair_motion", 1, &ExClient::StairMotionCallback, this);
}
void ExClient::CleanupZMQ() {
  try {
    reply_socket_->close();
    context_.close();
    LOG(INFO) << "ExClient ZMQ cleanup successful";
  } catch (...) {
    LOG(ERROR) << "ExClient ZMQ cleanup error";
  }
}

bool ExClient::Start() {
  if (is_running_) {
    LOG(WARNING) << "ExClient is already running";
    return true;
  }

  if (!InitZMQ()) {
    LOG(ERROR) << "Failed to initialize ZMQ";
    return false;
  }

  InitROS();
  is_running_ = true;

  // 初始化完成后，延迟5秒再切换到保存的地图
  ros::Duration(3.0).sleep();
  // 初始化完成后，切换到保存的地图
  std::string saved_map_path;
  if (file_storage_ && file_storage_->Read(saved_map_path) && !saved_map_path.empty()) {
    nlohmann::json map_request;
    map_request["path"] = saved_map_path;
    map_request["x"] = 0.0;
    map_request["y"] = 0.0;
    map_request["z"] = 0.0;
    map_request["qW"] = 1.0;
    map_request["qX"] = 0.0;
    map_request["qY"] = 0.0;
    map_request["qZ"] = 0.0;

    // 尝试读取同目录下的mark.json，提取default_reloc重定位点
    std::string mark_json_path =
        map_base_path_ + "/" + saved_map_path + "/mark.json";
    std::ifstream mark_file(mark_json_path);
    if (mark_file.good()) {
      try {
        nlohmann::json mark_json;
        mark_file >> mark_json;
        if (mark_json.contains("landmarks") && mark_json["landmarks"].is_array()) {
          for (const auto &lm : mark_json["landmarks"]) {
            if (lm.value("function", "") == "default_reloc") {
              double px = lm["position"].value("x", 0.0);
              double py = lm["position"].value("y", 0.0);
              double pz = lm["position"].value("z", 0.0);
              double yaw = lm["rotation"].value("yaw", 0.0);

              map_request["x"] = px;
              map_request["y"] = py;
              map_request["z"] = pz;

              auto quat = tf::createQuaternionMsgFromYaw(yaw);
              map_request["qW"] = quat.w;
              map_request["qX"] = quat.x;
              map_request["qY"] = quat.y;
              map_request["qZ"] = quat.z;

              LOG(INFO) << "default_reloc found: pos=(" << px << "," << py << "," << pz
                        << "), yaw=" << yaw << " -> quat=(" << quat.w << "," << quat.x
                        << "," << quat.y << "," << quat.z << ")";
              break;
            }
          }
        }
      } catch (const std::exception &e) {
        LOG(ERROR) << "Failed to parse mark.json: " << e.what();
      }
    } else {
      LOG(INFO) << "No mark.json found at " << mark_json_path << ", using default pose (0,0,0)";
    }

    LOG(INFO) << "Sys Start Switching to saved map: " << saved_map_path;
    HandleMapChange(map_request);
  } else {
    LOG(INFO) << "No saved map path found, skipping map switch";
  }
  server_thread_ = std::thread(&ExClient::ServerThread, this);
  if (task_manager_) {
    task_manager_->Start();
  }
  LOG(INFO) << "ExClient started";
  return true;
}

void ExClient::Stop() {
  if (!is_running_) {
    return;
  }
  // 先设置停止标志，让线程自然退出
  is_running_ = false;
  LOG(INFO) << "Stopping task_manager_...";
  task_manager_->Stop();
  LOG(INFO) << "Stopping bag_recording_api_...";
  bag_recording_api_->Stop();
  LOG(INFO) << "Stopping mapping_api_...";
  mapping_api_->Stop();
  // 等待线程退出（最多等待1秒）
  if (server_thread_.joinable()) {
    std::chrono::milliseconds timeout(1000);
    auto start = std::chrono::steady_clock::now();
    while (server_thread_.joinable()) {
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start) >= timeout) {
        LOG(WARNING) << "ExClient thread did not exit gracefully, forcing cleanup";
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (server_thread_.joinable()) {
      // 如果线程仍在运行，detach它（避免阻塞）
      server_thread_.detach();
    }
  }

  CleanupZMQ();
  LOG(INFO) << "ExClient stopped";
}

void ExClient::ServerThread() {
  while (is_running_) {
    try {
      // 设置接收超时为100ms，以便定期检查is_running_标志
#if USE_NEW_ZMQ_API
      reply_socket_->set(zmq::sockopt::rcvtimeo, 1000);
#else
      // 使用旧API (4.3.0-4.6.x 或更早版本)
      reply_socket_->setsockopt(ZMQ_RCVTIMEO, 1000);
#endif
      // 接收请求
      zmq::message_t request;
      zmq::recv_result_t recv_result = reply_socket_->recv(request, zmq::recv_flags::none);
      if (!recv_result.has_value()) {
        // 超时是正常的，继续循环检查is_running_
        continue;
      }
      // 处理请求
      std::string request_str(static_cast<char *>(request.data()), request.size());
      std::string response_str = ProcessRequest(request_str);
      // 发送响应
      zmq::message_t reply(response_str.begin(), response_str.end());
      zmq::send_result_t send_result = reply_socket_->send(reply, zmq::send_flags::none);
      if (!send_result.has_value()) {
        LOG(ERROR) << "Failed to send response";
      }
    } catch (const zmq::error_t &e) {
      if (is_running_) {
        LOG(ERROR) << "ZMQ error in server thread: " << e.what();
      }
    } catch (const std::exception &e) {
      LOG(ERROR) << "Error in server thread: " << e.what();
    }
  }
}

std::string ExClient::ProcessRequest(const std::string &json_str) {
  try {
    nlohmann::json request = nlohmann::json::parse(json_str);

    if (!request.contains("msgType")) {
      return PackMsg(false, "unknown message type");
    }
    auto msg_type = request["msgType"].get<std::string>();
    if (msg_type == "Link") {
      return HandleLink(request);
    } else if (msg_type == "Velocity") {
      return HandleVelocity(request);
    } else if (msg_type == "Goal") {
      return HandleGoal(request);
    } else if (msg_type == "CancelNav") {
      return HandleCancelNav();
    } else if (msg_type == "Position") {
      return HandlePosition();
    } else if (msg_type == "NavStatus") {
      return HandleNavStatus();
    } else if (msg_type == "MapChange") {
      return HandleMapChange(request);
    } else if (msg_type == "MapPath") {
      return HandleMapPath();
    } else if (msg_type == "PointCloud") {
      return HandlePointCloud();
    } else if (msg_type == "M20Motion") {
      if (robot_type_ == "m20") {
        return HandleM20Motion(request);
      }
      if (robot_type_ == "lite3") {
        return HandleLite3Motion(request);
      }
      return PackMsg(false, "Robot type is not lite3");
    } else if (msg_type == "Relocalization") {
      return HandleRelocalization(request);
    } else if (msg_type == "CameraGetPTZ") {
      return HandleCameraGetPTZ();
    } else if (msg_type == "CameraControlPTZ") {
      return HandleCameraControlPTZ(request);
    } else if (msg_type == "UploadTask") {
      return HandleTaskUpload(request);
    } else if (msg_type == "TerminateTask") {
      return TerminateTask();
    } else if (msg_type == "TaskPause") {
      return HandleTaskPause();
    } else if (msg_type == "TaskResume") {
      return HandleTaskResume();
    } else if (msg_type == "TaskStatus") {
      return HandleTaskStatus();
    } else if (msg_type == "BagRecording") {
      return HandleBagRecording(request);
    } else if (msg_type == "Mapping") {
      return HandleMapping(request);
    } else if (msg_type == "LiftCall") {
      return HandleLiftCall(request);
    } else if (msg_type == "LiftState") {
      return HandleLiftState();
    } else if (msg_type == "LiftDoor") {
      return HandleLiftDoor(request);
    } else if (msg_type == "WpOccupied") {
      return HandleWpOccupied();
    } else {
      return PackMsg(false, "unknown message type");
    }
  } catch (const nlohmann::json::exception &e) {
    return PackMsg(false, std::string("json parse error: ") + e.what());
  }
}

std::string ExClient::HandleVelocity(const nlohmann::json &request) const {
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = request.value("vX", 0.0);
  twist_msg.linear.y = request.value("vY", 0.0);
  twist_msg.linear.z = request.value("vZ", 0.0);
  twist_msg.angular.x = request.value("wX", 0.0);
  twist_msg.angular.y = request.value("wY", 0.0);
  twist_msg.angular.z = request.value("wZ", 0.0);
  cmd_vel_pub_.publish(twist_msg);

  return PackMsg(true, "published vel");
}

std::string ExClient::HandleGoal(const nlohmann::json &request) const {
  if (!move_base_client_) {
    return PackMsg(false, "move_base is not ready");
  }
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = request.value("frameId", "map");
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = request.value("x", 0.0);
  goal.target_pose.pose.position.y = request.value("y", 0.0);
  goal.target_pose.pose.position.z = request.value("z", 0.0);
  if (request.contains("yaw")) {
    double yaw = request.value("yaw", 0.0);
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  } else if (request.contains("qW")) {
    double qW = request.value("qW", 1.0);
    double qX = request.value("qX", 0.0);
    double qY = request.value("qY", 0.0);
    double qZ = request.value("qZ", 0.0);
    tf::Quaternion quat(qX, qY, qZ, qW);
    double yaw = tf::getYaw(quat);
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  }
  move_base_client_->sendGoal(goal);
  return PackMsg(true, "send goal finished");
}

std::string ExClient::HandleCancelNav() const {
  if (!move_base_client_) {
    return PackMsg(false, "move_base is not ready");
  }
  move_base_client_->cancelAllGoals();

  // 停止机器人
  geometry_msgs::Twist stop_msg;
  stop_msg.linear.x = 0.0;
  stop_msg.linear.y = 0.0;
  stop_msg.linear.z = 0.0;
  stop_msg.angular.x = 0.0;
  stop_msg.angular.y = 0.0;
  stop_msg.angular.z = 0.0;
  cmd_vel_pub_.publish(stop_msg);

  return PackMsg(true, "canceled navigation");
}

std::string ExClient::HandlePosition() {
  if (!loc_pose_received_ && !amcl_pose_received_) {
    return PackMsg(false, "loc and amcl is not received");
  }

  nlohmann::json response;
  response["status"] = true;

  if (loc_pose_received_) {
    tf::Quaternion q(current_pose_loc_.pose.pose.orientation.x, current_pose_loc_.pose.pose.orientation.y,
                     current_pose_loc_.pose.pose.orientation.z, current_pose_loc_.pose.pose.orientation.w);
    double yaw = tf::getYaw(q);

    response["x"] = current_pose_loc_.pose.pose.position.x;
    response["y"] = current_pose_loc_.pose.pose.position.y;
    response["z"] = current_pose_loc_.pose.pose.position.z;
    response["qW"] = current_pose_loc_.pose.pose.orientation.w;
    response["qX"] = current_pose_loc_.pose.pose.orientation.x;
    response["qY"] = current_pose_loc_.pose.pose.orientation.y;
    response["qZ"] = current_pose_loc_.pose.pose.orientation.z;
    response["yaw"] = yaw;
    response["timeStamp"] = current_pose_loc_.header.stamp.toSec();
  }

  if (amcl_pose_received_) {
    tf::Quaternion q(current_pose_amcl_.pose.pose.orientation.x, current_pose_amcl_.pose.pose.orientation.y,
                     current_pose_amcl_.pose.pose.orientation.z, current_pose_amcl_.pose.pose.orientation.w);
    double yaw = tf::getYaw(q);

    response["x"] = current_pose_amcl_.pose.pose.position.x;
    response["y"] = current_pose_amcl_.pose.pose.position.y;
    response["z"] = current_pose_amcl_.pose.pose.position.z;
    response["qW"] = current_pose_amcl_.pose.pose.orientation.w;
    response["qX"] = current_pose_amcl_.pose.pose.orientation.x;
    response["qY"] = current_pose_amcl_.pose.pose.orientation.y;
    response["qZ"] = current_pose_amcl_.pose.pose.orientation.z;
    response["yaw"] = yaw;
    response["timeStamp"] = current_pose_amcl_.header.stamp.toSec();
  }

  return response.dump();
}

std::string ExClient::HandleNavStatus() const {
  if (!move_base_client_) {
    return PackMsg(false, "move_base is not ready");
  }

  if (!move_base_client_->isServerConnected()) {
    return PackMsg(false, "move_base server is not connected");
  }

  nlohmann::json response;
  response["status"] = true;
  response["timeStamp"] = ros::Time::now().toSec();

  actionlib::SimpleClientGoalState state = move_base_client_->getState();
  response["navigationState"] = state.toString();
  response["navigationStateCode"] = state.state_;

  //...................新增，读取text信息
  std::string goal_text = state.getText();
  //.....................................................
  bool has_active_goal =
      (state == actionlib::SimpleClientGoalState::PENDING || state == actionlib::SimpleClientGoalState::ACTIVE);
  response["hasActiveGoal"] = has_active_goal;

  std::string state_description;
  switch (state.state_) {
    case actionlib::SimpleClientGoalState::PENDING:
      state_description = "Goal is pending";
      break;
    case actionlib::SimpleClientGoalState::ACTIVE:
      state_description = "Navigation is active";
      break;
    case actionlib::SimpleClientGoalState::RECALLED:
      state_description = "Goal was recalled";
      break;
    case actionlib::SimpleClientGoalState::REJECTED:
      state_description = "Goal was rejected";
      break;
    case actionlib::SimpleClientGoalState::PREEMPTED:
      state_description = "Goal was preempted";
      break;
    case actionlib::SimpleClientGoalState::ABORTED:
      state_description = "Navigation was aborted";

      //......... 若是目标被占据，修改state_description
      if (goal_text == "GoalIsOccupied")
      {
        LOG(INFO) << "goal_text: " << goal_text;
        state_description = "GoalIsOccupied";
      }
      //...............................................

      break;
    case actionlib::SimpleClientGoalState::SUCCEEDED:
      state_description = "Navigation succeeded";
      break;
    case actionlib::SimpleClientGoalState::LOST:
      state_description = "Goal was lost";
      break;
    default:
      state_description = "Unknown state";
      break;
  }
  response["state_description"] = state_description;

  return response.dump();
}

std::string ExClient::HandleRelocalization(const nlohmann::json &request) const {
  geometry_msgs::PoseWithCovarianceStamped initialpose_msg;

  initialpose_msg.header.frame_id = request.value("frameId", "map");
  initialpose_msg.header.stamp = ros::Time::now();

  initialpose_msg.pose.pose.position.x = request.value("x", 0.0);
  initialpose_msg.pose.pose.position.y = request.value("y", 0.0);
  initialpose_msg.pose.pose.position.z = request.value("z", 0.0);

  double qW = request.value("qW", 1.0);
  double qX = request.value("qX", 0.0);
  double qY = request.value("qY", 0.0);
  double qZ = request.value("qZ", 0.0);

  initialpose_msg.pose.pose.orientation.w = qW;
  initialpose_msg.pose.pose.orientation.x = qX;
  initialpose_msg.pose.pose.orientation.y = qY;
  initialpose_msg.pose.pose.orientation.z = qZ;

  // 设置协方差矩阵（36个元素，6x6矩阵按行展开）
  // 使用默认的协方差值，用户也可以在请求中传入
  std::vector<double> covariance = request.value(
      "covariance", std::vector<double>{0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                        0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0,
                                        0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.06853891945200942});

  if (covariance.size() != 36) {
    return PackMsg(false, "covariance array must have 36 elements");
  }

  for (int i = 0; i < 36; ++i) {
    initialpose_msg.pose.covariance[i] = covariance[i];
  }

  initialpose_pub_.publish(initialpose_msg);

  return PackMsg(true, "relocalization published");
}

std::string ExClient::HandleMapChange(const nlohmann::json &request) {
  const std::string path = request.value("path", "");
  if (path.empty()) {
    return PackMsg(false, "map path is empty");
  }
  LOG(INFO) << "change path: " << path;
  bool loc_success = false;
  bool nav_success = false;
  // 尝试调用定位地图切换服务
  if (loc_map_change_client_.exists()) {
    base_switch_map::Base_switch_map_srv srv;
    srv.request.map_path = map_base_path_ + "/" + path + "/map_dahei.pcd";
    LOG(INFO) << "srv.request.map_path: " << srv.request.map_path;
    srv.request.init_x = request.value("x", 0.0);
    srv.request.init_y = request.value("y", 0.0);
    srv.request.init_z = request.value("z", 0.0);
    srv.request.init_quat_w = request.value("qW", 0.0);
    srv.request.init_quat_x = request.value("qX", 0.0);
    srv.request.init_quat_y = request.value("qY", 0.0);
    srv.request.init_quat_z = request.value("qZ", 0.0);
    LOG(INFO) << "srv.request.init_x: " << srv.request.init_x;
    LOG(INFO) << "srv.request.init_y: " << srv.request.init_y;
    LOG(INFO) << "srv.request.init_z: " << srv.request.init_z;
    LOG(INFO) << "srv.request.init_quat_w: " << srv.request.init_quat_w;
    LOG(INFO) << "srv.request.init_quat_x: " << srv.request.init_quat_x;
    LOG(INFO) << "srv.request.init_quat_y: " << srv.request.init_quat_y;
    LOG(INFO) << "srv.request.init_quat_z: " << srv.request.init_quat_z;
    LOG(INFO) << "waitForExistence";
    loc_map_change_client_.waitForExistence();
    if (!loc_map_change_client_.call(srv)) {
      LOG(INFO) << "loc map service call failed restart service";
      // 重启服务接口，
      if (!loc_map_change_client_.call(srv))
      {
        loc_map_change_client_.shutdown();
        loc_map_change_client_ =
            nh_.serviceClient<base_switch_map::Base_switch_map_srv>("map_switch_service", true);
        loc_map_change_client_.call(srv);
      }
    } else if (srv.response.finshed) {
      LOG(INFO) << "loc map change success";
      loc_success = true;
    } else {
      LOG(INFO) << "loc map change failed (response.finshed=false)";
    }
  } else {
    LOG(INFO) << "loc map service client does not exist";
  }
  // 尝试调用导航地图切换服务
  if (nav_map_change_client_.exists()) {
    nav_msgs::LoadMap srv;
    srv.request.map_url = map_base_path_ + "/" + path + "/map_dahei.yaml";
    if (!nav_map_change_client_.call(srv)) {
      LOG(INFO) << "nav map service call failed, restart service register";
      if (!nav_map_change_client_.call(srv))
      {
        nav_map_change_client_.shutdown();
        nav_map_change_client_ =
            nh_.serviceClient<base_switch_map::Base_switch_map_srv>("change_map", true);
        nav_map_change_client_.call(srv);
      }
    } else if (srv.response.result == nav_msgs::LoadMap::Response::RESULT_SUCCESS) {
      LOG(INFO) << "srv.request.map_url: " << srv.request.map_url;
      nav_success = true;
    } else {
      LOG(INFO) << "nav map change failed (result=" << srv.response.result << ")";
    }
  } else {
    LOG(INFO) << "nav map service client does not exist";
  }

  // 判断整体是否成功
  if (loc_success || nav_success) {
    if (file_storage_) {
      file_storage_->Save(path);
    }
    std::string message = "map change success";
    if (loc_success && !nav_success) {
      message = "map change success (loc only)";
    } else if (!loc_success && nav_success) {
      message = "map change success (nav only)";
    }
    return PackMsg(true, message);
  } else {
    std::string error_msg;
    if (loc_map_change_client_.exists() || nav_map_change_client_.exists()) {
      error_msg = "map change failed (both services failed)";
    } else {
      error_msg = "map service clients are not ready";
    }
    return PackMsg(false, error_msg);
  }
}

void ExClient::LocCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  current_pose_loc_ = *msg;
  loc_pose_received_ = true;
}

void ExClient::AmclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
  current_pose_amcl_ = *msg;
  amcl_pose_received_ = true;
}

std::string ExClient::HandleLink(const nlohmann::json &request) {
  nlohmann::json response;
  response["status"] = true;
  response["message"] = "link success";
  response["robotType"] = robot_type_;
  response["timeStamp"] = ros::Time::now().toSec();
  return response.dump();
}

std::string ExClient::PackMsg(bool success, const std::string &message) {
  nlohmann::json response;
  response["status"] = success;
  response["message"] = message;
  response["timeStamp"] = ros::Time::now().toSec();
  return response.dump();
}

void ExClient::PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
  std::lock_guard lock(pointcloud_mutex_);
  current_pointcloud_ = *msg;
  pointcloud_received_ = true;
}

void ExClient::StairMotionCallback(const std_msgs::String::ConstPtr &msg) {
  std::string motion_type = msg->data;
  if (motion_type == "none" || motion_type.empty()) {
    LOG(INFO) << "Received none or empty motion command, skipping execution.";
    return;
  }

  LOG(INFO) << "Received stair motion command: " << motion_type;
  nlohmann::json request;
  request["motionType"] = motion_type;
  std::string result = HandleM20Motion(request);
  LOG(INFO) << "HandleM20Motion result: " << result;
}
nlohmann::json pointcloud2ToXYZJson(const sensor_msgs::PointCloud2 &msg) {
  nlohmann::json j;
  j["points"] = nlohmann::json::array();
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) continue;
    j["points"].push_back({*iter_x, *iter_y, *iter_z});
  }
  return j;
}
std::string ExClient::HandlePointCloud() {
  std::lock_guard lock(pointcloud_mutex_);
  if (!pointcloud_received_) {
    return PackMsg(false, "pointcloud data not received");
  }
  nlohmann::json response;
  response["status"] = true;
  response["message"] = "pointcloud2";
  response["frameId"] = current_pointcloud_.header.frame_id;
  response["timeStamp"] = current_pointcloud_.header.stamp.toSec();
  nlohmann::json pointcloud_data = pointcloud2ToXYZJson(current_pointcloud_);
  response["pointCloud"] = pointcloud_data;
  return response.dump();
}

std::string ExClient::HandleM20Motion(const nlohmann::json &request) {
  if (!m20_controller_) {
    return PackMsg(false, "M20 controller not initialized");
  }
  // 确保已连接
  if (!m20_controller_->isConnected() && !m20_controller_->connect()) {
    return PackMsg(false, "Failed to connect to M20 robot");
  }

  std::string motion_type = request.value("motionType", "");
  bool success = false;
  std::string description;

  // 辅助函数：发布 z_filter_min
  auto publishZFilterMin = [this](float value) {
    std_msgs::Float32 msg;
    msg.data = value;
    LOG(INFO) << "Publishing /z_filter_min = " << msg.data;
    for (int i = 0; i < 5; i++) {
      z_filter_min_pub_.publish(msg);
      ros::spinOnce();
      ros::Duration(0.2).sleep();
    }
  };

  // 辅助函数：清空 costmap
  auto clearCostmap = [this]() -> bool {
    std_srvs::Empty srv;
    if (clear_costmap_client_.call(srv)) {
      LOG(INFO) << "Costmaps cleared successfully.";
      return true;
    } else {
      LOG(ERROR) << "Failed to clear costmaps.";
      return false;
    }
  };

  // 根据运动类型执行相应指令
  if (motion_type == "stand") {
    success = m20_controller_->standUp();
    description = "stand up";
  } else if (motion_type == "sit") {
    success = m20_controller_->sitDown();
    description = "sit down";
  } else if (motion_type == "idle") {
    success = m20_controller_->setMotionIdle();
    description = "idle";
  } else if (motion_type == "normal") {
    success = m20_controller_->setUsageModeNormal();
    description = "normal mode";
  } else if (motion_type == "damping") {
    success = m20_controller_->motionDamping();
    description = "damping";
  } else if (motion_type == "boot_damping") {
    success = m20_controller_->motionBootDamping();
    description = "boot damping";
  } else if (motion_type == "flat") {
    // 平地敏捷模式
    success = m20_controller_->setGaitFlatFast();  // 平地敏捷模式 (0x3002)
    success = m20_controller_->setUsageModeNav();
    // 发送 z_filter_min = 0.35
    publishZFilterMin(-0.4f);
    description = "flat gait";
  } else if (motion_type == "stair") {
    // 楼梯敏捷模式
    success = m20_controller_->setGaitStairFast();  // 楼梯敏捷模式 (0x3003)
    success = m20_controller_->setUsageModeNav();
    // 发送 z_filter_min = 1.0
    // publishZFilterMin(1.0f);
    publishZFilterMin(0.5f);

    // 清空 costmap
    clearCostmap();
    description = "stair gait";
  } else if (motion_type == "basic_std") {
    success = m20_controller_->setGaitBasicStd();  // 基础标准模式 (0x1001)
    success= m20_controller_->setUsageModeNormal();
    description = "basic standard gait";
  } else if (motion_type == "start_charge") {
    success = m20_controller_->startCharge();
    if (success) {
      LOG(INFO)<< "Charging started.";
      // 发送充电指令后，开始查询充电状态，最多等待50s
      success = m20_controller_->waitForEnterCharge(50000);
    }
    description = "start charging";
  } else if (motion_type == "stop_charge") {
    success = m20_controller_->stopCharge();
    if (success) {
      LOG(INFO) << "stop charging";
      // 发送停止充电指令后，开始查询直到空闲状态，最多等待50s
      success = m20_controller_->waitForExitCharge(50000);
    }
    description = "stop charging";
  } else if (motion_type == "clear_charge") {
    success = m20_controller_->clearCharge();
    if (success) {
      // 发送清除充电状态指令后，开始查询直到空闲状态，最多等待50s
      success = m20_controller_->waitForExitCharge(50000);
    }
    description = "clear charge status";
  } else if (motion_type == "move") {
    // 统一的运动控制接口（轴指令）- 仅在常规模式下生效
    double x = request.value("x", 0.0);
    double y = request.value("y", 0.0);
    double z = request.value("z", 0.0);
    double roll = request.value("roll", 0.0);
    double pitch = request.value("pitch", 0.0);
    double yaw = request.value("yaw", 0.0);
    // 验证参数范围（根据协议文档，范围应为 [-1, 1]）
    if (x < -1.0 || x > 1.0 || y < -1.0 || y > 1.0 || z < -1.0 || z > 1.0 || roll < -1.0 || roll > 1.0 ||
        pitch < -1.0 || pitch > 1.0 || yaw < -1.0 || yaw > 1.0) {
      return PackMsg(false, "Invalid axis speed parameters. All values must be in range [-1.0, 1.0]");
    }
    success = m20_controller_->setAxisSpeed(x, y, z, roll, pitch, yaw);
    description = "axis speed";
  } else {
    return PackMsg(false,
                   "Unknown motion type: " + motion_type +
                       ". Supported types: stand, sit, idle, normal, damping, boot_damping, flat, stair, basic_std, "
                       "start_charge, stop_charge, clear_charge, move");
  }
  // 返回结果
  if (success) {
    nlohmann::json response;
    response["status"] = true;
    response["message"] = "M20 motion command executed successfully";
    response["motionType"] = motion_type;
    response["description"] = description;
    response["timeStamp"] = ros::Time::now().toSec();
    return response.dump();
  } else {
    return PackMsg(false, "Failed to execute M20 motion command: " + description);
  }
}

std::string ExClient::HandleLite3Motion(const nlohmann::json &request) const
{
  if (!lite3_controller_) {
    return PackMsg(false, "Lite3 controller not initialized");
  }
  if (!lite3_controller_->isConnected() && !lite3_controller_->connect()) {
    return PackMsg(false, "Failed to connect to Lite3 robot");
  }

  std::string motion_type = request.value("motionType", "");
  bool success = false;
  std::string description;

  if (motion_type == "stand") {
    success = lite3_controller_->standUp();
    description = "stand up";
  } else if (motion_type == "sit") {
    success = lite3_controller_->sitDown();
    description = "sit down";
  } else if (motion_type == "flat") {
    success = lite3_controller_->navControlMode();
    description = "nav mode";
  } else if (motion_type == "basic_std") {
    success = lite3_controller_->handleControlMode();
    description = "normal mode";
  } else {
    return PackMsg(false,
                   "Unknown motion type: " + motion_type +
                       ". Supported types: stand, sit, nav, normal");
  }

  if (success) {
    nlohmann::json response;
    response["status"] = true;
    response["message"] = "Lite3 motion command executed successfully";
    response["motionType"] = motion_type;
    response["description"] = description;
    response["timeStamp"] = ros::Time::now().toSec();
    return response.dump();
  } else {
    return PackMsg(false, "Failed to execute Lite3 motion command: " + description);
  }
}

std::string ExClient::HandleMapPath() const {
  if (!file_storage_) {
    return PackMsg(false, "file storage is not initialized");
  }
  std::string path;
  if (!file_storage_->Read(path)) {
    return PackMsg(false, "failed to read map path from file");
  }
  nlohmann::json response;
  response["status"] = true;
  response["message"] = "map path retrieved";
  response["mapPath"] = path;
  response["timeStamp"] = ros::Time::now().toSec();
  return response.dump();
}

std::string ExClient::HandleTaskUpload(const nlohmann::json &request) const {
  if (!task_manager_) {
    return PackMsg(false, "task manager is not initialized");
  }
  if (!request.contains("task")) {
    return PackMsg(false, "request must contain 'task' field");
  }

  try {
    if (!task_manager_->ReceiveTask(request["task"])) {
      return PackMsg(false, "failed to receive task");
    }

    // 新的任务链上传成功后，发布bool状态值，以支持重置里程计路程mileage为0
    std_msgs::Bool status_msg;
    status_msg.data = true; // 或者 false
    task_upload_status_pub_.publish(status_msg);

    return PackMsg(true, "task started");
  } catch (const nlohmann::json::exception &e) {
    return PackMsg(false, std::string("json parse error: ") + e.what());
  }
}

std::string ExClient::TerminateTask() const {
  if (!task_manager_) {
    return PackMsg(false, "task manager is not initialized");
  }
  task_manager_->TerminateTask();
  return PackMsg(true, "task cancelled");
}


std::string ExClient::HandleTaskPause() const {
  if (!task_manager_) {
    return PackMsg(false, "task manager is not initialized");
  }
  task_manager_->PauseTask();
  return PackMsg(true, "task paused");
}


std::string ExClient::HandleTaskResume() const {
  if (!task_manager_) {
    return PackMsg(false, "task manager is not initialized");
  }
  task_manager_->ResumeTask();
  return PackMsg(true, "task resumed");
}

std::string ExClient::HandleTaskStatus() const {
  if (!task_manager_) {
    return PackMsg(false, "task manager is not initialized");
  }
  TaskStatus status = task_manager_->GetStatus();
  nlohmann::json response;
  response["status"] = true;
  response["message"] = "task status retrieved";
  response["timeStamp"] = ros::Time::now().toSec();
  response["taskStatus"] = {{"state", static_cast<int32_t>(status.state)},
                            {"taskName", status.task_name},
                            {"currentActionIndex", status.current_action_index},
                            {"totalActions", status.total_actions},
                            {"currentActionType", status.current_action_type},
                            {"errorMessage", status.error_message},
                            {"timestamp", status.timestamp}};
  return response.dump();
}

std::string ExClient::HandleCameraGetPTZ() const {
  if (!camera_) {
    return PackMsg(false, "camera is not initialized");
  }
  PTZPosition position;
  std::string error_msg;
  if (!camera_->GetPTZ(position, error_msg)) {
    return PackMsg(false, std::string("Failed to get PTZ position: ") + error_msg);
  }
  nlohmann::json response;
  response["status"] = true;
  response["message"] = "PTZ position retrieved";
  response["timeStamp"] = ros::Time::now().toSec();
  response["ptz"] = {{"roll", position.roll},
                     {"pitch", position.pitch},
                     {"yaw", position.yaw}};
  return response.dump();
}
std::string ExClient::HandleCameraControlPTZ(const nlohmann::json &request) {
  if (!camera_) {
    return PackMsg(false, "camera is not initialized");
  }
  PTZPosition position;
  position.roll = request.value("roll", 0.0);
  position.pitch = request.value("pitch", 0.0);
  position.yaw = request.value("yaw", 0.0);
  std::string error_msg;
  if (!camera_->ControlPTZ(position, error_msg)) {
    return PackMsg(false, std::string("Failed to control PTZ: ") + error_msg);
  }
  return PackMsg(true, "PTZ control command sent");
}

std::string ExClient::HandleBagRecording(const nlohmann::json &request) {
  LOG(INFO) << "HandleBagRecording:" << request.dump();
  if (!bag_recording_api_) {
    return PackMsg(false, "bag_recording_api is not initialized");
  }

  std::string action = request.value("action", "");
  std::string result;

  if (action == "start_record") {
    LOG(INFO) << "BagRecording: start_record";
    result = bag_recording_api_->StartRecord();
  } else if (action == "stop_record") {
    LOG(INFO) << "BagRecording: stop_record";
    result = bag_recording_api_->StopRecord();
  } else if (action == "ping") {
    result = bag_recording_api_->Ping();
  } else {
    return PackMsg(false, "unknown bag_recording action: " + action + ". Supported: start_record / stop_record / ping");
  }

  LOG(INFO) << "BagRecording result: " << result;
  nlohmann::json response;
  response["status"] = true;
  response["message"] = result;
  response["timeStamp"] = ros::Time::now().toSec();
  return response.dump();
}

std::string ExClient::HandleMapping(const nlohmann::json &request) const
{
  LOG(INFO) << "HandleMapping:" << request.dump();
  if (!mapping_api_) {
    return PackMsg(false, "mapping_api is not initialized");
  }
  std::string action = request.value("action", "");
  std::string result;

  if (action == "start_map") {
    std::string map_name = request.value("mapName", "map");
    LOG(INFO) << "Mapping: start_map:" << map_name;
    result = mapping_api_->StartMap(map_name);
  } else if (action == "wait_map_complete") {
    LOG(INFO) << "Mapping: wait_map_complete";
    result = mapping_api_->WaitMapComplete();
  } else if (action == "abort_map") {
    LOG(INFO) << "Mapping: abort_map";
    result = mapping_api_->AbortMap();
  } else if (action == "ping") {
    result = mapping_api_->Ping();
  } else {
    return PackMsg(false,
                   "unknown mapping action: " + action + ". Supported: start_map / wait_map_complete / abort_map / ping");
  }

  LOG(INFO) << "Mapping result: " << result;
  nlohmann::json response;
  response["status"] = true;
  response["message"] = result;
  response["timeStamp"] = ros::Time::now().toSec();
  return response.dump();
}

void ExClient::WpOccupiedCallback(const std_msgs::Bool::ConstPtr &msg) {
  std::lock_guard lock(wp_occupied_mutex_);
  wp_occupied_ = msg->data;
  wp_occupied_stamp_ = ros::Time::now();
}

std::string ExClient::HandleLiftCall(const nlohmann::json &request) {
  if (!lift_call_client_) {
    LOG(ERROR) << "HandleLiftCall: lift call client not ready";
    return PackMsg(false, "lift call client not ready");
  }
  int floor = request.value("floor", 0);
  lift_comm::LiftCall srv;
  srv.request.target_floor = static_cast<uint8_t>(floor);
  LOG(INFO) << "HandleLiftCall: requesting elevator to floor " << floor;
  if (!lift_call_client_.call(srv)) {
    LOG(ERROR) << "HandleLiftCall: /lift/call service call failed (floor=" << floor << ")";
    return PackMsg(false, "lift call service call failed");
  }
  LOG(INFO) << "HandleLiftCall: floor=" << floor << " success=" << (srv.response.success ? "true" : "false");
  nlohmann::json response;
  response["status"] = srv.response.success;
  response["message"] = srv.response.success ? "lift call ok" : "lift call rejected";
  response["timeStamp"] = ros::Time::now().toSec();
  return response.dump();
}

std::string ExClient::HandleLiftState() {
  if (!lift_state_client_) {
    LOG(ERROR) << "HandleLiftState: lift state client not ready";
    return PackMsg(false, "lift state client not ready");
  }
  lift_comm::StateInquiry srv;
  srv.request.state_inquiry = true;
  if (!lift_state_client_.call(srv)) {
    LOG(WARNING) << "HandleLiftState: /lift/state_inquiry service call failed";
    return PackMsg(false, "lift state service call failed");
  }
  nlohmann::json response;
  response["status"] = true;
  response["message"] = "lift state ok";
  response["doorOpen"] = srv.response.door_open;
  response["currentFloor"] = srv.response.current_floor;
  response["upDownState"] = srv.response.up_down_state;
  response["timeStamp"] = ros::Time::now().toSec();
  return response.dump();
}

std::string ExClient::HandleLiftDoor(const nlohmann::json &request) {
  if (!lift_hodor_client_) {
    LOG(ERROR) << "HandleLiftDoor: lift hodor client not ready";
    return PackMsg(false, "lift hodor client not ready");
  }
  bool open = request.value("open", false);
  lift_comm::Hodor srv;
  srv.request.hodor = open;
  LOG(INFO) << "HandleLiftDoor: " << (open ? "open(hold)" : "close(stop hold)");
  if (!lift_hodor_client_.call(srv)) {
    LOG(ERROR) << "HandleLiftDoor: /lift/hodor service call failed (open=" << (open ? "true" : "false") << ")";
    return PackMsg(false, "lift hodor service call failed");
  }
  LOG(INFO) << "HandleLiftDoor: " << (open ? "open" : "close")
            << " success=" << (srv.response.success ? "true" : "false");
  nlohmann::json response;
  response["status"] = srv.response.success;
  response["message"] = srv.response.success ? "lift door ok" : "lift door failed";
  response["timeStamp"] = ros::Time::now().toSec();
  return response.dump();
}

std::string ExClient::HandleWpOccupied() {
  bool occupied;
  double age_ms;
  {
    std::lock_guard lock(wp_occupied_mutex_);
    occupied = wp_occupied_;
    age_ms = wp_occupied_stamp_.isZero()
                 ? 1e12
                 : (ros::Time::now() - wp_occupied_stamp_).toSec() * 1000.0;
  }
  nlohmann::json response;
  response["status"] = true;
  response["message"] = "wp occupied state";
  response["occupied"] = occupied;
  response["ageMs"] = age_ms;
  response["timeStamp"] = ros::Time::now().toSec();
  return response.dump();
}

bool ExClient::GetMapBasePath(std::string& map_base_path) const {
  map_base_path = map_base_path_;
  return true;
}

}  // namespace bridgex
