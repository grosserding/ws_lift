#pragma once

#include <ros/ros.h>
#include <zmq.hpp>
#include <string>
#include <thread>
#include <mutex>
#include <memory>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "json.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include "robot/m20_controller.h"
#include "robot/lite3_controller.h"
#include "robot/agv_controller.h"
#include "file_storage.h"
#include "task/task_manager.h"
#include "map/bag_recording_api.h"
#include "map/mapping_api.h"

#include "cam/camera.h"
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
namespace bridgex {

class ExClient {
 public:
  explicit ExClient(ros::NodeHandle &nh);

  ~ExClient();

  bool Start();

  void Stop();

  bool IsRunning() const { return is_running_; }

  std::string ProcessRequest(const std::string &json_str);

  bool GetMapBasePath(std::string& map_base_path) const;

 private:
  ros::NodeHandle &nh_;
  zmq::context_t context_;
  std::unique_ptr<zmq::socket_t> reply_socket_;
  std::string rep_endpoint_;
  bool is_running_;
  std::thread server_thread_;
  std::mutex mutex_;

  // 机器人类型
  std::string robot_type_;

  // 速度控制
  ros::Publisher cmd_vel_pub_;
  // 重定位
  ros::Publisher initialpose_pub_;
  // z_filter_min 发布器（用于障碍过滤）
  ros::Publisher z_filter_min_pub_;
  // 清空 costmap 服务
  ros::ServiceClient clear_costmap_client_;
  // 导航相关
  std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> > move_base_client_;
  // 地图切换
  ros::ServiceClient loc_map_change_client_;
  ros::ServiceClient nav_map_change_client_;
  // 定位相关
  ros::Subscriber loc_sub_;
  ros::Subscriber amcl_sub_;

  // 新的任务链上传成功后，发布bool状态值，以支持重置里程计路程mileage为0
  ros::Publisher task_upload_status_pub_;

  bool loc_pose_received_{false};
  nav_msgs::Odometry current_pose_loc_;

  bool amcl_pose_received_{false};
  geometry_msgs::PoseWithCovarianceStamped current_pose_amcl_;

  ros::Subscriber pointcloud_sub_;
  bool pointcloud_received_{false};
  sensor_msgs::PointCloud2 current_pointcloud_;
  std::mutex pointcloud_mutex_;

  std::unique_ptr<M20Controller> m20_controller_;
  std::unique_ptr<Lite3Controller> lite3_controller_;
  std::unique_ptr<AGVController> agv_controller_;

  std::unique_ptr<FileStorage> file_storage_;

  std::unique_ptr<TaskManager> task_manager_;

  // 地图基础路径
  std::string map_base_path_;

  std::unique_ptr<Camera> camera_;

  std::unique_ptr<BagRecordingApi> bag_recording_api_;
  std::unique_ptr<MappingApi> mapping_api_;
  // 楼梯运动
  ros::Subscriber robot_motion_sub_;
  void StairMotionCallback(const std_msgs::String::ConstPtr &msg);

  bool InitZMQ();

  void InitROS();

  void CleanupZMQ();

  void ServerThread();

  std::string HandleVelocity(const nlohmann::json &request) const;

  std::string HandleGoal(const nlohmann::json &request) const;

  std::string HandleCancelNav() const;

  std::string HandlePosition();

  std::string HandleNavStatus() const;

  std::string HandleRelocalization(const nlohmann::json &request) const;

  std::string HandleMapChange(const nlohmann::json &request);

  std::string HandleMapPath() const;

  std::string HandlePointCloud();

  std::string HandleM20Motion(const nlohmann::json &request);

  std::string HandleLite3Motion(const nlohmann::json &request) const;

  std::string HandleLink(const nlohmann::json &request);

  std::string HandleTaskUpload(const nlohmann::json &request) const;

  std::string TerminateTask() const;

  std::string HandleTaskPause() const;

  std::string HandleTaskResume() const;

  std::string HandleTaskStatus() const;

  std::string HandleCameraGetPTZ() const;

  std::string HandleCameraControlPTZ(const nlohmann::json &request);

  std::string HandleBagRecording(const nlohmann::json &request);

  std::string HandleMapping(const nlohmann::json &request) const;

  void LocCallback(const nav_msgs::Odometry::ConstPtr &msg);

  void AmclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

  void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

  static std::string PackMsg(bool success = false, const std::string &message = "");
};
}  // namespace bridgex
