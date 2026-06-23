#include "amclpose_message.h"
#include "log.h"

AMCLPoseMessage::AMCLPoseMessage(const std::string &name, ros::NodeHandle &nh)
    : MessageBase(name, nh), ros_topic_(name), zmq_topic_(name) {
  // 初始化位姿数据
  data_.header.frame_id = "map";
  data_.pose.pose.position.x = 0.0;
  data_.pose.pose.position.y = 0.0;
  data_.pose.pose.position.z = 0.0;
  data_.pose.pose.orientation.w = 1.0;
  data_.pose.pose.orientation.x = 0.0;
  data_.pose.pose.orientation.y = 0.0;
  data_.pose.pose.orientation.z = 0.0;
  // 初始化协方差矩阵为0
  for (int i = 0; i < 36; i++) {
    data_.pose.covariance[i] = 0.0;
  }
}

void AMCLPoseMessage::Init() {
  // 初始化ROS发布者
  ros_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(ros_topic_ + "_bridgex", 10);
  // 初始化ROS订阅者
  ros_sub_ = nh_.subscribe(ros_topic_, 10, &AMCLPoseMessage::RosCallback, this);
  // 设置ZMQ订阅
  SubFromZMQ(zmq_topic_, std::bind(&AMCLPoseMessage::ReceiveFromZMQ, this, std::placeholders::_1));
  LOG(INFO) << "[" << name_ << "] initialized, ROS topic: " << ros_topic_;
}

void AMCLPoseMessage::FromJson(const nlohmann::json &json) {
  // 解析位置
  if (json.contains("position")) {
    const auto &pos = json["position"];
    if (pos.contains("x")) {
      data_.pose.pose.position.x = pos["x"].get<double>();
    }
    if (pos.contains("y")) {
      data_.pose.pose.position.y = pos["y"].get<double>();
    }
    if (pos.contains("z")) {
      data_.pose.pose.position.z = pos["z"].get<double>();
    }
  }

  // 解析方向四元数
  if (json.contains("orientation")) {
    const auto &ori = json["orientation"];
    if (ori.contains("w")) {
      data_.pose.pose.orientation.w = ori["w"].get<double>();
    }
    if (ori.contains("x")) {
      data_.pose.pose.orientation.x = ori["x"].get<double>();
    }
    if (ori.contains("y")) {
      data_.pose.pose.orientation.y = ori["y"].get<double>();
    }
    if (ori.contains("z")) {
      data_.pose.pose.orientation.z = ori["z"].get<double>();
    }
  }

  // 解析协方差矩阵
  if (json.contains("covariance") && json["covariance"].is_array() && json["covariance"].size() == 36) {
    const auto &cov = json["covariance"];
    for (int i = 0; i < 36; i++) {
      data_.pose.covariance[i] = cov[i].get<double>();
    }
  }

  // 解析时间戳和坐标系
  if (json.contains("timestamp")) {
    data_.header.stamp.fromSec(json["timestamp"].get<double>());
  }
  if (json.contains("frame_id")) {
    data_.header.frame_id = json["frame_id"].get<std::string>();
  }
}

nlohmann::json AMCLPoseMessage::ToJson() const {
  nlohmann::json json;
  json["type"] = "amclpose";

  // 位置
  json["position"]["x"] = data_.pose.pose.position.x;
  json["position"]["y"] = data_.pose.pose.position.y;
  json["position"]["z"] = data_.pose.pose.position.z;

  // 方向四元数
  json["orientation"]["w"] = data_.pose.pose.orientation.w;
  json["orientation"]["x"] = data_.pose.pose.orientation.x;
  json["orientation"]["y"] = data_.pose.pose.orientation.y;
  json["orientation"]["z"] = data_.pose.pose.orientation.z;

  // 协方差矩阵
  nlohmann::json cov = nlohmann::json::array();
  for (int i = 0; i < 36; i++) {
    cov.push_back(data_.pose.covariance[i]);
  }
  json["covariance"] = cov;

  json["timestamp"] = data_.header.stamp.toSec();
  json["frame_id"] = data_.header.frame_id;

  return json;
}

void AMCLPoseMessage::PublishToROS() {
  if (!is_running_) return;
  ros_pub_.publish(data_);
}

void AMCLPoseMessage::RosCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
  data_ = *msg;
  // ROS_INFO("[%s] Received ROS AMCL pose: position[%.3f, %.3f, %.3f]", name_.c_str(), data_.pose.pose.position.x,
  //          data_.pose.pose.position.y, data_.pose.pose.position.z);
  PublishToZMQ();
}

void AMCLPoseMessage::PublishToZMQ() {
  if (!is_running_) return;
  try {
    const nlohmann::json json = ToJson();
    const std::string json_str = json.dump();
    PubToZMQ(zmq_topic_, json_str);
    // ROS_INFO("[%s] ZMQ publish: %s", name_.c_str(), json_str.c_str());
  } catch (const std::exception &e) {
    LOG(ERROR) << "[" << name_ << "] ZMQ publish error: " << e.what();
  }
}

void AMCLPoseMessage::ReceiveFromROS(const std::string &topic_name) {
  // 这个函数在RosCallback中实现
}

void AMCLPoseMessage::ReceiveFromZMQ(const std::string &data) {
  try {
    nlohmann::json json = nlohmann::json::parse(data);
    if (json["type"].get<std::string>() == "amclpose") {
      FromJson(json);
      // ROS_INFO("[%s] Received ZMQ AMCL pose: position[%.3f, %.3f, %.3f]", name_.c_str(), data_.pose.pose.position.x,
      //          data_.pose.pose.position.y, data_.pose.pose.position.z);
      // 转发到ROS
      PublishToROS();
    }
  } catch (const nlohmann::json::exception &e) {
    LOG(ERROR) << "[" << name_ << "] JSON parse error: " << e.what();
  }
}