#include "cmd_vel_message.h"
#include "log.h"

CmdVelMessage::CmdVelMessage(const std::string &name, ros::NodeHandle &nh)
    : MessageBase(name, nh), ros_topic_(name), zmq_topic_(name) {
  // 初始化速度为0
  data_.linear.x = 0.0;
  data_.linear.y = 0.0;
  data_.linear.z = 0.0;
  data_.angular.x = 0.0;
  data_.angular.y = 0.0;
  data_.angular.z = 0.0;
}

void CmdVelMessage::Init() {
  // 初始化ROS发布者
  ros_pub_ = nh_.advertise<geometry_msgs::Twist>(ros_topic_ + "_bridgex", 10);
  // 初始化ROS订阅者
  ros_sub_ = nh_.subscribe(ros_topic_, 10, &CmdVelMessage::RosCallback, this);
  // 设置ZMQ订阅
  SubFromZMQ(zmq_topic_, std::bind(&CmdVelMessage::ReceiveFromZMQ, this, std::placeholders::_1));
  LOG(INFO) << "[" << name_ << "] initialized, ROS topic: " << ros_topic_;
}

void CmdVelMessage::FromJson(const nlohmann::json &json) {
  if (json.contains("linear")) {
    const auto &linear = json["linear"];
    if (linear.contains("x")) {
      data_.linear.x = linear["x"].get<double>();
    }
    if (linear.contains("y")) {
      data_.linear.y = linear["y"].get<double>();
    }
    if (linear.contains("z")) {
      data_.linear.z = linear["z"].get<double>();
    }
  }
  if (json.contains("angular")) {
    const auto &angular = json["angular"];
    if (angular.contains("x")) {
      data_.angular.x = angular["x"].get<double>();
    }
    if (angular.contains("y")) {
      data_.angular.y = angular["y"].get<double>();
    }
    if (angular.contains("z")) {
      data_.angular.z = angular["z"].get<double>();
    }
  }
}

nlohmann::json CmdVelMessage::ToJson() const {
  nlohmann::json json;
  json["type"] = "cmd_vel";

  // 线速度
  json["linear"]["x"] = data_.linear.x;
  json["linear"]["y"] = data_.linear.y;
  json["linear"]["z"] = data_.linear.z;

  // 角速度
  json["angular"]["x"] = data_.angular.x;
  json["angular"]["y"] = data_.angular.y;
  json["angular"]["z"] = data_.angular.z;

  json["timestamp"] = ros::Time::now().toSec();
  return json;
}

void CmdVelMessage::PublishToROS() {
  if (!is_running_) return;
  ros_pub_.publish(data_);
}

void CmdVelMessage::RosCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  data_ = *msg;
  // ROS_INFO("[%s] Received ROS velocity command: linear[%.3f, %.3f, %.3f], angular[%.3f, %.3f, %.3f]", name_.c_str(),
  //          data_.linear.x, data_.linear.y, data_.linear.z, data_.angular.x, data_.angular.y, data_.angular.z);
  PublishToZMQ();
}

void CmdVelMessage::PublishToZMQ() {
  if (!is_running_) return;
  try {
    nlohmann::json json = ToJson();
    const std::string json_str = json.dump();
    PubToZMQ(zmq_topic_, json_str);
    // ROS_INFO("[%s] ZMQ publish: %s", name_.c_str(), json_str.c_str());
  } catch (const std::exception &e) {
    LOG(ERROR) << "[" << name_ << "] ZMQ publish error: " << e.what();
  }
}

void CmdVelMessage::ReceiveFromROS(const std::string &topic_name) {
  // 这个函数在RosCallback中实现
}

void CmdVelMessage::ReceiveFromZMQ(const std::string &data) {
  try {
    nlohmann::json json = nlohmann::json::parse(data);
    if (json["type"].get<std::string>() == "cmd_vel") {
      FromJson(json);
      // ROS_INFO("[%s] Received ZMQ velocity command: linear[%.3f, %.3f, %.3f], angular[%.3f, %.3f, %.3f]", name_.c_str(),
      //          data_.linear.x, data_.linear.y, data_.linear.z, data_.angular.x, data_.angular.y, data_.angular.z);
      // 转发到ROS
      PublishToROS();
    }
  } catch (const nlohmann::json::exception &e) {
    LOG(ERROR) << "[" << name_ << "] JSON parse error: " << e.what();
  }
}
