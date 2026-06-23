#include "loc_message.h"
#include "log.h"

LocMessage::LocMessage(const std::string &name, ros::NodeHandle &nh)
    : MessageBase(name, nh), ros_topic_(name), zmq_topic_(name) {
  // 初始化里程计数据
  data_.header.frame_id = "odom";
  data_.child_frame_id = "base_link";
  data_.pose.pose.position.x = 0.0;
  data_.pose.pose.position.y = 0.0;
  data_.pose.pose.position.z = 0.0;
  data_.pose.pose.orientation.w = 1.0;
  data_.pose.pose.orientation.x = 0.0;
  data_.pose.pose.orientation.y = 0.0;
  data_.pose.pose.orientation.z = 0.0;
  data_.twist.twist.linear.x = 0.0;
  data_.twist.twist.linear.y = 0.0;
  data_.twist.twist.linear.z = 0.0;
  data_.twist.twist.angular.x = 0.0;
  data_.twist.twist.angular.y = 0.0;
  data_.twist.twist.angular.z = 0.0;
}

void LocMessage::Init() {
  // 初始化ROS发布者
  ros_pub_ = nh_.advertise<nav_msgs::Odometry>(ros_topic_ + "_bridgex", 10);
  // 初始化ROS订阅者
  ros_sub_ = nh_.subscribe(ros_topic_, 10, &LocMessage::RosCallback, this);
  // 设置ZMQ订阅
  SubFromZMQ(zmq_topic_, std::bind(&LocMessage::ReceiveFromZMQ, this, std::placeholders::_1));
  LOG(INFO) << "[" << name_ << "] initialized, ROS topic: " << ros_topic_;
}

void LocMessage::FromJson(const nlohmann::json &json) {
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

  // 解析线速度
  if (json.contains("linear_velocity")) {
    const auto &linear = json["linear_velocity"];
    if (linear.contains("x")) {
      data_.twist.twist.linear.x = linear["x"].get<double>();
    }
    if (linear.contains("y")) {
      data_.twist.twist.linear.y = linear["y"].get<double>();
    }
    if (linear.contains("z")) {
      data_.twist.twist.linear.z = linear["z"].get<double>();
    }
  }

  // 解析角速度
  if (json.contains("angular_velocity")) {
    const auto &angular = json["angular_velocity"];
    if (angular.contains("x")) {
      data_.twist.twist.angular.x = angular["x"].get<double>();
    }
    if (angular.contains("y")) {
      data_.twist.twist.angular.y = angular["y"].get<double>();
    }
    if (angular.contains("z")) {
      data_.twist.twist.angular.z = angular["z"].get<double>();
    }
  }

  // 解析时间戳和坐标系
  if (json.contains("timestamp")) {
    data_.header.stamp.fromSec(json["timestamp"].get<double>());
  }
  if (json.contains("frame_id")) {
    data_.header.frame_id = json["frame_id"].get<std::string>();
  }
  if (json.contains("child_frame_id")) {
    data_.child_frame_id = json["child_frame_id"].get<std::string>();
  }
}

nlohmann::json LocMessage::ToJson() const {
  nlohmann::json json;
  json["type"] = "loc";

  // 位置
  json["position"]["x"] = data_.pose.pose.position.x;
  json["position"]["y"] = data_.pose.pose.position.y;
  json["position"]["z"] = data_.pose.pose.position.z;

  // 方向四元数
  json["orientation"]["w"] = data_.pose.pose.orientation.w;
  json["orientation"]["x"] = data_.pose.pose.orientation.x;
  json["orientation"]["y"] = data_.pose.pose.orientation.y;
  json["orientation"]["z"] = data_.pose.pose.orientation.z;

  // 线速度
  json["linear_velocity"]["x"] = data_.twist.twist.linear.x;
  json["linear_velocity"]["y"] = data_.twist.twist.linear.y;
  json["linear_velocity"]["z"] = data_.twist.twist.linear.z;

  // 角速度
  json["angular_velocity"]["x"] = data_.twist.twist.angular.x;
  json["angular_velocity"]["y"] = data_.twist.twist.angular.y;
  json["angular_velocity"]["z"] = data_.twist.twist.angular.z;

  json["timestamp"] = data_.header.stamp.toSec();
  json["frame_id"] = data_.header.frame_id;
  json["child_frame_id"] = data_.child_frame_id;

  return json;
}

void LocMessage::PublishToROS() {
  if (!is_running_) return;
  ros_pub_.publish(data_);
}

void LocMessage::RosCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  data_ = *msg;
  // ROS_INFO("[%s] Received ROS odometry: position[%.3f, %.3f, %.3f]",
  //          name_.c_str(),
  //          data_.pose.pose.position.x, data_.pose.pose.position.y,
  //          data_.pose.pose.position.z);
  PublishToZMQ();
}

void LocMessage::PublishToZMQ() {
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

void LocMessage::ReceiveFromROS(const std::string &topic_name) {
  // 这个函数在RosCallback中实现
}

void LocMessage::ReceiveFromZMQ(const std::string &data) {
  try {
    nlohmann::json json = nlohmann::json::parse(data);
    if (json["type"].get<std::string>() == "loc") {
      FromJson(json);
      // ROS_INFO("[%s] Received ZMQ odometry: position[%.3f, %.3f, %.3f]",
      //          name_.c_str(),
      //          data_.pose.pose.position.x, data_.pose.pose.position.y,
      //          data_.pose.pose.position.z);
      // 转发到ROS
      PublishToROS();
    }
  } catch (const nlohmann::json::exception &e) {
    LOG(ERROR) << "[" << name_ << "] JSON parse error: " << e.what();
  }
}