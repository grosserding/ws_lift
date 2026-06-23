#include "imu_message.h"
#include "log.h"

ImuMessage::ImuMessage(const std::string &name, ros::NodeHandle &nh)
    : MessageBase(name, nh), ros_topic_(name), zmq_topic_(name) {
  // 初始化IMU数据
  data_.header.frame_id = "imu_link";
  data_.orientation.w = 1.0;
  data_.orientation.x = 0.0;
  data_.orientation.y = 0.0;
  data_.orientation.z = 0.0;
  data_.angular_velocity.x = 0.0;
  data_.angular_velocity.y = 0.0;
  data_.angular_velocity.z = 0.0;
  data_.linear_acceleration.x = 0.0;
  data_.linear_acceleration.y = 0.0;
  data_.linear_acceleration.z = 0.0;
}

void ImuMessage::Init() {
  // 初始化ROS发布者
  ros_pub_ = nh_.advertise<sensor_msgs::Imu>(ros_topic_ + "_bridgex", 10);
  // 初始化ROS订阅者
  ros_sub_ = nh_.subscribe(ros_topic_, 10, &ImuMessage::RosCallback, this);
  // 设置ZMQ订阅
  SubFromZMQ(zmq_topic_, std::bind(&ImuMessage::ReceiveFromZMQ, this, std::placeholders::_1));
  LOG(INFO) << "[" << name_ << "] initialized, ROS topic: " << ros_topic_;
}

void ImuMessage::FromJson(const nlohmann::json &json) {
  // 解析方向四元数
  if (json.contains("orientation")) {
    const auto &ori = json["orientation"];
    if (ori.contains("w")) {
      data_.orientation.w = ori["w"].get<double>();
    }
    if (ori.contains("x")) {
      data_.orientation.x = ori["x"].get<double>();
    }
    if (ori.contains("y")) {
      data_.orientation.y = ori["y"].get<double>();
    }
    if (ori.contains("z")) {
      data_.orientation.z = ori["z"].get<double>();
    }
  }

  // 解析角速度
  if (json.contains("angular_velocity")) {
    const auto &angular = json["angular_velocity"];
    if (angular.contains("x")) {
      data_.angular_velocity.x = angular["x"].get<double>();
    }
    if (angular.contains("y")) {
      data_.angular_velocity.y = angular["y"].get<double>();
    }
    if (angular.contains("z")) {
      data_.angular_velocity.z = angular["z"].get<double>();
    }
  }

  // 解析线加速度
  if (json.contains("linear_velocity")) {
    const auto &linear = json["linear_velocity"];
    if (linear.contains("x")) {
      data_.linear_acceleration.x = linear["x"].get<double>();
    }
    if (linear.contains("y")) {
      data_.linear_acceleration.y = linear["y"].get<double>();
    }
    if (linear.contains("z")) {
      data_.linear_acceleration.z = linear["z"].get<double>();
    }
  }

  // 解析时间戳和坐标系
  if (json.contains("timestamp")) {
    const auto &timestamp  = json["timestamp"];
    data_.header.stamp.sec = timestamp["sec"].get<int64_t>();
    data_.header.stamp.nsec = timestamp["nsec"].get<int64_t>();
  }
  if (json.contains("frame_id")) {
    data_.header.frame_id = json["frame_id"].get<std::string>();
  }
}

nlohmann::json ImuMessage::ToJson() const {
  nlohmann::json json;
  json["type"] = "imu";

  // 方向四元数
  json["orientation"]["w"] = data_.orientation.w;
  json["orientation"]["x"] = data_.orientation.x;
  json["orientation"]["y"] = data_.orientation.y;
  json["orientation"]["z"] = data_.orientation.z;

  // 角速度
  json["angular_velocity"]["x"] = data_.angular_velocity.x;
  json["angular_velocity"]["y"] = data_.angular_velocity.y;
  json["angular_velocity"]["z"] = data_.angular_velocity.z;

  // 线加速度
  json["linear_acceleration"]["x"] = data_.linear_acceleration.x;
  json["linear_acceleration"]["y"] = data_.linear_acceleration.y;
  json["linear_acceleration"]["z"] = data_.linear_acceleration.z;

  json["timestamp"] = data_.header.stamp.toSec();
  json["frame_id"] = data_.header.frame_id;

  return json;
}

void ImuMessage::PublishToROS() {
  if (!is_running_) return;
  ros_pub_.publish(data_);
}

void ImuMessage::RosCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  data_ = *msg;
  // ROS_INFO("[%s] Received ROS IMU data: orientation[%.3f, %.3f, %.3f, %.3f]",
  //          name_.c_str(),
  //          data_.orientation.w, data_.orientation.x,
  //          data_.orientation.y, data_.orientation.z);
  // PublishToZMQ();
}

void ImuMessage::PublishToZMQ() {
  if (!is_running_) return;
  try {
    nlohmann::json json = ToJson();
    std::string json_str = json.dump();
    PubToZMQ(zmq_topic_, json_str);
    // ROS_INFO("[%s] ZMQ publish: %s", name_.c_str(), json_str.c_str());
  } catch (const std::exception &e) {
    LOG(ERROR) << "[" << name_ << "] ZMQ publish error: " << e.what();
  }
}

void ImuMessage::ReceiveFromROS(const std::string &topic_name) {
  // 这个函数在RosCallback中实现
}

void ImuMessage::ReceiveFromZMQ(const std::string &data) {
  try {
    nlohmann::json json = nlohmann::json::parse(data);
    if (json["type"].get<std::string>() == "imu") {
      FromJson(json);
      // ROS_INFO("[%s] Received ZMQ IMU data: orientation[%.3f, %.3f, %.3f, %.3f]",
      //          name_.c_str(),
      //          data_.orientation.w, data_.orientation.x,
      //          data_.orientation.y, data_.orientation.z);
      // 转发到ROS
      PublishToROS();
    }
  } catch (const nlohmann::json::exception &e) {
    LOG(ERROR) << "[" << name_ << "] JSON parse error: " << e.what();
  }
}