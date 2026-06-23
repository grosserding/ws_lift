#pragma once

#include "message_base.h"
#include <nav_msgs/Odometry.h>

class LocMessage : public MessageBase {
 public:
  LocMessage(const std::string &name, ros::NodeHandle &nh);

  ~LocMessage() override = default;

  // 实现基类的纯虚函数
  void Init() override;

  void FromJson(const nlohmann::json &json) override;

  nlohmann::json ToJson() const override;

  void PublishToROS() override;

  void ReceiveFromROS(const std::string &topic_name) override;

  void PublishToZMQ() override;

  void ReceiveFromZMQ(const std::string &data) override;

 private:
  nav_msgs::Odometry data_;
  std::string ros_topic_;
  std::string zmq_topic_;

  // ROS回调函数
  void RosCallback(const nav_msgs::Odometry::ConstPtr &msg);
};