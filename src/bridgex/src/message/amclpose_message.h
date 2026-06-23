#pragma once

#include "message_base.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class AMCLPoseMessage : public MessageBase {
 public:
  AMCLPoseMessage(const std::string &name, ros::NodeHandle &nh);

  ~AMCLPoseMessage() override = default;

  // 实现基类的纯虚函数
  void Init() override;

  void FromJson(const nlohmann::json &json) override;

  nlohmann::json ToJson() const override;

  void PublishToROS() override;

  void ReceiveFromROS(const std::string &topic_name) override;

  void PublishToZMQ() override;

  void ReceiveFromZMQ(const std::string &data) override;

 private:
  geometry_msgs::PoseWithCovarianceStamped data_;
  std::string ros_topic_;
  std::string zmq_topic_;

  // ROS回调函数
  void RosCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
};