#pragma once

#include <ros/ros.h>
#include <zmq.hpp>
#include <string>
#include "json.h"
#include <memory>
#include <functional>
#include <thread>
#include <mutex>
#include <vector>

class MessageBase {
 public:
  MessageBase(std::string name, ros::NodeHandle &nh);

  virtual ~MessageBase() = default;

  // 静态方法设置端点配置
  static void SetEndpoints(const std::string &pub_endpoint, const std::string &sub_endpoint);

  // 纯虚函数，子类必须实现
  virtual void Init() = 0;

  virtual void FromJson(const nlohmann::json &json) = 0;

  virtual nlohmann::json ToJson() const = 0;

  virtual void PublishToROS() = 0;

  // 这里实现的各种方式获取ROS消息
  virtual void ReceiveFromROS(const std::string &) = 0;

  virtual void PublishToZMQ() = 0;

  // 这里实现的各种方式获取ZMQ消息
  virtual void ReceiveFromZMQ(const std::string &) = 0;

  // 公共接口
  const std::string &GetName() const { return name_; }

  void Start();

  void Stop();

  bool IsRunning() const { return is_running_; }

 protected:
  std::string name_;
  ros::NodeHandle &nh_;
  ros::Publisher ros_pub_;
  ros::Subscriber ros_sub_;
  bool is_running_;

  // ZMQ通信相关
  zmq::context_t context_;
  std::unique_ptr<zmq::socket_t> publisher_;
  std::unique_ptr<zmq::socket_t> subscriber_;
  static std::string pub_endpoint_;
  static std::string sub_endpoint_;
  std::mutex mutex_;
  std::vector<std::thread> sub_threads_;

  // ZMQ通信方法
  void SetupZMQ();

  void CleanupZMQ();

  void PubToZMQ(const std::string &topic, const std::string &data);

  void SubFromZMQ(const std::string &topic, const std::function<void(const std::string &)> &callback);
};
