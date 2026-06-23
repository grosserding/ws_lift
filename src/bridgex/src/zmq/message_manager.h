#pragma once

#include "message_base.h"
#include <map>
#include <memory>
#include <vector>
#include <functional>

#include "log.h"

class MessageManager {
 public:
  MessageManager(ros::NodeHandle &nh);

  ~MessageManager();

  // 消息类型注册
  template <typename T>
  void RegisterMessageType(const std::string &name) {
    factories_[name] = [](const std::string &msg_name, ros::NodeHandle &nh) {
      return std::make_shared<T>(msg_name, nh);
    };
    // 创建消息类型实例
    auto msg_type = factories_[name](name, nh_);
    message_types_[name] = msg_type;
    LOG(INFO) << "Registered message type: " << name;
  }

  // 启动和停止所有消息类型
  void StartAll();

  void StopAll();

  // 启动和停止特定消息类型
  void Start(const std::string &name);

  void Stop(const std::string &name);

  // 获取消息类型列表
  std::vector<std::string> GetMessageTypes() const;

  // 获取特定消息类型
  std::shared_ptr<MessageBase> GetMessageType(const std::string &name);

 private:
  ros::NodeHandle &nh_;
  std::map<std::string, std::shared_ptr<MessageBase> > message_types_;

  // 消息类型工厂函数
  using MessageFactory = std::function<std::shared_ptr<MessageBase>(const std::string &, ros::NodeHandle &)>;
  std::map<std::string, MessageFactory> factories_;
};
