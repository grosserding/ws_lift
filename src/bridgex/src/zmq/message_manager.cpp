#include "message_manager.h"
#include "log.h"

MessageManager::MessageManager(ros::NodeHandle &nh) : nh_(nh) {
  // 从参数服务器获取ZMQ端点配置，如果未设置则使用默认值
  std::string pub_endpoint = "tcp://127.0.0.1:4565";  // 默认发布端点
  std::string sub_endpoint = "tcp://127.0.0.1:4566";  // 默认订阅端点

  nh_.param("pub_endpoint", pub_endpoint, pub_endpoint);
  nh_.param("sub_endpoint", sub_endpoint, sub_endpoint);
  MessageBase::SetEndpoints(pub_endpoint, sub_endpoint);
  LOG(INFO) << "MessageManager initialized with pub_endpoint: " << pub_endpoint << ", sub_endpoint: " << sub_endpoint;
}

MessageManager::~MessageManager() { StopAll(); }

void MessageManager::StartAll() {
  for (auto &[name, msg_type] : message_types_) {
    msg_type->Start();
  }
  LOG(INFO) << "All message types started";
}

void MessageManager::StopAll() {
  for (auto &[name, msg_type] : message_types_) {
    msg_type->Stop();
  }
  LOG(INFO) << "All message types stopped";
}

void MessageManager::Start(const std::string &name) {
  auto it = message_types_.find(name);
  if (it != message_types_.end()) {
    it->second->Start();
    LOG(INFO) << "Message type " << name << " started";
  } else {
    LOG(WARNING) << "Message type " << name << " not found";
  }
}

void MessageManager::Stop(const std::string &name) {
  auto it = message_types_.find(name);
  if (it != message_types_.end()) {
    it->second->Stop();
    LOG(INFO) << "Message type " << name << " stopped";
  } else {
    LOG(WARNING) << "Message type " << name << " not found";
  }
}

std::vector<std::string> MessageManager::GetMessageTypes() const {
  std::vector<std::string> types;
  for (const auto &[name, msg_type] : message_types_) {
    types.push_back(name);
  }
  return types;
}

std::shared_ptr<MessageBase> MessageManager::GetMessageType(const std::string &name) {
  auto it = message_types_.find(name);
  if (it != message_types_.end()) {
    return it->second;
  }
  return nullptr;
}
