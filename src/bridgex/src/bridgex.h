#pragma once

#include "zmq/message_manager.h"
#include "ex/ex_client.h"
#include <ros/ros.h>
#include <memory>
#include <std_srvs/SetBool.h>
class BridgeX {
 public:
  BridgeX();

  ~BridgeX();

  void Run();

  std::shared_ptr<bridgex::ExClient> GetROSBridgeClient() const { return ex_client_; }

 private:
  ros::NodeHandle nh_;
  std::unique_ptr<MessageManager> manager_;
  std::shared_ptr<bridgex::ExClient> ex_client_;
  bool running_;
  ros::ServiceServer start_service_;

  // 服务回调
  static void SetupServices();

  bool StartServiceCallback(const std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) const;

  void Stop();
};
