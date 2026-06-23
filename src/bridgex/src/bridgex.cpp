#include "bridgex.h"
#include <ros/ros.h>
#include <csignal>

#include <cmd_vel_message.h>
#include <imu_message.h>
#include <loc_message.h>
#include <amclpose_message.h>
static volatile sig_atomic_t shutdown_requested = 0;

void SignalHandler(int sig) { shutdown_requested = 1; }

BridgeX::BridgeX() : running_(false) {
  manager_ = std::make_unique<MessageManager>(nh_);
  manager_->RegisterMessageType<CmdVelMessage>("cmd_vel");
  // manager_->RegisterMessageType<ImuMessage>("imu");
  // manager_->RegisterMessageType<LocMessage>("odom");
  // manager_->RegisterMessageType<AMCLPoseMessage>("amcl_pose");
  ex_client_ = std::make_shared<bridgex::ExClient>(nh_);
  SetupServices();
  LOG(INFO) << "BridgeX initialized";
}

BridgeX::~BridgeX() { Stop(); }

void BridgeX::SetupServices() {
  // 这里可以添加ROS服务来控制消息类型的启动和停止
  // 例如：start_service_ = nh_.advertiseService("start_message_type", &BridgeX::StartServiceCallback, this);
}

bool BridgeX::StartServiceCallback(const std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) const {
  if (req.data) {
    manager_->StartAll();
    res.success = true;
    res.message = "All message types started";
  } else {
    manager_->StopAll();
    res.success = true;
    res.message = "All message types stopped";
  }
  return true;
}

void BridgeX::Run() {
  manager_->StartAll();
  ex_client_->Start();
  running_ = true;
  LOG(INFO)<<"BridgeX is running";
  ros::Rate rate(10);  // 10Hz
  while (!shutdown_requested && ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  Stop();
}
void BridgeX::Stop() {
  running_ = false;
  manager_->StopAll();
  ex_client_->Stop();
  LOG(INFO) << "BridgeX stopped";
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "bridgeX_node", ros::init_options::NoSigintHandler);

  // 注册信号处理器
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);
  LogFileManager log_manager;
  log_manager.start();
  try {
    BridgeX node;
    node.Run();
  } catch (const std::exception &e) {
    LOG(ERROR) << "BridgeX error: " << e.what();
    return -1;
  }

  ros::shutdown();
  LOG(INFO) << "Shutting down log...";
  log_manager.stop();
  return 0;
}
