#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <mutex>
#include <thread>
#include <atomic>
#include <functional>

namespace cetcrobot {

// ========== Lite3 命令码 ==========
constexpr uint32_t CMD_HEARTBEAT_LITE3 = 0x21040001;  // 心跳
constexpr uint32_t CMD_STAND_SIT = 0x21010202;        // 起立/坐下
constexpr uint32_t CMD_HANDLE_CONTROL = 0x21010C02;   // 手柄控制(正常模式)
constexpr uint32_t CMD_NAV_CONTROL = 0x21010C03;      // 导航控制模式
constexpr uint32_t CMD_STAND_STILL = 0x21010D05;      // 原地模式
constexpr uint32_t CMD_MOVE_MODE = 0x21010D06;        // 移动模式
constexpr uint32_t CMD_QUERY_STATUS = 0x00000901;     // 查询状态

// ========== 回调函数类型 ==========
using StatusCallbackLite3 = std::function<void(const std::vector<uint8_t>&)>;

// ========== Lite3Controller类 ==========
class Lite3Controller {
 public:
  explicit Lite3Controller(std::string  ip = "192.168.1.120", int port = 43893);
  ~Lite3Controller();

  // ========== 连接管理 ==========
  bool connect();
  void disconnect();
  bool isConnected() const;

  // ========== 回调注册 ==========
  void setStatusCallback(StatusCallbackLite3 callback);

  // ========== 控制指令 ==========
  // 起立/坐下
  bool standSit();
  bool standUp() { return standSit(); }
  bool sitDown() { return standSit(); }

  // 导航控制模式
  bool navControlMode();

  // 正常模式(手柄控制)
  bool handleControlMode();

  // 心跳
  bool sendHeartbeat();
  void startHeartbeat(int interval_seconds = 1);
  void stopHeartbeat();

 private:
  // ========== 协议封装 ==========
  static std::vector<uint8_t> buildPacket(uint32_t code, uint32_t parameters_size = 0, uint32_t type = 0);

  // ========== 通信 ==========
  bool sendPacket(const std::vector<uint8_t>& packet) const;
  void receiveThread();
  void heartbeatThread();

 private:
  std::string ip_;
  int port_;
  int sockfd_;
  bool connected_;

  std::thread receiveThread_;
  std::atomic<bool> running_;

  StatusCallbackLite3 statusCallback_;
};

} // namespace cetcrobot

using cetcrobot::Lite3Controller;