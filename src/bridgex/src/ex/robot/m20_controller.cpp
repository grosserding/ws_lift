#include "m20_controller.h"
#include "log.h"
#include "json.h"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <iostream>

using json = nlohmann::json;

namespace cetcrobot {

// ========== 构造函数与析构函数 ==========

M20Controller::M20Controller(const std::string& ip, int port)
    : ip_(ip),
      port_(port),
      sockfd_(-1),
      msgId_(0),
      running_(false) {
}

M20Controller::~M20Controller() {
  disconnect();
}

// ========== 连接管理 ==========

bool M20Controller::connect() {
  if (isConnected()) {
    return true;
  }

  sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd_ < 0) {
    LOG(ERROR) << "[M20Controller] Failed to create socket";
    return false;
  }

  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port_);

  if (inet_pton(AF_INET, ip_.c_str(), &server_addr.sin_addr) <= 0) {
    LOG(ERROR) << "[M20Controller] Invalid address: " << ip_;
    close(sockfd_);
    sockfd_ = -1;
    return false;
  }

  if (::connect(sockfd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    LOG(ERROR) << "[M20Controller] Failed to connect to " << ip_ << ":" << port_;
    close(sockfd_);
    sockfd_ = -1;
    return false;
  }

  LOG(INFO) << "[M20Controller] Connected to M20 at " << ip_ << ":" << port_;

  // 启动接收线程和心跳线程
  running_ = true;
  receiveThread_ = std::thread(&M20Controller::receiveThread, this);
  heartbeatThread_ = std::thread(&M20Controller::heartbeatThread, this);

  return true;
}

void M20Controller::disconnect() {
  running_ = false;

  if (receiveThread_.joinable()) {
    receiveThread_.join();
  }
  if (heartbeatThread_.joinable()) {
    heartbeatThread_.join();
  }

  if (sockfd_ != -1) {
    close(sockfd_);
    sockfd_ = -1;
  }

  LOG(INFO) << "[M20Controller] Disconnected";
}

bool M20Controller::isConnected() const {
  return sockfd_ != -1;
}

// ========== 回调注册 ==========

void M20Controller::setBasicStatusCallback(BasicStatusCallback callback) {
  basicStatusCallback_ = callback;
}

void M20Controller::setMotionStatusCallback(MotionStatusCallback callback) {
  motionStatusCallback_ = callback;
}

void M20Controller::setDeviceStatusCallback(DeviceStatusCallback callback) {
  deviceStatusCallback_ = callback;
}

void M20Controller::setErrorStatusCallback(ErrorStatusCallback callback) {
  errorStatusCallback_ = callback;
}

// ========== 控制指令 - 心跳 ==========

bool M20Controller::sendHeartbeat() {
  std::string json = buildHeartbeatJSON();
  std::vector<uint8_t> packet = buildPacket(json, FORMAT_JSON);
  return sendPacket(packet);
}

// ========== 控制指令 - 运动控制 ==========

bool M20Controller::setMotionState(MotionState state) {
  std::string json = buildMotionStateJSON(state);
  std::vector<uint8_t> packet = buildPacket(json, FORMAT_JSON);
  return sendPacket(packet);
}

bool M20Controller::setGait(uint32_t gait) {
  std::string json = buildGaitJSON(gait);
  std::vector<uint8_t> packet = buildPacket(json, FORMAT_JSON);
  return sendPacket(packet);
}

bool M20Controller::setAxisSpeed(double x, double y, double z,
                                  double roll, double pitch, double yaw) {
  std::string json = buildAxisSpeedJSON(x, y, z, roll, pitch, yaw);
  std::vector<uint8_t> packet = buildPacket(json, FORMAT_JSON);
  return sendPacket(packet);
}

bool M20Controller::setAutonomousCharge(int32_t charge) {
  // 参数校验：只允许 0(停止充电), 1(开始充电), 2(清除充电状态)
  if (charge != 0 && charge != 1 && charge != 2) {
    LOG(ERROR) << "[M20Controller] Invalid charge value: " << charge
               << ". Must be 0 (stop), 1 (start), or 2 (clear)";
    return false;
  }
  std::string json = buildAutonomousChargeJSON(charge);
  std::vector<uint8_t> packet = buildPacket(json, FORMAT_JSON);
  return sendPacket(packet);
}

// ========== 充电状态等待 ==========

bool M20Controller::waitForEnterCharge(int timeoutMs) {
  // 等待进入充电状态：仅充电中(2)
  auto start = std::chrono::steady_clock::now();
  int pollIntervalMs = 500;  // 每500ms查询一次

  while (true) {
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();
    if (elapsed >= timeoutMs) {
      LOG(ERROR) << "[M20Controller] waitForEnterCharge timeout after "
                 << timeoutMs << "ms";
      return false;
    }

    BasicStatus status = getBasicStatus();
    ChargeState chargeState = static_cast<ChargeState>(status.charge);

    if (chargeState == ChargeState::CHARGING) {
      LOG(INFO) << "[M20Controller] Enter charge state: charging (2)";
      return true;
    }

    if (chargeState == ChargeState::GOING_TO_PILE) {
      LOG(INFO) << "[M20Controller] Going to charging pile, waiting for actual charging...";
    } else {
      LOG(INFO) << "[M20Controller] Waiting for enter charge, current state: "
                << status.charge;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
  }
}

bool M20Controller::waitForExitCharge(int timeoutMs) {
  // 等待退出充电状态：空闲(0)
  auto start = std::chrono::steady_clock::now();
  int pollIntervalMs = 500;  // 每500ms查询一次

  while (true) {
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();
    if (elapsed >= timeoutMs) {
      LOG(ERROR) << "[M20Controller] waitForExitCharge timeout after "
                 << timeoutMs << "ms";
      return false;
    }

    BasicStatus status = getBasicStatus();
    ChargeState chargeState = static_cast<ChargeState>(status.charge);

    if (chargeState == ChargeState::IDLE) {
      LOG(INFO) << "[M20Controller] Exit charge state: idle";
      return true;
    }

    LOG(INFO) << "[M20Controller] Waiting for exit charge, current state: "
              << status.charge;

    std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
  }
}

// ========== 控制指令 - 系统配置 ==========

bool M20Controller::setUsageMode(UsageMode mode) {
  std::string json = buildUsageModeJSON(mode);
  std::vector<uint8_t> packet = buildPacket(json, FORMAT_JSON);
  return sendPacket(packet);
}

bool M20Controller::setLight(int32_t front, int32_t back) {
  std::string json = buildLightJSON(front, back);
  std::vector<uint8_t> packet = buildPacket(json, FORMAT_JSON);
  return sendPacket(packet);
}

bool M20Controller::setSleepMode(const SleepConfig& config) {
  std::string json = buildSleepModeJSON(config);
  std::vector<uint8_t> packet = buildPacket(json, FORMAT_JSON);
  return sendPacket(packet);
}

bool M20Controller::querySleepMode(SleepConfig& config) {
  std::string json = buildSleepQueryJSON();
  std::vector<uint8_t> packet = buildPacket(json, FORMAT_JSON);

  if (!sendPacket(packet)) {
    return false;
  }

  // 等待响应
  std::vector<uint8_t> response;
  if (!receivePacket(response, 2000)) {
    return false;
  }

  std::string asdu;
  uint8_t format;
  if (!parsePacket(response, asdu, format)) {
    return false;
  }

  return parseSleepConfigJSON(asdu, config);
}

// ========== 控制指令 - 导航（仅M20 Pro支持）==========

bool M20Controller::initLocation(double posX, double posY, double posZ, double yaw) {
  std::string json = buildInitLocationJSON(posX, posY, posZ, yaw);
  std::vector<uint8_t> packet = buildPacket(json, FORMAT_JSON);

  if (!sendPacket(packet)) {
    return false;
  }

  // 等待响应
  std::vector<uint8_t> response;
  if (!receivePacket(response, 2000)) {
    return false;
  }

  std::string asdu;
  uint8_t format;
  if (!parsePacket(response, asdu, format)) {
    return false;
  }

  int32_t errorCode;
  if (!parseInitLocationResponseJSON(asdu, errorCode)) {
    return false;
  }

  return errorCode == 0;
}

bool M20Controller::getLocationInfo(LocationInfo& info) {
  std::string json = buildLocationQueryJSON();
  std::vector<uint8_t> packet = buildPacket(json, FORMAT_JSON);

  if (!sendPacket(packet)) {
    return false;
  }

  // 等待响应
  std::vector<uint8_t> response;
  if (!receivePacket(response, 2000)) {
    return false;
  }

  std::string asdu;
  uint8_t format;
  if (!parsePacket(response, asdu, format)) {
    return false;
  }

  return parseLocationInfoJSON(asdu, info);
}

bool M20Controller::getPerceptionStatus(PerceptionStatus& status) {
  json j = {
    {"PatrolDevice", {
      {"Type", 2002},
      {"Command", 1},
      {"Time", getCurrentTime()},
      {"Items", json::object()}
    }}
  };
  std::string jsonStr = j.dump();
  std::vector<uint8_t> packet = buildPacket(jsonStr, FORMAT_JSON);

  if (!sendPacket(packet)) {
    return false;
  }

  // 等待响应
  std::vector<uint8_t> response;
  if (!receivePacket(response, 2000)) {
    return false;
  }

  std::string asdu;
  uint8_t format;
  if (!parsePacket(response, asdu, format)) {
    return false;
  }

  return parsePerceptionStatusJSON(asdu, status);
}

bool M20Controller::sendNavigationTask(const NavigationTarget& target) {
  std::string json = buildNavigationTaskJSON(target);
  std::vector<uint8_t> packet = buildPacket(json, FORMAT_JSON);

  if (!sendPacket(packet)) {
    return false;
  }

  // 等待响应
  std::vector<uint8_t> response;
  if (!receivePacket(response, 5000)) {
    return false;
  }

  std::string asdu;
  uint8_t format;
  if (!parsePacket(response, asdu, format)) {
    return false;
  }

  NavigationStatus status;
  if (!parseNavigationStatusJSON(asdu, status)) {
    return false;
  }

  return status.errorCode == 0;
}

bool M20Controller::cancelNavigationTask() {
  std::string json = buildCancelNavigationJSON();
  std::vector<uint8_t> packet = buildPacket(json, FORMAT_JSON);

  if (!sendPacket(packet)) {
    return false;
  }

  // 等待响应
  std::vector<uint8_t> response;
  if (!receivePacket(response, 2000)) {
    return false;
  }

  std::string asdu;
  uint8_t format;
  if (!parsePacket(response, asdu, format)) {
    return false;
  }

  int32_t errorCode;
  if (!parseCancelNavResponseJSON(asdu, errorCode)) {
    return false;
  }

  return errorCode == 0;
}

bool M20Controller::queryNavigationStatus(NavigationStatus& status) {
  std::string json = buildNavQueryJSON();
  std::vector<uint8_t> packet = buildPacket(json, FORMAT_JSON);

  if (!sendPacket(packet)) {
    return false;
  }

  // 等待响应
  std::vector<uint8_t> response;
  if (!receivePacket(response, 2000)) {
    return false;
  }

  std::string asdu;
  uint8_t format;
  if (!parsePacket(response, asdu, format)) {
    return false;
  }

  return parseNavigationStatusJSON(asdu, status);
}

// ========== 状态查询 ==========

BasicStatus M20Controller::getBasicStatus() const {
  std::lock_guard<std::mutex> lock(statusMutex_);
  return basicStatus_;
}

MotionStatus M20Controller::getMotionStatus() const {
  std::lock_guard<std::mutex> lock(statusMutex_);
  return motionStatus_;
}

MotorStatus M20Controller::getMotorStatus() const {
  std::lock_guard<std::mutex> lock(statusMutex_);
  return motorStatus_;
}

BatteryStatus M20Controller::getBatteryStatus() const {
  std::lock_guard<std::mutex> lock(statusMutex_);
  return batteryStatus_;
}

GPSInfo M20Controller::getGPSInfo() const {
  std::lock_guard<std::mutex> lock(statusMutex_);
  return gpsInfo_;
}

// ========== 协议封装 ==========

std::vector<uint8_t> M20Controller::buildPacket(const std::string& asdu, uint8_t format) {
  std::vector<uint8_t> packet;

  // 协议头部（16字节）
  packet.push_back(SYNC_BYTE_1);
  packet.push_back(SYNC_BYTE_2);
  packet.push_back(SYNC_BYTE_3);
  packet.push_back(SYNC_BYTE_4);

  // ASDU长度（小端）
  uint16_t length = static_cast<uint16_t>(asdu.size());
  packet.push_back(length & 0xFF);
  packet.push_back((length >> 8) & 0xFF);

  // 报文ID（小端）
  msgId_ = (msgId_ + 1) % 65536;
  packet.push_back(msgId_ & 0xFF);
  packet.push_back((msgId_ >> 8) & 0xFF);

  // ASDU格式
  packet.push_back(format);

  // 预留字节（7字节）
  for (int i = 0; i < 7; i++) {
    packet.push_back(0);
  }

  // ASDU数据
  packet.insert(packet.end(), asdu.begin(), asdu.end());

  return packet;
}

bool M20Controller::parsePacket(const std::vector<uint8_t>& packet,
                                std::string& asdu, uint8_t& format) {
  if (packet.size() < 16) {
    LOG(ERROR) << "[M20Controller] Packet too short: " << packet.size();
    return false;
  }

  // 检查同步字符
  if (packet[0] != SYNC_BYTE_1 || packet[1] != SYNC_BYTE_2 ||
      packet[2] != SYNC_BYTE_3 || packet[3] != SYNC_BYTE_4) {
    LOG(ERROR) << "[M20Controller] Invalid sync bytes";
    return false;
  }

  // 解析长度
  uint16_t length = packet[4] | (packet[5] << 8);

  // 检查数据完整性
  if (packet.size() < 16 + length) {
    LOG(ERROR) << "[M20Controller] Incomplete packet: expected " << (16 + length)
              << ", got " << packet.size();
    return false;
  }

  // 提取格式
  format = packet[8];

  // 提取ASDU
  asdu.assign(packet.begin() + 16, packet.begin() + 16 + length);

  return true;
}

// ========== JSON构建 ==========

std::string cetcrobot::M20Controller::buildHeartbeatJSON() {
  json j = {
    {"PatrolDevice", {
      {"Type", 100},
      {"Command", 100},
      {"Time", getCurrentTime()},
      {"Items", json::object()}
    }}
  };
  return j.dump();
}

std::string cetcrobot::M20Controller::buildMotionStateJSON(MotionState state) {
  json j = {
    {"PatrolDevice", {
      {"Type", 2},
      {"Command", 22},
      {"Time", getCurrentTime()},
      {"Items", {
        {"MotionParam", static_cast<int32_t>(state)}
      }}
    }}
  };
  return j.dump();
}

std::string cetcrobot::M20Controller::buildGaitJSON(uint32_t gait) {
  json j = {
    {"PatrolDevice", {
      {"Type", 2},
      {"Command", 23},
      {"Time", getCurrentTime()},
      {"Items", {
        {"GaitParam", gait}
      }}
    }}
  };
  return j.dump();
}

std::string cetcrobot::M20Controller::buildAxisSpeedJSON(double x, double y, double z,
                                               double roll, double pitch, double yaw) {
  json j = {
    {"PatrolDevice", {
      {"Type", 2},
      {"Command", 21},
      {"Time", getCurrentTime()},
      {"Items", {
        {"X", x},
        {"Y", y},
        {"Z", z},
        {"Roll", roll},
        {"Pitch", pitch},
        {"Yaw", yaw}
      }}
    }}
  };
  return j.dump();
}

std::string cetcrobot::M20Controller::buildAutonomousChargeJSON(int32_t charge) {
  json j = {
    {"PatrolDevice", {
      {"Type", 2},
      {"Command", 24},
      {"Time", getCurrentTime()},
      {"Items", {
        {"Charge", charge}
      }}
    }}
  };
  return j.dump();
}

std::string cetcrobot::M20Controller::buildUsageModeJSON(UsageMode mode) {
  json j = {
    {"PatrolDevice", {
      {"Type", 1101},
      {"Command", 5},
      {"Time", getCurrentTime()},
      {"Items", {
        {"Mode", static_cast<int32_t>(mode)}
      }}
    }}
  };
  return j.dump();
}

std::string cetcrobot::M20Controller::buildLightJSON(int32_t front, int32_t back) {
  json j = {
    {"PatrolDevice", {
      {"Type", 1101},
      {"Command", 2},
      {"Time", getCurrentTime()},
      {"Items", {
        {"Front", front},
        {"Back", back}
      }}
    }}
  };
  return j.dump();
}

std::string cetcrobot::M20Controller::buildSleepModeJSON(const SleepConfig& config) {
  json j = {
    {"PatrolDevice", {
      {"Type", 1101},
      {"Command", 6},
      {"Time", getCurrentTime()},
      {"Items", {
        {"Sleep", config.sleep},
        {"Auto", config.autoSleep},
        {"Time", config.time}
      }}
    }}
  };
  return j.dump();
}

std::string cetcrobot::M20Controller::buildSleepQueryJSON() {
  json j = {
    {"PatrolDevice", {
      {"Type", 1101},
      {"Command", 7},
      {"Time", getCurrentTime()},
      {"Items", json::object()}
    }}
  };
  return j.dump();
}

std::string cetcrobot::M20Controller::buildInitLocationJSON(double posX, double posY, double posZ, double yaw) {
  json j = {
    {"PatrolDevice", {
      {"Type", 2101},
      {"Command", 1},
      {"Time", getCurrentTime()},
      {"Items", {
        {"PosX", posX},
        {"PosY", posY},
        {"PosZ", posZ},
        {"Yaw", yaw}
      }}
    }}
  };
  return j.dump();
}

std::string cetcrobot::M20Controller::buildNavigationTaskJSON(const NavigationTarget& target) {
  json j = {
    {"PatrolDevice", {
      {"Type", 1003},
      {"Command", 1},
      {"Time", getCurrentTime()},
      {"Items", {
        {"Value", target.value},
        {"MapID", target.mapId},
        {"PosX", target.posX},
        {"PosY", target.posY},
        {"PosZ", target.posZ},
        {"AngleYaw", target.angleYaw},
        {"PointInfo", target.pointInfo},
        {"Gait", target.gait},
        {"Speed", target.speed},
        {"Manner", target.manner},
        {"ObsMode", target.obsMode},
        {"NavMode", target.navMode}
      }}
    }}
  };
  return j.dump();
}

std::string cetcrobot::M20Controller::buildCancelNavigationJSON() {
  json j = {
    {"PatrolDevice", {
      {"Type", 1004},
      {"Command", 1},
      {"Time", getCurrentTime()},
      {"Items", json::object()}
    }}
  };
  return j.dump();
}

std::string cetcrobot::M20Controller::buildNavQueryJSON() {
  json j = {
    {"PatrolDevice", {
      {"Type", 1007},
      {"Command", 1},
      {"Time", getCurrentTime()},
      {"Items", json::object()}
    }}
  };
  return j.dump();
}

std::string cetcrobot::M20Controller::buildLocationQueryJSON() {
  json j = {
    {"PatrolDevice", {
      {"Type", 1007},
      {"Command", 2},
      {"Time", getCurrentTime()},
      {"Items", json::object()}
    }}
  };
  return j.dump();
}

// ========== JSON解析 ==========

bool M20Controller::parseBasicStatusJSON(const std::string& jsonStr, BasicStatus& status) {
  try {
    json j = json::parse(jsonStr);
    if (!j.contains("PatrolDevice") || !j["PatrolDevice"].contains("Items") ||
        !j["PatrolDevice"]["Items"].contains("BasicStatus")) {
      return false;
    }

    auto& basicStatus = j["PatrolDevice"]["Items"]["BasicStatus"];
    status.motionState = basicStatus.value("MotionState", -1);
    status.gait = basicStatus.value("Gait", static_cast<uint32_t>(-1));
    status.charge = basicStatus.value("Charge", -1);
    status.hes = basicStatus.value("HES", -1);
    status.controlUsageMode = basicStatus.value("ControlUsageMode", -1);
    status.direction = basicStatus.value("Direction", -1);
    status.ooa = basicStatus.value("OOA", -1);
    status.powerManagement = basicStatus.value("PowerManagement", -1);
    status.sleep = basicStatus.value("Sleep", -1);
    status.version = basicStatus.value("Version", "");

    return true;
  } catch (...) {
    return false;
  }
}

bool M20Controller::parseMotionStatusJSON(const std::string& jsonStr,
                                          MotionStatus& motion, MotorStatus& motor) {
  try {
    json j = json::parse(jsonStr);
    if (!j.contains("PatrolDevice") || !j["PatrolDevice"].contains("Items")) {
      return false;
    }

    auto& items = j["PatrolDevice"]["Items"];

    if (items.contains("MotionStatus")) {
      auto& motionStatus = items["MotionStatus"];
      motion.roll = motionStatus.value("Roll", 0.0);
      motion.pitch = motionStatus.value("Pitch", 0.0);
      motion.yaw = motionStatus.value("Yaw", 0.0);
      motion.omegaZ = motionStatus.value("OmegaZ", 0.0);
      motion.linearX = motionStatus.value("LinearX", 0.0);
      motion.linearY = motionStatus.value("LinearY", 0.0);
      motion.height = motionStatus.value("Height", 0.0);
      motion.payload = motionStatus.value("Payload", 0.0);
      motion.remainMile = motionStatus.value("RemainMile", 0.0);
    }

    if (items.contains("MotorStatus")) {
      auto& motorStatus = items["MotorStatus"];
      motor.leftFrontHipX = motorStatus.value("LeftFrontHipX", 0.0);
      motor.leftFrontHipY = motorStatus.value("LeftFrontHipY", 0.0);
      motor.leftFrontKnee = motorStatus.value("LeftFrontKnee", 0.0);
      motor.leftFrontWheel = motorStatus.value("LeftFrontWheel", 0.0);
      motor.rightFrontHipX = motorStatus.value("RightFrontHipX", 0.0);
      motor.rightFrontHipY = motorStatus.value("RightFrontHipY", 0.0);
      motor.rightFrontKnee = motorStatus.value("RightFrontKnee", 0.0);
      motor.rightFrontWheel = motorStatus.value("RightFrontWheel", 0.0);
      motor.leftBackHipX = motorStatus.value("LeftBackHipX", 0.0);
      motor.leftBackHipY = motorStatus.value("LeftBackHipY", 0.0);
      motor.leftBackKnee = motorStatus.value("LeftBackKnee", 0.0);
      motor.leftBackWheel = motorStatus.value("LeftBackWheel", 0.0);
      motor.rightBackHipX = motorStatus.value("RightBackHipX", 0.0);
      motor.rightBackHipY = motorStatus.value("RightBackHipY", 0.0);
      motor.rightBackKnee = motorStatus.value("RightBackKnee", 0.0);
      motor.rightBackWheel = motorStatus.value("RightBackWheel", 0.0);
    }

    return true;
  } catch (...) {
    return false;
  }
}

bool M20Controller::parseDeviceStatusJSON(const std::string& jsonStr,
                                          BatteryStatus& battery, GPSInfo& gps) {
  try {
    json j = json::parse(jsonStr);
    if (!j.contains("PatrolDevice") || !j["PatrolDevice"].contains("Items")) {
      return false;
    }

    auto& items = j["PatrolDevice"]["Items"];

    if (items.contains("BatteryStatus")) {
      auto& batteryStatus = items["BatteryStatus"];
      battery.voltageLeft = batteryStatus.value("VoltageLeft", 0.0);
      battery.voltageRight = batteryStatus.value("VoltageRight", 0.0);
      battery.batteryLevelLeft = batteryStatus.value("BatteryLevelLeft", 0.0);
      battery.batteryLevelRight = batteryStatus.value("BatteryLevelRight", 0.0);
      battery.batteryTempLeft = batteryStatus.value("battery_temperatureLeft", 0.0);
      battery.batteryTempRight = batteryStatus.value("battery_temperatureRight", 0.0);
      battery.chargeLeft = batteryStatus.value("chargeLeft", false);
      battery.chargeRight = batteryStatus.value("chargeRight", false);
    }

    if (items.contains("GPS")) {
      auto& gpsInfo = items["GPS"];
      gps.latitude = gpsInfo.value("Latitude", 0.0);
      gps.longitude = gpsInfo.value("Longitude", 0.0);
      gps.speed = gpsInfo.value("Speed", 0.0);
      gps.course = gpsInfo.value("Course", 0.0);
      gps.fixQuality = gpsInfo.value("FixQuality", 0.0);
      gps.numSatellites = gpsInfo.value("NumSatellites", 0);
      gps.altitude = gpsInfo.value("Altitude", 0.0);
      gps.hdop = gpsInfo.value("HDOP", 0.0);
      gps.vdop = gpsInfo.value("VDOP", 0.0);
      gps.pdop = gpsInfo.value("PDOP", 0.0);
      gps.visibleSatellites = gpsInfo.value("VisibleSatellites", 0);
    }

    return true;
  } catch (...) {
    return false;
  }
}

bool M20Controller::parseErrorStatusJSON(const std::string& jsonStr, std::vector<ErrorInfo>& errors) {
  try {
    json j = json::parse(jsonStr);
    if (!j.contains("PatrolDevice") || !j["PatrolDevice"].contains("Items") ||
        !j["PatrolDevice"]["Items"].contains("ErrorList")) {
      return false;
    }

    auto& errorList = j["PatrolDevice"]["Items"]["ErrorList"];
    for (const auto& errorItem : errorList) {
      ErrorInfo info;
      info.errorCode = errorItem.value("errorCode", 0);
      info.component = errorItem.value("component", 0);
      errors.push_back(info);
    }

    return true;
  } catch (...) {
    return false;
  }
}

bool M20Controller::parseNavigationStatusJSON(const std::string& jsonStr, NavigationStatus& status) {
  try {
    json j = json::parse(jsonStr);
    if (!j.contains("PatrolDevice") || !j["PatrolDevice"].contains("Items")) {
      return false;
    }

    auto& items = j["PatrolDevice"]["Items"];
    status.value = items.value("Value", 0);
    status.status = items.value("Status", 0);
    status.errorCode = items.value("ErrorCode", 0);

    return true;
  } catch (...) {
    return false;
  }
}

bool M20Controller::parseLocationInfoJSON(const std::string& jsonStr, LocationInfo& info) {
  try {
    json j = json::parse(jsonStr);
    if (!j.contains("PatrolDevice") || !j["PatrolDevice"].contains("Items")) {
      return false;
    }

    auto& items = j["PatrolDevice"]["Items"];
    info.location = items.value("Location", -1);
    info.posX = items.value("PosX", 0.0);
    info.posY = items.value("PosY", 0.0);
    info.posZ = items.value("PosZ", 0.0);
    info.roll = items.value("Roll", 0.0);
    info.pitch = items.value("Pitch", 0.0);
    info.yaw = items.value("Yaw", 0.0);

    return true;
  } catch (...) {
    return false;
  }
}

bool M20Controller::parsePerceptionStatusJSON(const std::string& jsonStr, PerceptionStatus& status) {
  try {
    json j = json::parse(jsonStr);
    if (!j.contains("PatrolDevice") || !j["PatrolDevice"].contains("Items")) {
      return false;
    }

    auto& items = j["PatrolDevice"]["Items"];
    status.location = items.value("Location", -1);
    status.obsState = items.value("ObsState", -1);

    return true;
  } catch (...) {
    return false;
  }
}

bool M20Controller::parseSleepConfigJSON(const std::string& jsonStr, SleepConfig& config) {
  try {
    json j = json::parse(jsonStr);
    if (!j.contains("PatrolDevice") || !j["PatrolDevice"].contains("Items")) {
      return false;
    }

    auto& items = j["PatrolDevice"]["Items"];
    config.sleep = items.value("Sleep", false);
    config.autoSleep = items.value("Auto", false);
    config.time = items.value("Time", 5);

    return true;
  } catch (...) {
    return false;
  }
}

bool M20Controller::parseInitLocationResponseJSON(const std::string& jsonStr, int32_t& errorCode) {
  try {
    json j = json::parse(jsonStr);
    if (!j.contains("PatrolDevice") || !j["PatrolDevice"].contains("Items")) {
      return false;
    }

    errorCode = j["PatrolDevice"]["Items"].value("ErrorCode", -1);
    return true;
  } catch (...) {
    return false;
  }
}

bool M20Controller::parseCancelNavResponseJSON(const std::string& jsonStr, int32_t& errorCode) {
  try {
    json j = json::parse(jsonStr);
    if (!j.contains("PatrolDevice") || !j["PatrolDevice"].contains("Items")) {
      return false;
    }

    errorCode = j["PatrolDevice"]["Items"].value("ErrorCode", -1);
    return true;
  } catch (...) {
    return false;
  }
}

// ========== 通信 ==========

bool M20Controller::sendPacket(const std::vector<uint8_t>& packet) {
  if (!isConnected()) {
    LOG(ERROR) << "[M20Controller] Not connected";
    return false;
  }

  ssize_t sent = send(sockfd_, packet.data(), packet.size(), 0);
  if (sent != static_cast<ssize_t>(packet.size())) {
    LOG(ERROR) << "[M20Controller] Send failed: " << sent << " / " << packet.size();
    return false;
  }

  return true;
}

bool M20Controller::receivePacket(std::vector<uint8_t>& packet, int timeoutMs) {
  if (!isConnected()) {
    return false;
  }

  // 设置超时
  struct timeval tv;
  tv.tv_sec = timeoutMs / 1000;
  tv.tv_usec = (timeoutMs % 1000) * 1000;
  setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  uint8_t buffer[MAX_PACKET_SIZE];
  ssize_t received = recv(sockfd_, buffer, sizeof(buffer), 0);

  if (received <= 0) {
    return false;
  }

  packet.assign(buffer, buffer + received);
  return true;
}

// ========== 线程函数 ==========

void M20Controller::receiveThread() {
  std::vector<uint8_t> buffer;
  std::vector<uint8_t> packet;

  while (running_) {
    uint8_t temp[4096];
    ssize_t received = recv(sockfd_, temp, sizeof(temp), 0);

    if (received <= 0) {
      if (!running_) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    // 添加到缓冲区
    buffer.insert(buffer.end(), temp, temp + received);

    // 解析数据包
    while (buffer.size() >= 16) {
      // 查找同步字符
      size_t syncPos = 0;
      while (syncPos < buffer.size() - 3 &&
             !(buffer[syncPos] == SYNC_BYTE_1 &&
               buffer[syncPos + 1] == SYNC_BYTE_2 &&
               buffer[syncPos + 2] == SYNC_BYTE_3 &&
               buffer[syncPos + 3] == SYNC_BYTE_4)) {
        syncPos++;
      }

      if (syncPos > 0) {
        // 丢弃无效数据
        buffer.erase(buffer.begin(), buffer.begin() + syncPos);
        continue;
      }

      // 检查是否有足够的数据
      if (buffer.size() < 16) {
        break;
      }

      // 解析长度
      uint16_t length = buffer[4] | (buffer[5] << 8);

      // 检查数据完整性
      if (buffer.size() < 16 + length) {
        break;
      }

      // 提取完整数据包
      packet.assign(buffer.begin(), buffer.begin() + 16 + length);
      buffer.erase(buffer.begin(), buffer.begin() + 16 + length);

      // 解析数据包
      std::string asdu;
      uint8_t format;
      if (parsePacket(packet, asdu, format)) {
        // 提取消息类型和命令
        try {
          json j = json::parse(asdu);

          // JSON结构: {"PatrolDevice": {"Type": 1002, "Command": 6, "Time": "...", "Items": {...}}}
          if (!j.contains("PatrolDevice") || !j["PatrolDevice"].is_object()) {
            LOG(WARNING) << "[M20Controller] Invalid JSON: missing PatrolDevice object";
            continue;
          }

          auto& pd = j["PatrolDevice"];
          int32_t type = pd.value("Type", -1);
          int32_t command = pd.value("Command", -1);

          // 处理不同类型的消息
          if (type == static_cast<int32_t>(MsgType::BASIC_STATUS) && command == CMD_STATUS_BASIC) {
            BasicStatus status;
            if (parseBasicStatusJSON(asdu, status)) {
              std::lock_guard<std::mutex> lock(statusMutex_);
              basicStatus_ = status;
              if (basicStatusCallback_) {
                basicStatusCallback_(status);
              }
            } else {
              LOG(ERROR) << "[M20Controller] Failed to parse BasicStatus JSON";
            }
          } else if (type == static_cast<int32_t>(MsgType::BASIC_STATUS) && command == CMD_STATUS_MOTION) {
            MotionStatus motion;
            MotorStatus motor;
            if (parseMotionStatusJSON(asdu, motion, motor)) {
              std::lock_guard<std::mutex> lock(statusMutex_);
              motionStatus_ = motion;
              motorStatus_ = motor;
              if (motionStatusCallback_) {
                motionStatusCallback_(motion, motor);
              }
            }
          } else if (type == static_cast<int32_t>(MsgType::BASIC_STATUS) && command == CMD_STATUS_DEVICE) {
            BatteryStatus battery;
            GPSInfo gps;
            if (parseDeviceStatusJSON(asdu, battery, gps)) {
              std::lock_guard<std::mutex> lock(statusMutex_);
              batteryStatus_ = battery;
              gpsInfo_ = gps;
              if (deviceStatusCallback_) {
                deviceStatusCallback_(battery, gps);
              }
            }
          } else if (type == static_cast<int32_t>(MsgType::BASIC_STATUS) && command == CMD_STATUS_ERROR) {
            std::vector<ErrorInfo> errors;
            if (parseErrorStatusJSON(asdu, errors)) {
              if (errorStatusCallback_) {
                errorStatusCallback_(errors);
              }
            }
          }
        } catch (...) {
          // JSON解析失败，跳过此数据包
        }
      }
    }
  }
}

void M20Controller::heartbeatThread() {
  while (running_) {
    sendHeartbeat();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

// ========== 工具函数 ==========

std::string M20Controller::getCurrentTime() {
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  std::tm* tm_now = std::localtime(&time_t_now);

  std::stringstream ss;
  ss << std::setfill('0')
     << std::setw(4) << (tm_now->tm_year + 1900) << "-"
     << std::setw(2) << (tm_now->tm_mon + 1) << "-"
     << std::setw(2) << tm_now->tm_mday << " "
     << std::setw(2) << tm_now->tm_hour << ":"
     << std::setw(2) << tm_now->tm_min << ":"
     << std::setw(2) << tm_now->tm_sec;

  return ss.str();
}

std::string M20Controller::escapeJSON(const std::string& str) {
  std::string result;
  for (char c : str) {
    switch (c) {
      case '"': result += "\\\""; break;
      case '\\': result += "\\\\"; break;
      case '\b': result += "\\b"; break;
      case '\f': result += "\\f"; break;
      case '\n': result += "\\n"; break;
      case '\r': result += "\\r"; break;
      case '\t': result += "\\t"; break;
      default:
        if (c < '\x20') {
          char buf[7];
          snprintf(buf, sizeof(buf), "\\u%04X", c);
          result += buf;
        } else {
          result += c;
        }
    }
  }
  return result;
}

} // namespace cetcrobot
