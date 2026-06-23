#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <mutex>
#include <thread>
#include <atomic>
#include <memory>
#include <functional>
#include <cstring>

namespace cetcrobot {

// ========== 协议常量 ==========
constexpr uint8_t SYNC_BYTE_1 = 0xEB;
constexpr uint8_t SYNC_BYTE_2 = 0x91;
constexpr uint8_t SYNC_BYTE_3 = 0xEB;
constexpr uint8_t SYNC_BYTE_4 = 0x90;
constexpr uint8_t FORMAT_XML = 0x00;
constexpr uint8_t FORMAT_JSON = 0x01;
constexpr uint32_t MAX_PACKET_SIZE = 65535 + 16;

// ========== 消息类型和命令码 ==========

// 消息类型
enum class MsgType : uint16_t {
  HEARTBEAT = 100,          // 心跳指令
  MOTION_CONTROL = 2,       // 运动控制
  AUTONOMOUS_CHARGE = 2,    // 自主充电
  BASIC_STATUS = 1002,      // 基础状态
  SYSTEM_CONFIG = 1101,     // 系统配置
  NAV_TASK = 1003,          // 导航任务
  CANCEL_NAV = 1004,        // 取消导航
  NAV_QUERY = 1007,         // 导航查询
  PERCEPTION_STATUS = 2002, // 感知软件状态
  LOCATION_INIT = 2101      // 初始化和重置定位
};

// 心跳指令命令码
constexpr int CMD_HEARTBEAT = 100;

// 运动控制命令码
constexpr int CMD_MOTION_AXIS_SPEED = 21;    // 运动控制（轴指令）
constexpr int CMD_MOTION_STATE_CHANGE = 22;  // 运动状态转换
constexpr int CMD_MOTION_GAIT_CHANGE = 23;   // 步态切换
constexpr int CMD_AUTONOMOUS_CHARGE = 24;    // 自主充电

// 基础状态命令码
constexpr int CMD_STATUS_ERROR = 3;          // 异常状态信息
constexpr int CMD_STATUS_MOTION = 4;         // 运控状态上报
constexpr int CMD_STATUS_DEVICE = 5;         // 设备状态上报
constexpr int CMD_STATUS_BASIC = 6;          // 基础状态上报

// 系统配置命令码
constexpr int CMD_CONFIG_LIGHT = 2;          // 照明灯
constexpr int CMD_CONFIG_USAGE_MODE = 5;     // 使用模式切换
constexpr int CMD_CONFIG_SLEEP = 6;          // 休眠模式设置
constexpr int CMD_CONFIG_SLEEP_QUERY = 7;    // 休眠状态及设置信息查询

// 导航任务命令码
constexpr int CMD_NAV_TASK = 1;              // 下发导航任务
constexpr int CMD_NAV_LOCATION = 2;          // 获取地图坐标系下位置信息
constexpr int CMD_NAV_STATUS = 1;            // 查询导航任务执行状态

// ========== 枚举定义 ==========

// 运动状态
enum class MotionState : int32_t {
  IDLE = 0,           // 空闲
  STAND = 1,          // 站立
  DAMPING = 2,        // 关节阻尼/软急停
  BOOT_DAMPING = 3,   // 开机阻尼
  DOWN = 4,           // 趴下
  RL_CONTROL = 17     // RL控制
};

// 步态
enum class GaitMode : uint32_t {
  BASIC_STD = 0x1001,      // 基础（标准运动模式）
  HIGH_STAGE_STD = 0x1002, // 高台（标准运动模式）
  STAIR_STD = 0x1003,      // 楼梯（标准运动模式）
  FLAT_FAST = 0x3002,      // 平地（敏捷运动模式）
  STAIR_FAST = 0x3003      // 楼梯（敏捷运动模式）
};

// 使用模式
enum class UsageMode : int32_t {
  NORMAL = 0,   // 常规模式
  NAV = 1,      // 导航模式
  ASSIST = 2    // 辅助模式
};

// 导航任务类型
enum class NavPointType : int32_t {
  TRANSITION = 0,  // 过渡点
  TASK = 1,        // 任务点
  CHARGE = 3       // 充电点
};

// 导航速度
enum class NavSpeed : int32_t {
  NORMAL = 0,   // 正常
  LOW = 1,      // 低速
  HIGH = 2      // 高速
};

// 导航运动方式
enum class NavManner : int32_t {
  FORWARD = 0,  // 前进行走
  BACKWARD = 1  // 倒退行走
};

// 导航避障模式
enum class NavObsMode : int32_t {
  ENABLE = 0,   // 开启
  DISABLE = 1   // 关闭
};

// 导航方式
enum class NavMode : int32_t {
  STRAIGHT = 0, // 直线导航
  AUTO = 1      // 自主导航
};

// 导航任务执行状态
enum class NavStatus : int32_t {
  IDLE = 0,         // 空闲
  EXIT_CHARGING = 1,// 退出充电桩中
  PREPROCESSING = 2,// 导航预处理
  NAVIGATING = 3,   // 导航中
  COMPLETED = 4,    // 导航完成
  ENTER_CHARGING = 5,// 进入充电桩中
  PAUSED = 0xFF     // 暂停中
};

// 定位状态
enum class LocationState : int32_t {
  OK = 0,    // 定位正常
  LOST = 1   // 定位丢失
};

// 充电状态
enum class ChargeState : int32_t {
  IDLE = 0,              // 空闲
  GOING_TO_PILE = 1,     // 前往充电桩过程中
  CHARGING = 2,          // 充电中
  EXITING_PILE = 3,      // 退出充电桩过程中
  ERROR = 4,             // 机器人异常
  ON_PILE_NOT_CHARGING = 5 // 机器人在桩上，但未充电
};

// ========== 数据结构 ==========

// 协议头部
struct ProtocolHeader {
  uint8_t sync[4];        // 0xEB, 0x91, 0xEB, 0x90
  uint16_t length;        // ASDU长度（小端）
  uint16_t msgId;         // 报文ID（小端）
  uint8_t format;         // ASDU格式：0=XML, 1=JSON
  uint8_t reserved[7];    // 预留

  ProtocolHeader() {
    sync[0] = SYNC_BYTE_1;
    sync[1] = SYNC_BYTE_2;
    sync[2] = SYNC_BYTE_3;
    sync[3] = SYNC_BYTE_4;
    length = 0;
    msgId = 0;
    format = FORMAT_JSON;
    memset(reserved, 0, 7);
  }
};

// 基础状态信息
struct BasicStatus {
  int32_t motionState;      // 运动状态
  uint32_t gait;            // 步态
  int32_t charge;           // 充电状态
  int32_t hes;              // 硬急停状态
  int32_t controlUsageMode; // 控制使用模式
  int32_t direction;        // 前进正方向
  int32_t ooa;              // 辅助模式避障状态
  int32_t powerManagement;  // 电源管理模式
  int32_t sleep;            // 休眠状态
  std::string version;      // 设备版本

  BasicStatus()
      : motionState(-1),
        gait(-1),
        charge(-1),
        hes(-1),
        controlUsageMode(-1),
        direction(-1),
        ooa(-1),
        powerManagement(-1),
        sleep(-1),
        version("") {}
};

// 运控状态信息
struct MotionStatus {
  double roll;
  double pitch;
  double yaw;
  double omegaZ;
  double linearX;
  double linearY;
  double height;
  double payload;
  double remainMile;

  MotionStatus()
      : roll(0), pitch(0), yaw(0), omegaZ(0),
        linearX(0), linearY(0), height(0),
        payload(0), remainMile(0) {}
};

// 电机状态信息
struct MotorStatus {
  double leftFrontHipX;
  double leftFrontHipY;
  double leftFrontKnee;
  double leftFrontWheel;
  double rightFrontHipX;
  double rightFrontHipY;
  double rightFrontKnee;
  double rightFrontWheel;
  double leftBackHipX;
  double leftBackHipY;
  double leftBackKnee;
  double leftBackWheel;
  double rightBackHipX;
  double rightBackHipY;
  double rightBackKnee;
  double rightBackWheel;

  MotorStatus() {
    memset(this, 0, sizeof(MotorStatus));
  }
};

// 电池状态信息
struct BatteryStatus {
  double voltageLeft;
  double voltageRight;
  double batteryLevelLeft;
  double batteryLevelRight;
  double batteryTempLeft;
  double batteryTempRight;
  bool chargeLeft;
  bool chargeRight;

  BatteryStatus()
      : voltageLeft(0), voltageRight(0),
        batteryLevelLeft(0), batteryLevelRight(0),
        batteryTempLeft(0), batteryTempRight(0),
        chargeLeft(false), chargeRight(false) {}
};

// GPS信息
struct GPSInfo {
  double latitude;
  double longitude;
  double speed;
  double course;
  double fixQuality;
  int32_t numSatellites;
  double altitude;
  double hdop;
  double vdop;
  double pdop;
  int32_t visibleSatellites;

  GPSInfo()
      : latitude(0), longitude(0), speed(0), course(0),
        fixQuality(0), numSatellites(0), altitude(0),
        hdop(0), vdop(0), pdop(0), visibleSatellites(0) {}
};

// 错误信息
struct ErrorInfo {
  int32_t errorCode;
  int32_t component;

  ErrorInfo() : errorCode(0), component(0) {}
};

// 导航目标点
struct NavigationTarget {
  int32_t value;      // 目标点编号
  int32_t mapId;      // 地图ID
  double posX;        // X坐标
  double posY;        // Y坐标
  double posZ;        // Z坐标
  double angleYaw;    // 朝向角度
  int32_t pointInfo;  // 点类型
  uint32_t gait;      // 步态
  int32_t speed;      // 速度
  int32_t manner;     // 运动方式
  int32_t obsMode;    // 避障模式
  int32_t navMode;    // 导航方式

  NavigationTarget()
      : value(0), mapId(0), posX(0), posY(0), posZ(0),
        angleYaw(0), pointInfo(0), gait(0x3002),
        speed(0), manner(0), obsMode(0), navMode(1) {}
};

// 导航执行状态
struct NavigationStatus {
  int32_t value;      // 目标点编号
  int32_t status;     // 执行状态
  int32_t errorCode;  // 错误码

  NavigationStatus() : value(0), status(0), errorCode(0) {}
};

// 位置信息
struct LocationInfo {
  int32_t location;   // 定位状态
  double posX;        // X坐标
  double posY;        // Y坐标
  double posZ;        // Z坐标
  double roll;        // 滚转角
  double pitch;       // 俯仰角
  double yaw;         // 偏航角

  LocationInfo()
      : location(-1), posX(0), posY(0), posZ(0),
        roll(0), pitch(0), yaw(0) {}
};

// 感知软件状态
struct PerceptionStatus {
  int32_t location;   // 定位状态
  int32_t obsState;   // 避障状态

  PerceptionStatus() : location(-1), obsState(-1) {}
};

// 休眠配置
struct SleepConfig {
  bool sleep;         // 休眠模式启停
  bool autoSleep;     // 自动休眠开关
  int32_t time;       // 等待时间（分钟）

  SleepConfig() : sleep(false), autoSleep(false), time(5) {}
};

// ========== 回调函数类型 ==========

using BasicStatusCallback = std::function<void(const BasicStatus&)>;
using MotionStatusCallback = std::function<void(const MotionStatus&, const MotorStatus&)>;
using DeviceStatusCallback = std::function<void(const BatteryStatus&, const GPSInfo&)>;
using ErrorStatusCallback = std::function<void(const std::vector<ErrorInfo>&)>;

// ========== M20Controller类 ==========

class M20Controller {
 public:
  explicit M20Controller(const std::string& ip = "10.21.31.103", int port = 30001);
  ~M20Controller();

  // ========== 连接管理 ==========
  bool connect();
  void disconnect();
  bool isConnected() const;

  // ========== 回调注册 ==========
  void setBasicStatusCallback(BasicStatusCallback callback);
  void setMotionStatusCallback(MotionStatusCallback callback);
  void setDeviceStatusCallback(DeviceStatusCallback callback);
  void setErrorStatusCallback(ErrorStatusCallback callback);

  // ========== 控制指令 - 心跳 ==========
  bool sendHeartbeat();

  // ========== 控制指令 - 运动控制 ==========
  // 运动状态转换
  bool setMotionState(MotionState state);
  bool motionIdle() { return setMotionState(MotionState::IDLE); }
  bool setMotionIdle() { return setMotionState(MotionState::IDLE); }
  bool motionStand() { return setMotionState(MotionState::STAND); }
  bool standUp() { return setMotionState(MotionState::STAND); }
  bool motionDamping() { return setMotionState(MotionState::DAMPING); }
  bool motionBootDamping() { return setMotionState(MotionState::BOOT_DAMPING); }
  bool motionDown() { return setMotionState(MotionState::DOWN); }
  bool sitDown() { return setMotionState(MotionState::DOWN); }
  bool motionRLControl() { return setMotionState(MotionState::RL_CONTROL); }

  // 步态切换
  bool setGait(uint32_t gait);
  bool setGaitBasicStd() { return setGait(static_cast<uint32_t>(GaitMode::BASIC_STD)); }
  bool setGaitHighStageStd() { return setGait(static_cast<uint32_t>(GaitMode::HIGH_STAGE_STD)); }
  bool setGaitStairStd() { return setGait(static_cast<uint32_t>(GaitMode::STAIR_STD)); }
  bool setGaitFlatFast() { return setGait(static_cast<uint32_t>(GaitMode::FLAT_FAST)); }
  bool setGaitStairFast() { return setGait(static_cast<uint32_t>(GaitMode::STAIR_FAST)); }

  // 运动控制（轴指令）- 仅在常规模式下生效
  bool setAxisSpeed(double x, double y, double z,
                    double roll, double pitch, double yaw);

  // 自主充电
  // charge: 0=停止充电, 1=开始充电, 2=清除充电状态
  bool setAutonomousCharge(int32_t charge);
  bool startCharge() { return setAutonomousCharge(1); }    // 开始充电
  bool stopCharge() { return setAutonomousCharge(0); }     // 停止充电
  bool clearCharge() { return setAutonomousCharge(2); }    // 清除充电状态

  // 充电状态等待（发送指令后循环查询）
  // waitForEnterCharge: 等待进入充电状态（仅充电中=2），超时 50s
  // waitForExitCharge: 等待退出充电状态（空闲=0），超时 50s
  bool waitForEnterCharge(int timeoutMs = 50000);
  bool waitForExitCharge(int timeoutMs = 50000);

  // ========== 控制指令 - 系统配置 ==========
  // 使用模式切换
  bool setUsageMode(UsageMode mode);
  bool setUsageModeNormal() { return setUsageMode(UsageMode::NORMAL); }
  bool setUsageModeNav() { return setUsageMode(UsageMode::NAV); }
  bool setUsageModeAssist() { return setUsageMode(UsageMode::ASSIST); }

  // 照明灯控制
  bool setLight(int32_t front, int32_t back);

  // 休眠模式设置
  bool setSleepMode(const SleepConfig& config);
  bool querySleepMode(SleepConfig& config);

  // ========== 控制指令 - 导航（仅M20 Pro支持）==========
  // 初始化和重置定位
  bool initLocation(double posX, double posY, double posZ, double yaw);

  // 获取地图坐标系下位置信息
  bool getLocationInfo(LocationInfo& info);

  // 获取感知软件状态信息
  bool getPerceptionStatus(PerceptionStatus& status);

  // 下发单点导航任务
  bool sendNavigationTask(const NavigationTarget& target);

  // 取消导航任务
  bool cancelNavigationTask();

  // 查询导航任务执行状态
  bool queryNavigationStatus(NavigationStatus& status);

  // ========== 状态查询 ==========
  BasicStatus getBasicStatus() const;
  MotionStatus getMotionStatus() const;
  MotorStatus getMotorStatus() const;
  BatteryStatus getBatteryStatus() const;
  GPSInfo getGPSInfo() const;

 private:
  // ========== 协议封装 ==========
  std::vector<uint8_t> buildPacket(const std::string& asdu, uint8_t format = FORMAT_JSON);
  bool parsePacket(const std::vector<uint8_t>& packet, std::string& asdu, uint8_t& format);

  // ========== JSON/XML构建 ==========
  std::string buildHeartbeatJSON();
  std::string buildMotionStateJSON(MotionState state);
  std::string buildGaitJSON(uint32_t gait);
  std::string buildAxisSpeedJSON(double x, double y, double z,
                                  double roll, double pitch, double yaw);
  std::string buildAutonomousChargeJSON(int32_t charge);
  std::string buildUsageModeJSON(UsageMode mode);
  std::string buildLightJSON(int32_t front, int32_t back);
  std::string buildSleepModeJSON(const SleepConfig& config);
  std::string buildSleepQueryJSON();
  std::string buildInitLocationJSON(double posX, double posY, double posZ, double yaw);
  std::string buildNavigationTaskJSON(const NavigationTarget& target);
  std::string buildCancelNavigationJSON();
  std::string buildNavQueryJSON();
  std::string buildLocationQueryJSON();

  // ========== JSON解析 ==========
  bool parseBasicStatusJSON(const std::string& json, BasicStatus& status);
  bool parseMotionStatusJSON(const std::string& json, MotionStatus& motion, MotorStatus& motor);
  bool parseDeviceStatusJSON(const std::string& json, BatteryStatus& battery, GPSInfo& gps);
  bool parseErrorStatusJSON(const std::string& json, std::vector<ErrorInfo>& errors);
  bool parseNavigationStatusJSON(const std::string& json, NavigationStatus& status);
  bool parseLocationInfoJSON(const std::string& json, LocationInfo& info);
  bool parsePerceptionStatusJSON(const std::string& json, PerceptionStatus& status);
  bool parseSleepConfigJSON(const std::string& json, SleepConfig& config);
  bool parseInitLocationResponseJSON(const std::string& json, int32_t& errorCode);
  bool parseCancelNavResponseJSON(const std::string& json, int32_t& errorCode);

  // ========== 通信 ==========
  bool sendPacket(const std::vector<uint8_t>& packet);
  bool receivePacket(std::vector<uint8_t>& packet, int timeoutMs = 1000);

  // ========== 线程函数 ==========
  void receiveThread();
  void heartbeatThread();

  // ========== 工具函数 ==========
  static std::string getCurrentTime();
  static std::string escapeJSON(const std::string& str);

 private:
  // ========== 网络相关 ==========
  std::string ip_;
  int port_;
  int sockfd_;
  uint16_t msgId_;

  // ========== 线程控制 ==========
  std::thread receiveThread_;
  std::thread heartbeatThread_;
  std::atomic<bool> running_;

  // ========== 状态缓存 ==========
  BasicStatus basicStatus_;
  MotionStatus motionStatus_;
  MotorStatus motorStatus_;
  BatteryStatus batteryStatus_;
  GPSInfo gpsInfo_;

  mutable std::mutex statusMutex_;

  // ========== 回调函数 ==========
  BasicStatusCallback basicStatusCallback_;
  MotionStatusCallback motionStatusCallback_;
  DeviceStatusCallback deviceStatusCallback_;
  ErrorStatusCallback errorStatusCallback_;
};

} // namespace cetcrobot

// 方便其他文件使用，不需要加命名空间前缀
using cetcrobot::M20Controller;