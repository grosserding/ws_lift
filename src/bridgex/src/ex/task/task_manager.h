#pragma once

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <functional>
#include "json.h"

namespace bridgex {

// 前向声明 ExClient
class ExClient;

// ========== 动作类型 ==========
enum class ActionType : int32_t { Delay = 0,
  Navigate = 1,
  Action = 2,
  Camera = 3,
  SwitchMap =4,  // 切图
  Elevator = 5   // 梯控
};

struct NavigationParam {
  std::string name;  // 点位名称
  double x;          // X 坐标
  double y;          // Y 坐标
  double z;          // Z 坐标
  double qW;         // 四元数 W
  double qX;         // 四元数 X
  double qY;         // 四元数 Y
  double qZ;         // 四元数 Z
  NavigationParam() : x(0), y(0), z(0), qW(1.0), qX(0), qY(0), qZ(0) {}
};

struct M20MotionParam {
  std::string motion_type;
  M20MotionParam() = default;
};

struct DelayParam {
  int32_t duration_ms;
  DelayParam() : duration_ms(0) {}
};

struct SwitchMapParam {
  std::string mapName;  // 地图名称
  std::string name;     // 点位名称
  double x;             // X 坐标
  double y;             // Y 坐标
  double z;             // Z 坐标
  double qW;            // 四元数 W
  double qX;            // 四元数 X
  double qY;            // 四元数 Y
  double qZ;            // 四元数 Z
  bool executeTaskChain;  // 是否执行子任务链
  std::string taskName;    // 子任务名称
  SwitchMapParam() : x(0), y(0), z(0), qW(1.0), qX(0), qY(0), qZ(0), executeTaskChain(false) {}
};

struct ElevatorParam {
  std::string op;     // open_door / close_door / call
  std::string mode;   // come / go （op==call 时有效）
  int32_t floor;      // 目标楼层（op==call 时有效）
  ElevatorParam() : floor(0) {}
};

struct Action {
  ActionType type;
  nlohmann::json param;
  Action() : type(ActionType::Delay) {}
};

struct Schedule {
  int hour;       // 小时 (0-23)
  int minute;     // 分钟 (0-59)
  bool enabled;   // 是否启用
  std::string last_run_date;  // 上次执行日期 "YYYY-MM-DD"，防同一天重复执行
  Schedule() : hour(0), minute(0), enabled(true) {}
};

struct Task {
  std::string name;             // 任务名称
  bool loop;                    // 是否循环执行
  std::vector<Schedule> schedules;  // 定时调度表
  std::vector<Action> actions;  // 动作列表
  Task() : loop(false) {}
};

enum class TaskExecutionState : int32_t {
  IDLE = 0,       // 空闲
  RUNNING = 1,    // 运行中
  PAUSED = 2,     // 暂停
  COMPLETED = 3,  // 已完成
  FAILED = 4,     // 失败
  CANCELLED = 5   // 已取消
};

struct TaskStatus {
  TaskExecutionState state;         // 执行状态
  std::string task_name;            // 当前任务名称
  int32_t current_action_index;     // 当前执行的动作索引
  int32_t total_actions;            // 总动作数
  std::string current_action_type;  // 当前动作类型描述
  std::string error_message;        // 错误信息
  double timestamp;                 // 时间戳

  TaskStatus() : state(TaskExecutionState::IDLE), current_action_index(0), total_actions(0), timestamp(0) {}
};

using TaskStatusCallback = std::function<void(const TaskStatus&)>;

class TaskManager {
 public:
  explicit TaskManager(ExClient& ex_client);
  ~TaskManager();

  void Start();

  void Stop();

  bool IsRunning() const { return is_running_; }

  bool ReceiveTask(const nlohmann::json& task_json);

  void PauseTask();

  void ResumeTask();

  void TerminateTask();

  TaskStatus GetStatus() const;

  void SetStatusCallback(const TaskStatusCallback& callback);

 private:
  static bool ParseTask(const nlohmann::json& json_obj, Task& task);

  void ExecuteThread();

  bool ExecuteAction(const Action& action, std::string& error_msg);

  bool ExecuteDelay(const DelayParam& param, std::string& error_msg);

  bool ExecuteNavigation(const NavigationParam& param, std::string& error_msg);

  bool ExecuteM20Motion(const M20MotionParam& param, std::string& error_msg) const;

  bool ExecuteElevator(const ElevatorParam& param, std::string& error_msg);

  // 梯控辅助：呼梯一次、查电梯楼层、开/关门、查梯内点占据
  bool LiftCallOnce(int floor);

  bool QueryLiftFloor(int& current_floor);

  bool SetLiftDoor(bool open, std::string& error_msg);

  bool IsWpOccupied();

  // 去某楼层：定频呼梯直到电梯到达目标楼层
  bool ElevatorGoToFloor(int floor, std::string& error_msg);

  // 来某楼层：呼梯到层后判断能否进梯，占据则等电梯离层后重呼
  bool ElevatorComeToFloor(int floor, std::string& error_msg);

  // 可被取消/暂停打断的睡眠，返回 false 表示被取消
  bool InterruptibleSleep(int duration_ms);

  bool ExecuteSwitchMap(const SwitchMapParam& param, std::string& error_msg);

  bool LoadTaskChain(const std::string& mapName, const std::string& taskName,
                     std::vector<Action>& actions, std::string& error_msg) const;

  bool WaitForNavigationCompletion(std::string& error_msg);

  bool ExecuteScheduledTask(const Task& task);

  bool ExecuteTaskActions(const Task& task);

  void UpdateStatus(TaskExecutionState state, const std::string& error_msg = "");

  void NotifyStatusChange() const;

  ExClient& ex_client_;  // ExClient 引用

  // 任务执行控制
  std::atomic<bool> is_running_{false};     // 任务管理器是否运行
  std::atomic<bool> is_paused_{false};      // 当前任务是否暂停
  std::atomic<bool> should_cancel_{false};  // 是否应该取消任务
  std::atomic<bool> has_new_task_{false};   // 是否有新任务
  std::thread execute_thread_;       // 执行线程

  // 任务数据
  Task current_task_;                // 当前任务
  std::mutex task_mutex_;            // 任务互斥锁
  std::condition_variable task_cv_;  // 任务条件变量

  // 状态信息
  TaskStatus status_;                   // 当前状态
  mutable std::mutex status_mutex_;     // 状态互斥锁
  TaskStatusCallback status_callback_;  // 状态回调

  // 导航查询参数
  static constexpr int NAV_STATUS_CHECK_INTERVAL_MS = 500;  // 导航状态查询间隔（毫秒）
  static constexpr int NAV_TIMEOUT_MS = 3600000;              // 导航超时时间（60分钟）

  // 梯控可调参数（从 ROS 参数服务器读取，便于测试调优）
  double elevator_call_rate_hz_{1.0};            // 呼梯频率
  double elevator_status_query_rate_hz_{5.0};    // 电梯状态查询频率
  int elevator_judge_start_delay_ms_{5000};      // 到层到开始判断的等待（等门完全打开）
  int elevator_free_confirm_ms_{2000};           // 连续空闲多久判定“可进入”
  int elevator_max_judge_ms_{8000};              // 最长观察时长，超时仍无连续空闲则判占据放弃
  double elevator_occupied_check_rate_hz_{10.0}; // 判断窗口内采样占据的频率
  int elevator_wp_occupied_freshness_ms_{500};   // /wp_occupied 消息有效期，超期视为未收到
  int elevator_person_count_freshness_ms_{500};  // /yolo/person_count 消息有效期，超期视为未收到
  bool elevator_enforce_yolo_{false};            // 是否强制要求收到 yolo 人数；收不到时 true=判占据 false=判未占据
};

}  // namespace bridgex