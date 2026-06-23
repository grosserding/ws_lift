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
  SwitchMap =4  // 切图
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
};

}  // namespace bridgex