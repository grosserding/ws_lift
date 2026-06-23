#include "task_manager.h"
#include "log.h"
#include "ex_client.h"
#include <ros/ros.h>
#include <thread>
#include <chrono>
#include <sstream>
#include <ctime>

namespace bridgex {

TaskManager::TaskManager(ExClient& ex_client)
    : ex_client_(ex_client) {
}

TaskManager::~TaskManager() { Stop(); }

void TaskManager::Start() {
  if (is_running_) {
    LOG(WARNING) << "TaskManager is already running";
  }
  is_running_ = true;
  execute_thread_ = std::thread(&TaskManager::ExecuteThread, this);
  LOG(INFO) << "TaskManager started";
}
void TaskManager::Stop() {
  if (!is_running_) {
    return;
  }
  is_running_ = false;
  should_cancel_ = true;
  is_paused_ = false;
  task_cv_.notify_all();
  TerminateTask();
  if (execute_thread_.joinable()) {
    execute_thread_.join();
  }
  LOG(INFO) << "TaskManager stopped";
}

bool TaskManager::ReceiveTask(const nlohmann::json& task_json) {
  try {
    Task new_task;
    if (!ParseTask(task_json, new_task)) {
      LOG(ERROR) << "Failed to parse task JSON";
      return false;
    }
    {
      std::lock_guard lock(task_mutex_);
      current_task_ = new_task;
      has_new_task_ = true;
      should_cancel_ = false;
      is_paused_ = false;
    }
    task_cv_.notify_all();
    LOG(INFO) << "Task received: " << new_task.name << ", actions count: " << new_task.actions.size() << ", loop: " << (new_task.loop ? "true" : "false");
    return true;
  } catch (const std::exception& e) {
    LOG(ERROR) << "Exception in ReceiveTask: " << e.what();
    return false;
  }
}

void TaskManager::PauseTask() {
  std::lock_guard lock(status_mutex_);
  if (status_.state == TaskExecutionState::RUNNING) {
    is_paused_ = true;
    UpdateStatus(TaskExecutionState::PAUSED, "Task paused");
    LOG(INFO) << "Task paused";
  }
}

void TaskManager::ResumeTask() {
  std::lock_guard lock(status_mutex_);
  if (status_.state == TaskExecutionState::PAUSED) {
    is_paused_ = false;
    UpdateStatus(TaskExecutionState::RUNNING, "Task resumed");
    task_cv_.notify_all();
    LOG(INFO) << "Task resumed";
  }
}

void TaskManager::TerminateTask() {
  should_cancel_ = true;
  is_paused_ = false;
  task_cv_.notify_all();
  ex_client_.ProcessRequest(R"({"msgType": "CancelNav"})");
  LOG(INFO) << "Task cancelled";
}
TaskStatus TaskManager::GetStatus() const {
  std::lock_guard lock(status_mutex_);
  return status_;
}

void TaskManager::SetStatusCallback(const TaskStatusCallback& callback) { status_callback_ = callback; }

bool TaskManager::ParseTask(const nlohmann::json& json_obj, Task& task) {
  try {
    task.name = json_obj.value("name", "unknown");
    task.loop = json_obj.value("loop", false);
    if (!json_obj.contains("actions") || !json_obj["actions"].is_array()) {
      LOG(ERROR) << "Task JSON must contain 'actions' array";
      return false;
    }
    task.actions.clear();
    for (const auto& action_json : json_obj["actions"]) {
      Action action;
      if (!action_json.contains("type")) {
        LOG(ERROR) << "Action must contain 'type' field";
        return false;
      }
      action.type = static_cast<ActionType>(action_json["type"].get<int32_t>());
      action.param = action_json.value("param", nlohmann::json::object());
      task.actions.push_back(action);
    }
    if (task.actions.empty()) {
      LOG(ERROR) << "Task must contain at least one action";
      return false;
    }

    // 解析 schedules
    task.schedules.clear();
    if (json_obj.contains("schedules") && json_obj["schedules"].is_array()) {
      for (const auto& sched_json : json_obj["schedules"]) {
        Schedule sched;
        sched.hour = sched_json.value("hour", 0);
        sched.minute = sched_json.value("minute", 0);
        sched.enabled = sched_json.value("enabled", true);
        sched.last_run_date.clear();  // 新任务没有上次执行日期
        task.schedules.push_back(sched);
      }
    }

    return true;
  } catch (const nlohmann::json::exception& e) {
    LOG(ERROR) << "JSON parse error: " << e.what();
    return false;
  }
}

void TaskManager::ExecuteThread() {
  LOG(INFO) << "Task execution thread started";
  while (is_running_) {
    {
      std::unique_lock lock(task_mutex_);
      task_cv_.wait(lock, [this] { return has_new_task_ || !is_running_; });
      if (!is_running_) {
        break;
      }
      has_new_task_ = false;
    }
    do {
      if (should_cancel_) {
        UpdateStatus(TaskExecutionState::CANCELLED, "Task cancelled");
        break;
      }
      Task task_copy;
      {
        std::lock_guard lock(task_mutex_);
        task_copy = current_task_;
      }

      // 检查是否有 schedules
      if (!task_copy.schedules.empty()) {
        // 定时任务模式：等待到 schedule 指定时间再执行
        if (!ExecuteScheduledTask(task_copy)) {
          break;
        }
      } else {
        // 即时任务模式：立即执行
        if (!ExecuteTaskActions(task_copy)) {
          break;
        }
      }

    } while (current_task_.loop && is_running_ && !should_cancel_);
  }

  LOG(INFO) << "Task execution thread ended";
}

bool TaskManager::ExecuteScheduledTask(const Task& task) {
  while (is_running_ && !should_cancel_) {
    // 获取当前系统时间
    std::time_t now = std::time(nullptr);
    std::tm* now_tm = std::localtime(&now);

    // 构建今天的日期字符串 "YYYY-MM-DD"
    char date_buf[32];
    std::strftime(date_buf, sizeof(date_buf), "%Y-%m-%d", now_tm);
    std::string today_date(date_buf);

    // 检查是否有 schedule 触发
    for (size_t i = 0; i < task.schedules.size(); ++i) {
      const auto& sched = task.schedules[i];
      if (!sched.enabled) continue;

      // 检查是否今天已执行过
      bool already_run_today = (sched.last_run_date == today_date);
      if (already_run_today) continue;

      // 检查当前时间是否匹配
      if (now_tm->tm_hour == sched.hour && now_tm->tm_min == sched.minute) {
        // 匹配成功，更新 last_run_date
        {
          std::lock_guard lock(task_mutex_);
          current_task_.schedules[i].last_run_date = today_date;
        }

        LOG(INFO) << "Scheduled time reached: " << sched.hour << ":" << sched.minute
                  << ", executing task: " << task.name;
        UpdateStatus(TaskExecutionState::RUNNING, "Executing scheduled task");

        // 执行 actions
        if (!ExecuteTaskActions(task)) {
          return false;
        }

        // 如果 loop=true，继续等待下一个 schedule
        if (task.loop) {
          LOG(INFO) << "Task actions completed, waiting for next schedule";
          UpdateStatus(TaskExecutionState::RUNNING, "Waiting for next schedule");
          continue;  // 继续检查下一个触发时间
        } else {
          return false;  // 非 loop 任务执行完就结束
        }
      }
    }

    // 暂停检查
    {
      std::unique_lock lock(task_mutex_);
      task_cv_.wait_for(lock, std::chrono::seconds(1), [this] {
        return should_cancel_ || !is_running_;
      });
      if (should_cancel_ || !is_running_) {
        return false;
      }
    }
  }
  return false;
}

bool TaskManager::ExecuteTaskActions(const Task& task_copy) {
  for (size_t i = 0; i < task_copy.actions.size(); ++i) {
    if (should_cancel_) {
      UpdateStatus(TaskExecutionState::CANCELLED, "Task cancelled during execution");
      return false;
    }
    {
      std::unique_lock lock(task_mutex_);
      task_cv_.wait(lock, [this] { return !is_paused_ || should_cancel_ || !is_running_; });
      if (should_cancel_ || !is_running_) {
        UpdateStatus(TaskExecutionState::CANCELLED, "Task cancelled during pause");
        return false;
      }
    }
    // 更新当前动作信息
    {
      std::lock_guard lock(status_mutex_);
      status_.current_action_index = static_cast<int32_t>(i);
      status_.total_actions = static_cast<int32_t>(task_copy.actions.size());
      const auto& action = task_copy.actions[i];
      switch (action.type) {
        case ActionType::Delay:
          status_.current_action_type = "Delay";
          break;
        case ActionType::Navigate:
          status_.current_action_type = "Navigate";
          break;
        case ActionType::Action:
          status_.current_action_type = "Action";
          break;
        case ActionType::SwitchMap:
          status_.current_action_type = "SwitchMap";
          break;
      }
      NotifyStatusChange();
    }
    std::string error_msg;
    if (!ExecuteAction(task_copy.actions[i], error_msg)) {
      LOG(ERROR) << "Failed to execute action " << i << ": " << error_msg;
      UpdateStatus(TaskExecutionState::FAILED, error_msg);
      return false;
    }
  }
  UpdateStatus(TaskExecutionState::COMPLETED, "Task completed successfully");
  return true;
}

bool TaskManager::ExecuteAction(const Action& action, std::string& error_msg) {
  switch (action.type) {
    case ActionType::Delay: {
      DelayParam param;
      param.duration_ms = action.param.value("duration_ms", 0);
      return ExecuteDelay(param, error_msg);
    }
    case ActionType::Navigate: {
      NavigationParam param;
      param.name = action.param.value("name", "");
      param.x = action.param.value("x", 0.0);
      param.y = action.param.value("y", 0.0);
      param.z = action.param.value("z", 0.0);
      param.qW = action.param.value("qW", 1.0);
      param.qX = action.param.value("qX", 0.0);
      param.qY = action.param.value("qY", 0.0);
      param.qZ = action.param.value("qZ", 0.0);
      return ExecuteNavigation(param, error_msg);
    }
    case ActionType::Action: {
      M20MotionParam param;
      param.motion_type = action.param.value("motion_type", "");
      return ExecuteM20Motion(param, error_msg);
    }
    case ActionType::SwitchMap: {
      SwitchMapParam param;
      param.mapName = action.param.value("mapName", "");
      param.name = action.param.value("name", "");
      param.x = action.param.value("x", 0.0);
      param.y = action.param.value("y", 0.0);
      param.z = action.param.value("z", 0.0);
      param.qW = action.param.value("qW", 1.0);
      param.qX = action.param.value("qX", 0.0);
      param.qY = action.param.value("qY", 0.0);
      param.qZ = action.param.value("qZ", 0.0);
      param.executeTaskChain = action.param.value("executeTaskChain", false);
      param.taskName = action.param.value("taskName", "");
      return ExecuteSwitchMap(param, error_msg);
    }
    default:
      error_msg = "Unknown action type";
      return false;
  }
}
bool TaskManager::ExecuteDelay(const DelayParam& param, std::string& error_msg) {
  LOG(INFO) << "Executing delay: " << param.duration_ms << " ms";

  auto start_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::milliseconds(param.duration_ms);

  while (is_running_ && !should_cancel_) {
    if (is_paused_) {
      std::unique_lock lock(task_mutex_);
      task_cv_.wait(lock, [this] { return !is_paused_ || should_cancel_ || !is_running_; });
      if (should_cancel_ || !is_running_) {
        break;
      }
      start_time = std::chrono::steady_clock::now();
    }
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed >= duration) {
      break;
    }
    // 短暂休眠避免 CPU 占用过高
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (should_cancel_) {
    error_msg = "Delay cancelled";
    return false;
  }

  LOG(INFO) << "Delay completed";
  return true;
}

bool TaskManager::ExecuteNavigation(const NavigationParam& param, std::string& error_msg) {
  LOG(INFO) << "Executing navigation to: " << param.name << " (" << param.x << ", " << param.y << ", " << param.z << ")";
  nlohmann::json goal_request;
  goal_request["msgType"] = "Goal";
  goal_request["x"] = param.x;
  goal_request["y"] = param.y;
  goal_request["z"] = param.z;
  goal_request["qW"] = param.qW;
  goal_request["qX"] = param.qX;
  goal_request["qY"] = param.qY;
  goal_request["qZ"] = param.qZ;
  std::string response = ex_client_.ProcessRequest(goal_request.dump());
  try {
    nlohmann::json response_json = nlohmann::json::parse(response);
    if (!response_json.value("status", false)) {
      error_msg = "Failed to send navigation goal: " + response_json.value("message", "unknown error");
      return false;
    }
  } catch (const nlohmann::json::exception& e) {
    error_msg = "Failed to parse navigation response: " + std::string(e.what());
    return false;
  }
  return WaitForNavigationCompletion(error_msg);
}

bool TaskManager::ExecuteM20Motion(const M20MotionParam& param, std::string& error_msg) const {
  LOG(INFO) << "Executing M20 motion: " << param.motion_type;
  nlohmann::json motion_request;
  motion_request["msgType"] = "M20Motion";
  motion_request["motionType"] = param.motion_type;
  std::string response = ex_client_.ProcessRequest(motion_request.dump());
  try {
    nlohmann::json response_json = nlohmann::json::parse(response);
    if (!response_json.value("status", false)) {
      error_msg = "Failed to execute M20 motion: " + response_json.value("message", "unknown error");
      return false;
    }
  } catch (const nlohmann::json::exception& e) {
    error_msg = "Failed to parse M20 motion response: " + std::string(e.what());
    return false;
  }
  LOG(INFO) << "M20 motion completed";
  return true;
}

bool TaskManager::ExecuteSwitchMap(const SwitchMapParam& param, std::string& error_msg) {
  LOG(INFO) << "Executing map switch to: " << param.mapName;
  nlohmann::json map_change_request;
  map_change_request["msgType"] = "MapChange";
  map_change_request["path"] = param.mapName;
  map_change_request["x"] = param.x;
  map_change_request["y"] = param.y;
  map_change_request["z"] = param.z;
  map_change_request["qW"] = param.qW;
  map_change_request["qX"] = param.qX;
  map_change_request["qY"] = param.qY;
  map_change_request["qZ"] = param.qZ;
  std::string response = ex_client_.ProcessRequest(map_change_request.dump());
  try {
    nlohmann::json response_json = nlohmann::json::parse(response);
    if (!response_json.value("status", false)) {
      error_msg = "Failed to switch map: " + response_json.value("message", "unknown error");
      return false;
    }
  } catch (const nlohmann::json::exception& e) {
    error_msg = "Failed to parse map change response: " + std::string(e.what());
    return false;
  }
  LOG(INFO) << "Map switch completed to: " << param.mapName;

  // 如果需要执行子任务链
  if (param.executeTaskChain && !param.taskName.empty()) {
    std::vector<Action> chain_actions;
    if (!LoadTaskChain(param.mapName, param.taskName, chain_actions, error_msg)) {
      return false;
    }
    LOG(INFO) << "Executing task chain: " << param.taskName << " with " << chain_actions.size() << " actions";
    for (const auto& action : chain_actions) {
      if (should_cancel_) {
        error_msg = "Task chain cancelled";
        return false;
      }
      if (!ExecuteAction(action, error_msg)) {
        return false;
      }
    }
    LOG(INFO) << "Task chain completed: " << param.taskName;
  }

  return true;
}

bool TaskManager::LoadTaskChain(const std::string& mapName, const std::string& taskName,
                                std::vector<Action>& actions, std::string& error_msg) const {
  std::string map_base_path;
  if (!ex_client_.GetMapBasePath(map_base_path)) {
    error_msg = "Failed to get map base path";
    return false;
  }

  std::string task_file_path = map_base_path + "/" + mapName + "/task.json";
  LOG(INFO) << "Loading task chain from: " << task_file_path;

  std::ifstream task_file(task_file_path);
  if (!task_file.is_open()) {
    error_msg = "Failed to open task file: " + task_file_path;
    return false;
  }

  std::stringstream buffer;
  buffer << task_file.rdbuf();
  std::string task_content = buffer.str();
  task_file.close();

  try {
    nlohmann::json task_json = nlohmann::json::parse(task_content);
    if (!task_json.contains("tasks") || !task_json["tasks"].is_array()) {
      error_msg = "Invalid task file format: missing 'tasks' array";
      return false;
    }

    for (const auto& task_obj : task_json["tasks"]) {
      std::string name = task_obj.value("name", "");
      if (name == taskName) {
        if (!task_obj.contains("actions") || !task_obj["actions"].is_array()) {
          error_msg = "Task '" + taskName + "' has no actions";
          return false;
        }
        for (const auto& action_json : task_obj["actions"]) {
          Action action;
          action.type = static_cast<ActionType>(action_json["type"].get<int32_t>());
          action.param = action_json.value("param", nlohmann::json::object());
          actions.push_back(action);
        }
        return true;
      }
    }
    error_msg = "Task '" + taskName + "' not found in " + task_file_path;
    return false;
  } catch (const nlohmann::json::exception& e) {
    error_msg = std::string("Failed to parse task file: ") + e.what();
    return false;
  }
}

bool TaskManager::WaitForNavigationCompletion(std::string& error_msg) {
  LOG(INFO) << "Waiting for navigation completion...";
  auto start_time = std::chrono::steady_clock::now();
  auto timeout = std::chrono::milliseconds(NAV_TIMEOUT_MS);
  while (is_running_ && !should_cancel_) {
    if (is_paused_) {
      std::unique_lock lock(task_mutex_);
      task_cv_.wait(lock, [this] { return !is_paused_ || should_cancel_ || !is_running_; });
      if (should_cancel_ || !is_running_) {
        break;
      }
      start_time = std::chrono::steady_clock::now();
    }
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed >= timeout) {
      error_msg = "Navigation timeout";
      LOG(ERROR) << "Navigation timeout after " << std::chrono::duration<double>(elapsed).count() << " seconds";
      ex_client_.ProcessRequest(R"({"msgType": "CancelNav"})");
      return false;
    }
    std::string response = ex_client_.ProcessRequest(R"({"msgType": "NavStatus"})");
    try {
      nlohmann::json response_json = nlohmann::json::parse(response);
      if (!response_json.value("status", false)) {
        error_msg = "Failed to query navigation status";
        return false;
      }
      int state_code = response_json.value("navigationStateCode", -1);
      std::string state_str = response_json.value("navigationState", "UNKNOWN");

      //...................增加 goal_text文本获取
      std::string goal_text_task = response_json.value("state_description", "Unknown state");
      //..........................................

      // 使用 actionlib 状态码判断
      // PENDING=0, ACTIVE=1, RECALLED=2, REJECTED=3, PREEMPTED=4, ABORTED=5, SUCCEEDED=6, LOST=7
      if (state_code == 6) {
        // SUCCEEDED
        LOG(INFO) << "Navigation completed successfully";
        return true;
      } else if (state_code == 5) {
        // ABORTED

        //........新增，若是被占据,返回true.............
        if (goal_text_task == "GoalIsOccupied")
        {
          error_msg = "Goal is occupied";
          LOG(ERROR) << "Goal is occupied";
          return true; // 返回true，任务继续
        }
        //...........................................

        error_msg = "Navigation aborted";
        LOG(ERROR) << "Navigation aborted";
        return false;
      } else if (state_code == 7) {
        // LOST
        error_msg = "Navigation goal lost";
        LOG(ERROR) << "Navigation goal lost";
        return false;
      } else if (state_code == 2 || state_code == 3) {
        // RECALLED or REJECTED
        error_msg = "Navigation goal " + state_str;
        LOG(ERROR) << "Navigation goal " << state_str;
        return false;
      }
      // PENDING 或 ACTIVE，继续等待
    } catch (const nlohmann::json::exception& e) {
      LOG(WARNING) << "Failed to parse navigation status: " << e.what();
    }
    // 短暂休眠避免频繁查询
    std::this_thread::sleep_for(std::chrono::milliseconds(NAV_STATUS_CHECK_INTERVAL_MS));
  }
  if (should_cancel_) {
    error_msg = "Navigation cancelled";
    LOG(INFO) << "Navigation cancelled";
    ex_client_.ProcessRequest(R"({"msgType": "CancelNav"})");
    return false;
  }
  error_msg = "Navigation interrupted";
  return false;
}

void TaskManager::UpdateStatus(TaskExecutionState state, const std::string& error_msg) {
  std::lock_guard lock(status_mutex_);
  status_.state = state;
  status_.error_message = error_msg;
  status_.timestamp = ros::Time::now().toSec();
  NotifyStatusChange();
}

void TaskManager::NotifyStatusChange() const {
  if (status_callback_) {
    status_callback_(status_);
  }
}

}  // namespace bridgex