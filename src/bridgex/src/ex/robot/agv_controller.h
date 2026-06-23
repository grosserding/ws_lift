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

// ========== AGVController类 ==========

class AGVController {
 public:
  explicit AGVController();
  ~AGVController();
};

} // namespace cetcrobot

// 方便其他文件使用，不需要加命名空间前缀
using cetcrobot::AGVController;