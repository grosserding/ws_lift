#pragma once

#include <string>
#include <fstream>
#include <sstream>

namespace bridgex {

// 文件存储类，用于读写文本文件
class FileStorage {
 public:
  explicit FileStorage(const std::string &default_path = "unknown",const std::string &robot_type = "unknown");

  ~FileStorage() = default;

  // 读取文件内容，返回字符串
  bool Read(std::string &content) const;

  // 保存内容到文件
  bool Save(const std::string &content) const;

 private:
  std::string file_path_;
  std::string robot_type_;
};

}  // namespace bridgex