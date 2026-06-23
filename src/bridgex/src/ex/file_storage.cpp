#include "file_storage.h"
#include "log.h"
#include <ros/ros.h>

namespace bridgex {

FileStorage::FileStorage(const std::string &default_path,const std::string &robot_type) : file_path_(default_path),robot_type_(robot_type) {
  // 检查文件是否存在，不存在则创建
  std::ifstream check_file(file_path_);
  if (!check_file.good()) {
    std::ofstream create_file(file_path_);
    if (create_file.good()) {
      LOG(INFO) << "FileStorage: Created new file at " << file_path_;
      // 默认写入固定路径
      create_file << "map";
    } else {
      LOG(ERROR) << "FileStorage: Failed to create file at " << file_path_;
    }
  } else {
    LOG(INFO) << "FileStorage: File exists at " << file_path_;
  }
}

bool FileStorage::Read(std::string &content) const {
  std::ifstream file(file_path_);
  if (!file.good()) {
    LOG(ERROR) << "FileStorage: Failed to open file for reading: " << file_path_;
    return false;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  content = buffer.str();
  file.close();

  return true;
}

bool FileStorage::Save(const std::string &content) const {
  std::ofstream file(file_path_);
  if (!file.good()) {
    LOG(ERROR) << "FileStorage: Failed to open file for writing: " << file_path_;
    return false;
  }

  file << content;
  file.close();

  LOG(INFO) << "FileStorage: Saved content to " << file_path_;
  return true;
}

}  // namespace bridgex