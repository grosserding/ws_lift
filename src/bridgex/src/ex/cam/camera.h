#pragma once

#include <zmq.hpp>
#include <string>
#include <memory>
#include <mutex>
#include "json.h"

namespace bridgex {

struct PTZPosition {
  double roll;
  double pitch;
  double yaw;

  PTZPosition() : roll(0.0), pitch(0.0), yaw(0.0) {}
  PTZPosition(double r, double p, double y) : roll(r), pitch(p), yaw(y) {}
};
class Camera {
 public:
  explicit Camera(const std::string &endpoint = "tcp://127.0.0.1:5333");
  ~Camera();
  bool Connect();
  void Disconnect();
  bool IsConnected() const;
  bool GetPTZ(PTZPosition &position, std::string &error_msg) const;
  bool ControlPTZ(const PTZPosition &position, std::string &error_msg) const;
 private:
  zmq::context_t context_{1};
  std::unique_ptr<zmq::socket_t> socket_;
  std::string endpoint_;
  bool connected_{false};
  bool SendRequest(const nlohmann::json &request, nlohmann::json &response, std::string &error_msg) const;
};

}  // namespace bridgex