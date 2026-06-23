#include "camera.h"
#include "log.h"
#include <iostream>

namespace bridgex {

Camera::Camera(const std::string &endpoint)
    :endpoint_(endpoint) {}

Camera::~Camera() { Disconnect(); }
bool Camera::Connect() {
  if (connected_) {
    return true;
  }
  try {
    socket_ = std::make_unique<zmq::socket_t>(context_, zmq::socket_type::req);
    socket_->connect(endpoint_);
    connected_ = true;
    LOG(INFO) << "Camera Client connected to: " << endpoint_;
    return true;
  } catch (const zmq::error_t &e) {
    LOG(ERROR) << "Camera Client connection error: " << e.what();
    connected_ = false;
    return false;
  }
}

void Camera::Disconnect() {
  socket_->close();
  socket_.reset();
  connected_ = false;
}

bool Camera::IsConnected() const {
  return connected_;
}

bool Camera::SendRequest(const nlohmann::json &request, nlohmann::json &response,
                               std::string &error_msg) const {
  if (!connected_) {
    error_msg = "Camera not connected";
    return false;
  }
  try {
    std::string request_str = request.dump();
    zmq::message_t req_msg(request_str.begin(), request_str.end());
    auto send_result = socket_->send(req_msg, zmq::send_flags::none);
    if (!send_result.has_value()) {
      error_msg = "Failed to send request";
      return false;
    }
    zmq::message_t rep_msg;
    auto recv_result = socket_->recv(rep_msg, zmq::recv_flags::none);
    if (!recv_result.has_value()) {
      error_msg = "Failed to receive response (timeout)";
      return false;
    }

    // 解析响应
    std::string response_str(static_cast<char *>(rep_msg.data()), rep_msg.size());
    response = nlohmann::json::parse(response_str);
    return true;
  } catch (const zmq::error_t &e) {
    error_msg = std::string("ZMQ error: ") + e.what();
    return false;
  } catch (const nlohmann::json::exception &e) {
    error_msg = std::string("JSON parse error: ") + e.what();
    return false;
  } catch (const std::exception &e) {
    error_msg = std::string("Error: ") + e.what();
    return false;
  }
}

bool Camera::GetPTZ(PTZPosition &position, std::string &error_msg) const {
  nlohmann::json request;
  request["method"] = "get_position";
  request["params"] = nlohmann::json::object();
  nlohmann::json response;
  if (!SendRequest(request, response, error_msg)) {
    return false;
  }
  // 检查响应结果
  if (!response.value("result", false)) {
    error_msg = response.value("error", "Unknown error");
    return false;
  }
  // 解析位置数据
  if (response.contains("data") && response["data"].is_object()) {
    auto &data = response["data"];
    position.roll = data.value("roll", 0.0);
    position.pitch = data.value("pitch", 0.0);
    position.yaw = data.value("yaw", 0.0);
    return true;
  }
  error_msg = "Invalid response data format";
  return false;
}

bool Camera::ControlPTZ(const PTZPosition &position, std::string &error_msg) const {
  nlohmann::json request;
  request["method"] = "ptz_ctrl_position";
  nlohmann::json params;
  params["roll"] = position.roll;
  params["pitch"] = position.pitch;
  params["yaw"] = position.yaw;
  request["params"] = params;
  nlohmann::json response;
  if (!SendRequest(request, response, error_msg)) {
    return false;
  }
  if (!response.value("result", false)) {
    error_msg = response.value("error", "Unknown error");
    return false;
  }
  return true;
}

}  // namespace bridgex