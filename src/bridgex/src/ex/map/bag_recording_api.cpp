#include "bag_recording_api.h"
#include "log.h"
#include <iostream>

namespace bridgex {

BagRecordingApi::BagRecordingApi(const std::string& endpoint)
    : endpoint_(endpoint), is_initialized_(false) {
}

BagRecordingApi::~BagRecordingApi() { Stop(); }

bool BagRecordingApi::Init() {
  try {
    socket_ = std::make_unique<zmq::socket_t>(context_, zmq::socket_type::req);
    socket_->connect(endpoint_);
    is_initialized_ = true;
    LOG(INFO) << "BagRecordingApi connected to: " << endpoint_;
    return true;
  } catch (const zmq::error_t& e) {
    LOG(ERROR) << "BagRecordingApi connection error: " << e.what();
    return false;
  }
}

void BagRecordingApi::Stop() {
  try {
    socket_->close();
    socket_.reset();
    is_initialized_ = false;
  } catch (...) {
    LOG(ERROR) << "BagRecordingApi stop error";
  }
}

std::string BagRecordingApi::SendAndRecv(const std::string& msg) const {
  if (!is_initialized_) {
    return "ERR:NOT_INITIALIZED";
  }
  try {
#if USE_NEW_ZMQ_API
    socket_->set(zmq::sockopt::sndtimeo, 1000);
    socket_->set(zmq::sockopt::rcvtimeo, 1000);
#else
    socket_->setsockopt(ZMQ_SNDTIMEO, 1000);
    socket_->setsockopt(ZMQ_RCVTIMEO, 1000);
#endif
    zmq::message_t req_msg(msg.begin(), msg.end());
    auto send_result = socket_->send(req_msg, zmq::send_flags::none);
    if (!send_result.has_value()) {
      return "ERR:SEND_TIMEOUT";
    }
    zmq::message_t rep_msg;
    auto recv_result = socket_->recv(rep_msg, zmq::recv_flags::none);
    if (!recv_result.has_value()) {
      return "ERR:RECV_TIMEOUT";
    }
    return std::string(static_cast<char*>(rep_msg.data()), rep_msg.size());
  } catch (const zmq::error_t& e) {
    LOG(ERROR) << "BagRecordingApi ZMQ error: " << e.what();
    return std::string("ZMQ ERR:") + e.what();
  }
}

std::string BagRecordingApi::StartRecord() const {
  return SendAndRecv("start_record");
}

std::string BagRecordingApi::StopRecord() const {
  return SendAndRecv("stop_record");
}

std::string BagRecordingApi::Ping() const {
  return SendAndRecv("ping");
}

}  // namespace bridgex