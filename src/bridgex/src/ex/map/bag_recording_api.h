#pragma once

#include <zmq.hpp>
#include <string>
#include <memory>

namespace bridgex {

class BagRecordingApi {
 public:
  explicit BagRecordingApi(const std::string& endpoint = "tcp://127.0.0.1:15510");
  ~BagRecordingApi();

  bool Init();
  void Stop();

  std::string StartRecord() const;
  std::string StopRecord() const;
  std::string Ping() const;

 private:
  std::string SendAndRecv(const std::string& msg) const;
  std::string endpoint_;
  zmq::context_t context_;
  std::unique_ptr<zmq::socket_t> socket_;
  bool is_initialized_;
};

}  // namespace bridgex