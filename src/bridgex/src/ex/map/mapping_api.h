#pragma once

#include <zmq.hpp>
#include <string>
#include <memory>

namespace bridgex {

class MappingApi {
 public:
  explicit MappingApi(const std::string& endpoint = "tcp://127.0.0.1:15520");
  ~MappingApi();

  bool Init();
  void Stop();

  std::string StartMap(const std::string& name) const;
  std::string WaitMapComplete() const;
  std::string AbortMap() const;
  std::string Ping() const;

 private:
  std::string SendAndRecv(const std::string& msg) const;
  std::string endpoint_;
  zmq::context_t context_;
  std::unique_ptr<zmq::socket_t> socket_;
  bool is_initialized_;
};

}  // namespace bridgex