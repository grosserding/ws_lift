#include "lite3_controller.h"
#include "log.h"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <utility>

namespace cetcrobot {

Lite3Controller::Lite3Controller(std::string  ip, int port)
    : ip_(std::move(ip)),
      port_(port),
      sockfd_(-1),
      connected_(false),
      running_(false) {
}

Lite3Controller::~Lite3Controller() {
  disconnect();
}

bool Lite3Controller::connect() {
  if (isConnected()) {
    return true;
  }

  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd_ < 0) {
    LOG(ERROR) << "[Lite3Controller] Failed to create socket";
    return false;
  }

  struct sockaddr_in server_addr{};
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port_);

  if (inet_pton(AF_INET, ip_.c_str(), &server_addr.sin_addr) <= 0) {
    LOG(ERROR) << "[Lite3Controller] Invalid address: " << ip_;
    close(sockfd_);
    sockfd_ = -1;
    return false;
  }

  // 设置超时
  struct timeval tv;
  tv.tv_sec = 1;
  tv.tv_usec = 0;
  setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  LOG(INFO) << "[Lite3Controller] Connected to Lite3 at " << ip_ << ":" << port_;

  connected_ = true;
  running_ = true;
  receiveThread_ = std::thread(&Lite3Controller::receiveThread, this);
  // 启动心跳
  startHeartbeat(1);

  return true;
}

void Lite3Controller::disconnect() {
  running_ = false;

  if (receiveThread_.joinable()) {
    receiveThread_.join();
  }

  if (sockfd_ != -1) {
    close(sockfd_);
    sockfd_ = -1;
  }

  connected_ = false;
  LOG(INFO) << "[Lite3Controller] Disconnected";
}

bool Lite3Controller::isConnected() const {
  return connected_ && sockfd_ != -1;
}

void Lite3Controller::setStatusCallback(StatusCallbackLite3 callback) {
  statusCallback_ = callback;
}

std::vector<uint8_t> Lite3Controller::buildPacket(uint32_t code, uint32_t parameters_size, uint32_t type) {
  std::vector<uint8_t> packet(12);
  // 小端序打包: code, parameters_size, type
  packet[0] = code & 0xFF;
  packet[1] = (code >> 8) & 0xFF;
  packet[2] = (code >> 16) & 0xFF;
  packet[3] = (code >> 24) & 0xFF;

  packet[4] = parameters_size & 0xFF;
  packet[5] = (parameters_size >> 8) & 0xFF;
  packet[6] = (parameters_size >> 16) & 0xFF;
  packet[7] = (parameters_size >> 24) & 0xFF;

  packet[8] = type & 0xFF;
  packet[9] = (type >> 8) & 0xFF;
  packet[10] = (type >> 16) & 0xFF;
  packet[11] = (type >> 24) & 0xFF;

  return packet;
}

bool Lite3Controller::sendPacket(const std::vector<uint8_t>& packet) const
{
  if (!isConnected()) {
    LOG(ERROR) << "[Lite3Controller] Not connected";
    return false;
  }

  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port_);
  inet_pton(AF_INET, ip_.c_str(), &server_addr.sin_addr);

  ssize_t sent = sendto(sockfd_, packet.data(), packet.size(), 0,
                        (struct sockaddr*)&server_addr, sizeof(server_addr));
  if (sent != static_cast<ssize_t>(packet.size())) {
    LOG(ERROR) << "[Lite3Controller] Send failed";
    return false;
  }

  return true;
}

bool Lite3Controller::standSit() {
  auto packet = buildPacket(CMD_STAND_SIT, 0, 0);
  return sendPacket(packet);
}

bool Lite3Controller::navControlMode() {
  auto packet = buildPacket(CMD_NAV_CONTROL, 0, 0);
  return sendPacket(packet);
}

bool Lite3Controller::handleControlMode() {
  auto packet = buildPacket(CMD_HANDLE_CONTROL, 0, 0);
  return sendPacket(packet);
}

bool Lite3Controller::sendHeartbeat() {
  auto packet = buildPacket(CMD_HEARTBEAT_LITE3, 0, 0);
  return sendPacket(packet);
}

void Lite3Controller::startHeartbeat(int interval_seconds) {
  std::thread([this, interval_seconds]() {
    while (running_) {
      sendHeartbeat();
      std::this_thread::sleep_for(std::chrono::seconds(interval_seconds));
    }
  }).detach();
}

void Lite3Controller::stopHeartbeat() {
  running_ = false;
}

void Lite3Controller::receiveThread() {
  std::vector<uint8_t> buffer(1024);

  while (running_) {
    ssize_t received = recv(sockfd_, buffer.data(), buffer.size(), 0);
    if (received > 0) {
      std::vector<uint8_t> data(buffer.begin(), buffer.begin() + received);
      if (statusCallback_) {
        statusCallback_(data);
      }
    }
  }
}

} // namespace cetcrobot