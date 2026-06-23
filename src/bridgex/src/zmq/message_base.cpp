#include "message_base.h"
#include "log.h"
std::string MessageBase::pub_endpoint_;
std::string MessageBase::sub_endpoint_;

MessageBase::MessageBase(std::string name, ros::NodeHandle &nh) : name_(std::move(name)), nh_(nh), is_running_(false) {
  static bool zmq_initialized = false;
  if (!zmq_initialized) {
    SetupZMQ();
    zmq_initialized = true;
  }
}

void MessageBase::SetEndpoints(const std::string &pub_endpoint, const std::string &sub_endpoint) {
  pub_endpoint_ = pub_endpoint;
  sub_endpoint_ = sub_endpoint;
}

void MessageBase::SetupZMQ() {
  try {
    // 创建发布者
    publisher_ = std::make_unique<zmq::socket_t>(context_, zmq::socket_type::pub);
    publisher_->bind(pub_endpoint_);

    // 创建订阅者
    subscriber_ = std::make_unique<zmq::socket_t>(context_, zmq::socket_type::sub);
    subscriber_->connect(pub_endpoint_);

    LOG(INFO) << "ZMQ initialized on: " << pub_endpoint_;
  } catch (const zmq::error_t &e) {
    LOG(ERROR) << "ZMQ initialization error: " << e.what();
    CleanupZMQ();
  }
}

void MessageBase::CleanupZMQ() {
  try {
    if (publisher_) publisher_->close();
    if (subscriber_) subscriber_->close();
    context_.close();

    for (auto &thread : sub_threads_) {
      if (thread.joinable()) {
        thread.join();
      }
    }
    LOG(INFO) << "ZMQ cleanup successful";
  } catch (...) {
    LOG(ERROR) << "ZMQ cleanup error";
  }
}

void MessageBase::PubToZMQ(const std::string &topic, const std::string &data) {
  try {
    std::lock_guard lock(mutex_);
    if (!publisher_) {
      return;
    }
    publisher_->send(zmq::buffer(topic), zmq::send_flags::sndmore);
    publisher_->send(zmq::buffer(data), zmq::send_flags::dontwait);
  } catch (const zmq::error_t &e) {
    LOG(ERROR) << "Publish error: " << e.what();
  }
}
void MessageBase::SubFromZMQ(const std::string &topic, const std::function<void(const std::string &)> &callback) {
  sub_threads_.emplace_back([this, topic, callback]() {
    try {
      zmq::socket_t subscriber(context_, zmq::socket_type::sub);
      subscriber.connect(sub_endpoint_);
      LOG(INFO) << "Subscribing sub_endpoint: " << sub_endpoint_;
#if USE_NEW_ZMQ_API
      subscriber.set(zmq::sockopt::subscribe, topic);
#else
      // 使用旧API (4.3.0-4.6.x 或更早版本)
      subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.data(), topic.size());
#endif
      while (is_running_) {
        zmq::message_t topic_msg, data_msg;
        if (!subscriber.recv(topic_msg) || !subscriber.recv(data_msg)) {
          LOG(WARNING) << "Subscription receive failed for topic: " << topic;
          continue;
        }
        if (std::string received_topic(static_cast<char *>(topic_msg.data()), topic_msg.size());
            received_topic != topic) {
          continue;
        }
        std::string data(static_cast<char *>(data_msg.data()), data_msg.size());
        callback(data);
      }
    } catch (const zmq::error_t &e) {
      LOG(ERROR) << "Exception in Sub thread for topic [" << topic << "]: " << e.what();
    }
  });
}

void MessageBase::Start() {
  if (is_running_) {
    return;
  }
  Init();
  is_running_ = true;
  LOG(INFO) << "[" << name_ << "] Message handler started";
}

void MessageBase::Stop() {
  if (!is_running_) {
    return;
  }
  is_running_ = false;
  ros_pub_.shutdown();
  ros_sub_.shutdown();

  LOG(INFO) << "[" << name_ << "] Message handler stopped";
}
