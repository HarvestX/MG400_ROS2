// Copyright 2022 HarvestX Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "mg400_interface/tcp_interface/realtime_feedback_tcp_interface.hpp"

#include <atomic>
#include <cstddef>
#include <exception>
#include <utility>

namespace mg400_interface
{
namespace
{

ServoFeedbackState::ConnectionEpoch nextConnectionEpoch()
{
  static std::atomic<ServoFeedbackState::ConnectionEpoch> next_epoch{1};
  return next_epoch.fetch_add(1);
}

}  // namespace

RealtimeFeedbackTcpInterface::RealtimeFeedbackTcpInterface(
  const std::string & ip, const std::string & prefix)
: frame_id_prefix(prefix),
  current_joints_{}, rt_data_{},
  servo_feedback_state_(std::make_shared<ServoFeedbackState>(nextConnectionEpoch())),
  realtime_data_callback_{}
{
  this->is_running_ = false;
  this->tcp_socket_ = std::make_shared<TcpSocketHandler>(ip, this->PORT_);
}

RealtimeFeedbackTcpInterface::~RealtimeFeedbackTcpInterface()
{
  if (this->is_running_) {
    this->disConnect();
  }
}

void RealtimeFeedbackTcpInterface::init() noexcept
{
  try {
    // Each socket activation is a distinct connection epoch. A new Session
    // captures this value and cannot consume a frame from an older activation.
    this->servo_feedback_state_->beginConnectionEpoch(nextConnectionEpoch());
    this->is_running_ = true;
    this->thread_ = std::make_unique<std::thread>(&RealtimeFeedbackTcpInterface::recvData, this);
  } catch (const TcpSocketException & err) {
    RCLCPP_ERROR(this->getLogger(), "%s", err.what());
  }
}

rclcpp::Logger RealtimeFeedbackTcpInterface::getLogger()
{
  return rclcpp::get_logger("Realtime Feedback Tcp Interface");
}

bool RealtimeFeedbackTcpInterface::isConnected()
{
  return this->tcp_socket_->isConnected();
}

bool RealtimeFeedbackTcpInterface::isActive()
{
  std::lock_guard<std::mutex> lock_rt_data(this->mutex_rt_data_);
  return this->rt_data_ != nullptr;
}

void RealtimeFeedbackTcpInterface::getCurrentJointStates(std::array<double, 4> & joints)
{
  std::lock_guard<std::mutex> lock_current_joints(this->mutex_current_joints_);
  joints = this->current_joints_;
}

void RealtimeFeedbackTcpInterface::getCurrentEndPose(Pose & pose)
{
  std::lock_guard<std::mutex> lock_current_joints(this->mutex_current_joints_);
  JointHandler::getEndPose(this->current_joints_, pose);
}

bool RealtimeFeedbackTcpInterface::getRealtimeData(RealTimeData & data)
{
  std::lock_guard<std::mutex> lock_rt_data(this->mutex_rt_data_);
  if (this->rt_data_ != nullptr) {
    data = *this->rt_data_;
    return true;
  } else {
    return false;
  }
}

bool RealtimeFeedbackTcpInterface::getRobotMode(uint64_t & mode)
{
  std::lock_guard<std::mutex> lock_rt_data(this->mutex_rt_data_);
  if (this->rt_data_) {
    mode = this->rt_data_->robot_mode;
    return true;
  } else {
    return false;
  }
}

bool RealtimeFeedbackTcpInterface::isRobotMode(const uint64_t & expected_mode)
{
  std::lock_guard<std::mutex> lock_rt_data(this->mutex_rt_data_);
  if (this->rt_data_) {
    return this->rt_data_->robot_mode == expected_mode;
  } else {
    return false;
  }
}

void RealtimeFeedbackTcpInterface::setRealtimeDataCallback(RealtimeDataCallback callback)
{
  std::lock_guard<std::mutex> lock_callback(this->mutex_realtime_data_callback_);
  this->realtime_data_callback_ = std::move(callback);
}

ServoFeedbackState::SharedPtr
RealtimeFeedbackTcpInterface::getServoFeedbackStateShared() const noexcept
{
  return this->servo_feedback_state_;
}

void RealtimeFeedbackTcpInterface::disConnect()
{
  this->is_running_ = false;
  if (this->thread_ && this->thread_->joinable()) {
    this->thread_->join();
  }
  this->thread_.reset();
  this->servo_feedback_state_->invalidate();
  this->tcp_socket_->disConnect();
  RCLCPP_INFO(this->getLogger(), "Close connection.");
}

void RealtimeFeedbackTcpInterface::recvData()
{
  using namespace std::chrono_literals;  // NOLINT
  while (this->is_running_) {
    try {
      // Error: Connection error
      if (!this->tcp_socket_->isConnected()) {
        // TcpSocketHandler can reconnect inside one lifecycle activation. Mark
        // that transport reconnect as a new feedback epoch before attempting
        // it, so no Session can consume the preceding socket's last frame.
        this->servo_feedback_state_->beginConnectionEpoch(nextConnectionEpoch());
        this->tcp_socket_->connect(10s);
        continue;
      }

      auto recvd_data = std::make_shared<RealTimeData>();
      if (!this->tcp_socket_->recv(recvd_data.get(), sizeof(RealTimeData), 1s)) {
        // Error: Data take timeout
        RCLCPP_WARN(this->getLogger(), "Tcp recv timeout");
        {
          std::lock_guard<std::mutex> lock_rt_data(this->mutex_rt_data_);
          this->rt_data_ = nullptr;
        }
        this->servo_feedback_state_->invalidate();
        continue;
      }

      if (!recvd_data->isValid()) {
        // Error: Invalid packet framing or memory-layout test value
        {
          std::lock_guard<std::mutex> lock_rt_data(this->mutex_rt_data_);
          this->rt_data_ = nullptr;
        }
        this->servo_feedback_state_->invalidate();
        continue;
      }

      {
        std::lock_guard<std::mutex> lock_rt_data(this->mutex_rt_data_);
        this->rt_data_ = recvd_data;
      }

      std::array<double, 4> current_joints;
      for (std::size_t i = 0; i < current_joints.size(); ++i) {
        current_joints[i] = recvd_data->q_actual[i] * TO_RADIAN;
      }
      {
        std::lock_guard<std::mutex> lock_current_joints(this->mutex_current_joints_);
        this->current_joints_ = current_joints;
      }

      this->servo_feedback_state_->update(current_joints);

      RealtimeDataCallback callback;
      {
        std::lock_guard<std::mutex> lock_callback(this->mutex_realtime_data_callback_);
        callback = this->realtime_data_callback_;
      }
      if (callback) {
        try {
          callback(*recvd_data);
        } catch (const std::exception & error) {
          RCLCPP_ERROR(this->getLogger(), "Realtime data callback failed: %s", error.what());
        } catch (...) {
          RCLCPP_ERROR(this->getLogger(), "Realtime data callback failed with an unknown error");
        }
      }
    } catch (const TcpSocketException & err) {
      this->tcp_socket_->disConnect();
      this->servo_feedback_state_->invalidate();
      RCLCPP_ERROR(this->getLogger(), "Tcp recv error: %s", err.what());
      return;
    }
  }
}
}  // namespace mg400_interface
