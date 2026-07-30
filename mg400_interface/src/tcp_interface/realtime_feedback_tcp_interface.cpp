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

#include <array>
#include <exception>
#include <limits>
#include <utility>

namespace mg400_interface
{
RealtimeFeedbackTcpInterface::RealtimeFeedbackTcpInterface(
  const std::string & ip, const std::string & prefix)
: frame_id_prefix(prefix),
  realtime_data_snapshot_{}, realtime_data_callback_{}
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
    this->beginConnectionEpoch();
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

bool RealtimeFeedbackTcpInterface::isActive() const
{
  std::lock_guard<std::mutex> lock(this->mutex_realtime_data_);
  return this->realtime_data_snapshot_.has_data;
}

bool RealtimeFeedbackTcpInterface::getCurrentJointStates(std::array<double, 4> & joints) const
{
  const auto snapshot = this->getLatestRealtimeData();
  if (!snapshot.has_data) {
    return false;
  }
  joints = snapshot.jointAnglesRad();
  return true;
}

bool RealtimeFeedbackTcpInterface::getCurrentEndPose(Pose & pose) const
{
  std::array<double, 4> joints{};
  return this->getCurrentJointStates(joints) && JointHandler::getEndPose(joints, pose);
}

RealtimeDataSnapshot RealtimeFeedbackTcpInterface::getLatestRealtimeData() const
{
  std::lock_guard<std::mutex> lock(this->mutex_realtime_data_);
  return this->realtime_data_snapshot_;
}

bool RealtimeFeedbackTcpInterface::getRobotMode(uint64_t & mode) const
{
  const auto snapshot = this->getLatestRealtimeData();
  if (!snapshot.has_data) {
    return false;
  }
  mode = snapshot.data.robot_mode;
  return true;
}

bool RealtimeFeedbackTcpInterface::isRobotMode(const uint64_t & expected_mode) const
{
  const auto snapshot = this->getLatestRealtimeData();
  return snapshot.has_data && snapshot.data.robot_mode == expected_mode;
}

void RealtimeFeedbackTcpInterface::setRealtimeDataCallback(RealtimeDataCallback callback)
{
  std::lock_guard<std::mutex> lock_callback(this->mutex_realtime_data_callback_);
  this->realtime_data_callback_ = std::move(callback);
}

void RealtimeFeedbackTcpInterface::disConnect()
{
  this->is_running_ = false;
  if (this->thread_ && this->thread_->joinable()) {
    this->thread_->join();
  }
  this->thread_.reset();
  this->invalidateRealtimeData();
  this->tcp_socket_->disConnect();
  RCLCPP_INFO(this->getLogger(), "Close connection.");
}

void RealtimeFeedbackTcpInterface::beginConnectionEpoch()
{
  std::lock_guard<std::mutex> lock(this->mutex_realtime_data_);
  const auto current_epoch = this->realtime_data_snapshot_.connection_epoch;
  const auto next_epoch =
    current_epoch == std::numeric_limits<RealtimeDataSnapshot::ConnectionEpoch>::max() ?
    1 : current_epoch + 1;
  this->realtime_data_snapshot_ = RealtimeDataSnapshot{};
  this->realtime_data_snapshot_.connection_epoch = next_epoch;
}

void RealtimeFeedbackTcpInterface::invalidateRealtimeData()
{
  std::lock_guard<std::mutex> lock(this->mutex_realtime_data_);
  const auto connection_epoch = this->realtime_data_snapshot_.connection_epoch;
  this->realtime_data_snapshot_ = RealtimeDataSnapshot{};
  this->realtime_data_snapshot_.connection_epoch = connection_epoch;
}

void RealtimeFeedbackTcpInterface::recvData()
{
  using namespace std::chrono_literals;  // NOLINT
  bool has_connected = false;
  while (this->is_running_) {
    try {
      // Error: Connection error
      if (!this->tcp_socket_->isConnected()) {
        if (has_connected) {
          this->beginConnectionEpoch();
        }
        this->tcp_socket_->connect(10s);
        has_connected = true;
        continue;
      }

      RealTimeData received_data{};
      if (!this->tcp_socket_->recv(&received_data, sizeof(received_data), 1s)) {
        // Error: Data take timeout
        RCLCPP_WARN(this->getLogger(), "Tcp recv timeout");
        this->invalidateRealtimeData();
        continue;
      }

      if (!received_data.isValid()) {
        // Error: Invalid packet framing or memory-layout test value
        this->invalidateRealtimeData();
        continue;
      }

      {
        std::lock_guard<std::mutex> lock(this->mutex_realtime_data_);
        this->realtime_data_snapshot_.data = received_data;
        this->realtime_data_snapshot_.received_at = RealtimeDataSnapshot::Clock::now();
        this->realtime_data_snapshot_.has_data = true;
      }

      RealtimeDataCallback callback;
      {
        std::lock_guard<std::mutex> lock_callback(this->mutex_realtime_data_callback_);
        callback = this->realtime_data_callback_;
      }
      if (callback) {
        try {
          callback(received_data);
        } catch (const std::exception & error) {
          RCLCPP_ERROR(this->getLogger(), "Realtime data callback failed: %s", error.what());
        } catch (...) {
          RCLCPP_ERROR(this->getLogger(), "Realtime data callback failed with an unknown error");
        }
      }
    } catch (const TcpSocketException & err) {
      this->tcp_socket_->disConnect();
      this->invalidateRealtimeData();
      RCLCPP_ERROR(this->getLogger(), "Tcp recv error: %s", err.what());
      return;
    }
  }
}
}  // namespace mg400_interface
