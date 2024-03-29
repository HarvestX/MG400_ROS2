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

namespace mg400_interface
{
RealtimeFeedbackTcpInterface::RealtimeFeedbackTcpInterface(
  const std::string & ip, const std::string & prefix)
: frame_id_prefix(prefix),
  current_joints_{}, rt_data_{}
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

void RealtimeFeedbackTcpInterface::disConnect()
{
  this->is_running_ = false;
  if (this->thread_->joinable()) {
    this->thread_->join();
  }
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
        this->tcp_socket_->connect(1s);
        continue;
      }

      auto recvd_data = std::make_shared<RealTimeData>();
      if (!this->tcp_socket_->recv(recvd_data.get(), sizeof(RealTimeData), 1s)) {
        // Error: Data take timeout
        RCLCPP_WARN(this->getLogger(), "Tcp recv timeout");
        std::lock_guard<std::mutex> lock_rt_data(this->mutex_rt_data_);
        this->rt_data_ = nullptr;
        continue;
      }

      if (recvd_data->len != sizeof(RealTimeData)) {
        // Error: Invalid size
        std::lock_guard<std::mutex> lock_rt_data(this->mutex_rt_data_);
        this->rt_data_ = nullptr;
        continue;
      } else {
        // Success
        std::lock_guard<std::mutex> lock_rt_data(this->mutex_rt_data_);
        this->rt_data_ = std::move(recvd_data);
      }

      std::lock_guard<std::mutex> lock_current_joints(this->mutex_current_joints_);
      std::lock_guard<std::mutex> lock_rt_data(this->mutex_rt_data_);
      for (uint64_t i = 0; i < this->current_joints_.size(); ++i) {
        this->current_joints_[i] = this->rt_data_->q_actual[i] * TO_RADIAN;
      }
    } catch (const TcpSocketException & err) {
      this->tcp_socket_->disConnect();
      RCLCPP_ERROR(this->getLogger(), "Tcp recv error: %s", err.what());
      return;
    }
  }
}
}  // namespace mg400_interface
