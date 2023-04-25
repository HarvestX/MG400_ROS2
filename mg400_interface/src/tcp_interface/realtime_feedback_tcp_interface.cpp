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
  this->is_running_.store(false);
  this->tcp_socket_ = std::make_shared<TcpSocketHandler>(ip, this->PORT_);
}

RealtimeFeedbackTcpInterface::~RealtimeFeedbackTcpInterface()
{
  this->disConnect();
}

void RealtimeFeedbackTcpInterface::init() noexcept
{
  try {
    this->is_running_.store(true);
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
  return this->getRealtimeData() != nullptr;
}

void RealtimeFeedbackTcpInterface::getCurrentJointStates(std::array<double, 4> & joints)
{
  this->mutex_current_joints_.lock();
  joints = this->current_joints_;
  this->mutex_current_joints_.unlock();
}

void RealtimeFeedbackTcpInterface::getCurrentEndPose(Pose & pose)
{
  this->mutex_current_joints_.lock();
  JointHandler::getEndPose(this->current_joints_, pose);
  this->mutex_current_joints_.unlock();
}

std::shared_ptr<RealTimeData> RealtimeFeedbackTcpInterface::getRealtimeData()
{
  std::shared_ptr<RealTimeData> rt_data_local_;
  this->mutex_rt_data_.lock();
  rt_data_local_ = this->rt_data_;
  this->mutex_rt_data_.unlock();
  return rt_data_local_;
}

bool RealtimeFeedbackTcpInterface::getRobotMode(uint64_t & mode)
{
  std::shared_ptr<RealTimeData> rt_data_local_ = this->getRealtimeData();
  if (rt_data_local_) {
    mode = rt_data_local_->robot_mode;
    return true;
  } else {
    return false;
  }
}

bool RealtimeFeedbackTcpInterface::isRobotMode(const uint64_t & expected_mode)
{
  std::shared_ptr<RealTimeData> rt_data_local_ = this->getRealtimeData();
  if (rt_data_local_) {
    return rt_data_local_->robot_mode == expected_mode;
  } else {
    return false;
  }
}

void RealtimeFeedbackTcpInterface::disConnect()
{
  this->is_running_.store(false);
  if (this->thread_->joinable()) {
    this->thread_->join();
  }
  this->tcp_socket_->disConnect();
  RCLCPP_INFO(this->getLogger(), "Close connection.");
}

void RealtimeFeedbackTcpInterface::recvData()
{
  using namespace std::chrono_literals;  // NOLINT
  while (true) {
    if (!this->is_running_.load()) {
      return;
    }
    try {
      if (this->tcp_socket_->isConnected()) {
        auto recvd_data = std::make_shared<RealTimeData>();
        if (this->tcp_socket_->recv(recvd_data.get(), sizeof(RealTimeData), 1000)) {
          if (recvd_data->len != 1440) {
            this->mutex_rt_data_.lock();
            this->rt_data_ = nullptr;
            this->mutex_rt_data_.unlock();
            continue;
          }
          this->mutex_rt_data_.lock();
          this->rt_data_ = recvd_data;
          this->mutex_rt_data_.unlock();

          this->mutex_current_joints_.lock();
          for (uint64_t i = 0; i < 4; ++i) {
            this->current_joints_[i] = this->getRealtimeData()->q_actual[i] * TO_RADIAN;
          }
          this->mutex_current_joints_.unlock();
          continue;
        } else {
          // timeout
          RCLCPP_WARN(this->getLogger(), "Tcp recv timeout");
          this->mutex_rt_data_.lock();
          this->rt_data_ = nullptr;
          this->mutex_rt_data_.unlock();
        }
      } else {
        try {
          this->tcp_socket_->connect(1000);
        } catch (const TcpSocketException & err) {
          RCLCPP_ERROR(this->getLogger(), "Tcp recv error: %s", err.what());
          this->is_running_.store(false);
          return;
        }
      }
    } catch (const TcpSocketException & err) {
      this->tcp_socket_->disConnect();
      RCLCPP_ERROR(this->getLogger(), "Tcp recv error: %s", err.what());
      this->is_running_.store(false);
    }
  }
}

}  // namespace mg400_interface
