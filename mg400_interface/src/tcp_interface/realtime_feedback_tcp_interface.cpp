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
RealtimeFeedbackTcpInterface::RealtimeFeedbackTcpInterface(const std::string & ip)
: current_joints_{}, rt_data_{}, is_running_(false)
{
  this->tcp_socket_ = std::make_shared<TcpSocketHandler>(ip, this->PORT_);
}

RealtimeFeedbackTcpInterface::~RealtimeFeedbackTcpInterface()
{
  this->disConnect();
}

void RealtimeFeedbackTcpInterface::init() noexcept
{
  try {
    this->is_running_ = true;
    this->thread_ = std::make_unique<std::thread>(
      &RealtimeFeedbackTcpInterface::recvData, this);
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

void RealtimeFeedbackTcpInterface::getCurrentJointStates(
  std::array<double, 6> & joints)
{
  this->mutex_.lock();
  joints = this->current_joints_;
  this->mutex_.unlock();
}

RealTimeData RealtimeFeedbackTcpInterface::getRealtimeData()
{
  return this->rt_data_;
}

RobotMode RealtimeFeedbackTcpInterface::getRobotMode()
{
  return static_cast<RobotMode>(this->rt_data_.robot_mode);
}

void RealtimeFeedbackTcpInterface::disConnect()
{
  RCLCPP_INFO(
    this->getLogger(),
    "Close connection.");
  this->is_running_ = false;
  this->thread_->join();
  this->tcp_socket_->disConnect();
}

void RealtimeFeedbackTcpInterface::recvData()
{
  static const int CONNECTION_TRIAL = 3;
  using namespace std::chrono_literals;
  int failed_cnd = 0;
  try {
    while (failed_cnd < CONNECTION_TRIAL) {
      if (!this->is_running_) {
        return;
      }
      try {
        if (this->tcp_socket_->isConnected()) {
          if (this->tcp_socket_->recv(
              &this->rt_data_, sizeof(this->rt_data_), 5000))
          {
            if (this->rt_data_.len != 1440) {
              continue;
            }

            this->mutex_.lock();
            for (uint64_t i = 0; i < 4; ++i) {
              this->current_joints_[i] =
                this->rt_data_.q_actual[i] * TO_RADIAN;
            }
            this->mutex_.unlock();
            continue;
          } else {
            // timeout
            RCLCPP_WARN(this->getLogger(), "Tcp recv timeout");
          }
        } else {
          try {
            this->tcp_socket_->connect();
          } catch (const TcpSocketException & err) {
            RCLCPP_ERROR(
              this->getLogger(), "Tcp recv error: %s", err.what());
            rclcpp::sleep_for(500ms);
            failed_cnd++;
          }
        }
      } catch (const TcpSocketException & err) {
        this->tcp_socket_->disConnect();
        RCLCPP_ERROR(
          this->getLogger(),
          "Tcp recv error: %s", err.what());
        rclcpp::sleep_for(500ms);
        failed_cnd++;
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->getLogger(),
      e.what());
  }

  RCLCPP_ERROR(
    this->getLogger(),
    "Failed more than %d times.. . Close connection.",
    failed_cnd);
  this->is_running_ = false;
}

}  // namespace mg400_interface
