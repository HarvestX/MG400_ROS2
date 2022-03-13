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

#include "mg400_control/mg400_control/commander.hpp"

namespace mg400_control
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("MG400Commander");

Commander::Commander(const std::string & ip)
: current_joints_{}, tool_vector_{}, real_time_data_{}, is_running_(false)
{
  this->is_running_ = false;
  this->real_time_tcp_ = std::make_shared<TcpClient>(ip, 30003);
  this->dash_board_tcp_ = std::make_shared<TcpClient>(ip, 29999);
}

Commander::~Commander()
{
  this->is_running_ = false;
  this->thread_->join();
}

void Commander::getCurrentJointStates(double * joints)
{
  this->mutex_.lock();
  memcpy(joints, this->current_joints_, sizeof(this->current_joints_));
  mutex_.unlock();
}

void Commander::getToolVectorActual(double * val)
{
  this->mutex_.lock();
  memcpy(val, this->tool_vector_, sizeof(this->tool_vector_));
  this->mutex_.unlock();
}

void Commander::recvTask() noexcept
{
  while (this->is_running_) {
    try {
      if (this->dash_board_tcp_->isConnected() && this->real_time_tcp_->isConnected()) {
        if (this->real_time_tcp_->recv(
            &this->real_time_data_, sizeof(this->real_time_data_),
            5000))
        {
          if (this->real_time_data_.len != 1440) {
            continue;
          }

          this->mutex_.lock();
          for (uint64_t i = 0; i < 6; ++i) {
            this->current_joints_[i] = this->deg2Rad(
              this->real_time_data_.q_actual[i]
            );
          }
          memcpy(
            this->tool_vector_, this->real_time_data_.tool_vector_actual,
            sizeof(this->tool_vector_));
          this->mutex_.unlock();
        } else {
          // timeout
          RCLCPP_WARN(
            LOGGER,
            "Tcp recv timeout"
          );
        }
      } else {
        try {
          this->real_time_tcp_->connect();
          this->dash_board_tcp_->connect();
        } catch (const TcpClientException & err) {
          RCLCPP_ERROR(
            LOGGER,
            "Tcp recv error : %s",
            err.what()
          );
          sleep(3);
        }
      }
    } catch (const TcpClientException & err) {
      dash_board_tcp_->disConnect();
      RCLCPP_ERROR(
        LOGGER,
        "Tcp recv error : %s", err.what()
      );
    }
  }
}

void Commander::init() noexcept
{
  try {
    this->is_running_ = true;
    this->thread_ = std::make_unique<std::thread>(
      &Commander::recvTask, this
    );
  } catch (const TcpClientException & err) {
    RCLCPP_ERROR(
      LOGGER,
      "Commander : %s",
      err.what()
    );
  }
}

bool Commander::isEnabled() const noexcept
{
  return this->real_time_data_.robot_mode == 5;
}

bool Commander::isConnected() const noexcept
{
  return this->dash_board_tcp_->isConnected() && this->real_time_tcp_->isConnected();
}

void Commander::enableRobot()
{
  const char * cmd = "EnableRobot()";
  this->dash_board_tcp_->send(cmd, strlen(cmd));
}

void Commander::disableRobot()
{
  const char * cmd = "DisableRobot()";
  this->dash_board_tcp_->send(cmd, strlen(cmd));
}

void Commander::clearError()
{
  const char * cmd = "ClearError()";
  this->dash_board_tcp_->send(cmd, strlen(cmd));
}

void Commander::resetRobot()
{
  const char * cmd = "ResetRobot()";
  this->dash_board_tcp_->send(cmd, strlen(cmd));
}

void Commander::speedFactor(int ratio)
{
  char cmd[100];
  sprintf(cmd, "SpeedFactor(%d)", ratio);
  this->dash_board_tcp_->send(cmd, strlen(cmd));
}

void Commander::movJ(double x, double y, double z, double a, double b, double c)
{
  char cmd[100];
  sprintf(cmd, "MovJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", x, y, z, a, b, c);
  this->real_time_tcp_->send(cmd, strlen(cmd));
}

void Commander::movL(double x, double y, double z, double a, double b, double c)
{
  char cmd[100];
  sprintf(cmd, "MovL(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", x, y, z, a, b, c);
  this->real_time_tcp_->send(cmd, strlen(cmd));
}

void Commander::jointMovJ(double j1, double j2, double j3, double j4, double j5, double j6)
{
  char cmd[100];
  sprintf(cmd, "JointMovJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", j1, j2, j3, j4, j5, j6);
  this->real_time_tcp_->send(cmd, strlen(cmd));
}

void Commander::moveJog(const std::string & axis)
{
  char cmd[100];
  sprintf(cmd, "MoveJog(%s)", axis.c_str());
  this->real_time_tcp_->send(cmd, strlen(cmd));
}


void Commander::relMovJ(
  double offset1, double offset2, double offset3, double offset4, double offset5,
  double offset6)
{
  char cmd[100];
  sprintf(
    cmd, "RelMovJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", offset1, offset2, offset3, offset4,
    offset5,
    offset6);
  this->real_time_tcp_->send(cmd, strlen(cmd));
}

void Commander::relMovL(double x, double y, double z)
{
  char cmd[100];
  sprintf(cmd, "RelMovL(%0.3f,%0.3f,%0.3f)", x, y, z);
  this->real_time_tcp_->send(cmd, strlen(cmd));
}

void Commander::servoJ(double j1, double j2, double j3, double j4, double j5, double j6)
{
  char cmd[100];
  sprintf(cmd, "ServoJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", j1, j2, j3, j4, j5, j6);
  this->real_time_tcp_->send(cmd, strlen(cmd));
}

void Commander::servoP(double x, double y, double z, double a, double b, double c)
{
  char cmd[100];
  sprintf(cmd, "ServoP(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", x, y, z, a, b, c);
  this->real_time_tcp_->send(cmd, strlen(cmd));
}

void Commander::dashSendCmd(const char * cmd, uint32_t len)
{
  this->dash_board_tcp_->send(cmd, len);
}

void Commander::realSendCmd(const char * cmd, uint32_t len)
{
  this->real_time_tcp_->send(cmd, len);
}
} // namespace mg400_control
