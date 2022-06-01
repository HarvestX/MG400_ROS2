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

#include "mg400_control/mg400_interface/commander.hpp"

namespace mg400_interface
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
      if (this->dash_board_tcp_->isConnected() &&
        this->real_time_tcp_->isConnected())
      {
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
              this->real_time_data_.q_actual[i]);
          }
          memcpy(
            this->tool_vector_, this->real_time_data_.tool_vector_actual,
            sizeof(this->tool_vector_));
          this->mutex_.unlock();
        } else {
          // timeout
          RCLCPP_WARN(
            LOGGER,
            "Tcp recv timeout");
        }
      } else {
        try {
          this->real_time_tcp_->connect();
          this->dash_board_tcp_->connect();
        } catch (const TcpClientException & err) {
          RCLCPP_ERROR(
            LOGGER,
            "Tcp recv error : %s",
            err.what());
          sleep(3);
        }
      }
    } catch (const TcpClientException & err) {
      dash_board_tcp_->disConnect();
      RCLCPP_ERROR(
        LOGGER,
        "Tcp recv error : %s", err.what());
    }
  }
}

void Commander::init() noexcept
{
  try {
    this->is_running_ = true;
    this->thread_ = std::make_unique<std::thread>(
      &Commander::recvTask, this);
  } catch (const TcpClientException & err) {
    RCLCPP_ERROR(
      LOGGER,
      "Commander : %s",
      err.what());
  }
}

bool Commander::isEnabled() const noexcept
{
  return this->real_time_data_.robot_mode == 5;
}

bool Commander::isConnected() const noexcept
{
  return this->dash_board_tcp_->isConnected() &&
         this->real_time_tcp_->isConnected();
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

void Commander::getErrorID()
{
  const char * cmd = "GetErrorID()";
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
  snprintf(cmd, sizeof(cmd), "SpeedFactor(%d)", ratio);
  this->dash_board_tcp_->send(cmd, strlen(cmd));
}

/**
 * @brief Point to point movement, the target point is Cartesan point
 *
 * @param x X-axis coordinates, unit: m
 * @param y Y-axis coordinates, unit: m
 * @param z Z-axis coordinates, unit: m
 * @param a A-axis coordinates, unit: rad
 * @param b B-axis coordinates, unit: rad
 * @param c C-axis coordinates, unit: rad
 */
void Commander::movJ(double x, double y, double z, double a, double b, double c)
{
  char cmd[100];
  snprintf(
    cmd, sizeof(cmd),
    "MovJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)",
    this->m2mm(x),
    this->m2mm(y),
    this->m2mm(z),
    this->rad2Deg(a),
    this->rad2Deg(b),
    this->rad2Deg(c));
  this->real_time_tcp_->send(cmd, strlen(cmd));
}

/**
 * @brief Linear movement, the target point is Cartesan point
 *
 * @param x X-axis coordinates, unit: m
 * @param y Y-axis coordinates, unit: m
 * @param z Z-axis coordinates, unit: m
 * @param a A-axis coordinates, unit: rad
 * @param b B-axis coordinates, unit: rad
 * @param c C-axis coordinates, unit: rad
 */
void Commander::movL(double x, double y, double z, double a, double b, double c)
{
  char cmd[100];
  snprintf(
    cmd, sizeof(cmd),
    "MovL(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)",
    this->m2mm(x),
    this->m2mm(y),
    this->m2mm(z),
    this->rad2Deg(a),
    this->rad2Deg(b),
    this->rad2Deg(c));
  this->real_time_tcp_->send(cmd, strlen(cmd));
}

/**
 * @brief Point to point movement, the target point is joint point
 *
 * @param j1 J1 coordinates, unit: rad
 * @param j2 J2 coordinates, unit: rad
 * @param j3 J3 coordinates, unit: rad
 * @param j4 J4 coordinates, unit: rad
 * @param j5 J5 coordinates, unit: rad
 * @param j6 J6 coordinates, unit: rad
 */
void Commander::jointMovJ(
  double j1, double j2, double j3,
  double j4, double j5, double j6)
{
  char cmd[100];
  snprintf(
    cmd, sizeof(cmd),
    "JointMovJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)",
    this->rad2Deg(j1),
    this->rad2Deg(j2),
    this->rad2Deg(j3),
    this->rad2Deg(j4),
    this->rad2Deg(j5),
    this->rad2Deg(j6));
  this->real_time_tcp_->send(cmd, strlen(cmd));
}

void Commander::moveJog(const std::string & axis)
{
  char cmd[100];
  snprintf(cmd, sizeof(cmd), "MoveJog(%s)", axis.c_str());
  this->real_time_tcp_->send(cmd, strlen(cmd));
}

/**
 * @brief Relative motion is performed along the tool coordinate system, and the end motion is joint motion
 *
 * @param offset1 X-axis coordinates, unit: m
 * @param offset2 Y-axis coordinates, unit: m
 * @param offset3 Z-axis coordinates, unit: m
 * @param offset4 A-axis coordinates, unit: rad
 * @param offset5 B-axis coordinates, unit: rad
 * @param offset6 C-axis coordinates, unit: rad
 */
void Commander::relMovJ(
  double offset1, double offset2, double offset3,
  double offset4, double offset5, double offset6)
{
  char cmd[100];
  snprintf(
    cmd, sizeof(cmd),
    "RelMovJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)",
    this->m2mm(offset1),
    this->m2mm(offset2),
    this->m2mm(offset3),
    this->rad2Deg(offset4),
    this->rad2Deg(offset5),
    this->rad2Deg(offset6));
  this->real_time_tcp_->send(cmd, strlen(cmd));
}

/**
 * @brief Relative motion is performed along the tool coordinate system, and the end motion is linear motion
 *
 * @param x X-axis coordinates, unit: m
 * @param y Y-axis coordinates, unit: m
 * @param z Z-axis coordinates, unit: m
 */
void Commander::relMovL(double x, double y, double z)
{
  char cmd[100];
  snprintf(
    cmd, sizeof(cmd), "RelMovL(%0.3f,%0.3f,%0.3f)",
    this->m2mm(x),
    this->m2mm(y),
    this->m2mm(z));
  this->real_time_tcp_->send(cmd, strlen(cmd));
}

/**
 * @brief Dynamic following command based on joint space
 *
 * @param j1 J1 coordinates, unit: rad
 * @param j2 J2 coordinates, unit: rad
 * @param j3 J3 coordinates, unit: rad
 * @param j4 J4 coordinates, unit: rad
 * @param j5 J5 coordinates, unit: rad
 * @param j6 J6 coordinates, unit: rad
 */
void Commander::servoJ(
  double j1, double j2, double j3,
  double j4, double j5, double j6)
{
  char cmd[100];
  snprintf(
    cmd, sizeof(cmd),
    "ServoJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)",
    this->rad2Deg(j1),
    this->rad2Deg(j2),
    this->rad2Deg(j3),
    this->rad2Deg(j4),
    this->rad2Deg(j5),
    this->rad2Deg(j6));
  this->real_time_tcp_->send(cmd, strlen(cmd));
}

/**
 * @brief Dynamic following command based on Cartesian space
 *
 * @param x X-axis coordinates, unit: m
 * @param y Y-axis coordinates, unit: m
 * @param z Z-axis coordinates, unit: m
 * @param a A-axis coordinates, unit: rad
 * @param b B-axis coordinates, unit: rad
 * @param c C-axis coordinates, unit: rad
 */
void Commander::servoP(
  double x, double y, double z,
  double a, double b, double c)
{
  char cmd[100];
  snprintf(
    cmd, sizeof(cmd),
    "ServoP(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)",
    this->m2mm(x),
    this->m2mm(y),
    this->m2mm(z),
    this->rad2Deg(a),
    this->rad2Deg(b),
    this->rad2Deg(c));
  this->real_time_tcp_->send(cmd, strlen(cmd));
}

/**
 * @brief Move from the current position to a target position in an arc interpolated mode under the Cartesian coordinate system
 *
 * @param x1 X-axis coordinates, unit: m
 * @param y1 Y-axis coordinates, unit: m
 * @param z1 Z-axis coordinates, unit: m
 * @param rx1 X-axis coordinates, unit: rad
 * @param ry1 Y-axis coordinates, unit: rad
 * @param rz1 Z-axis coordinates, unit: rad
 * @param x2 X-axis coordinates, unit: m
 * @param y2 Y-axis coordinates, unit: m
 * @param z2 Z-axis coordinates, unit: m
 * @param rx2 X-axis coordinates, unit: rad
 * @param ry2 Y-axis coordinates, unit: rad
 * @param rz2 Z-axis coordinates, unit: rad
 */
void Commander::arc(
  double x1, double y1, double z1,
  double rx1, double ry1, double rz1,
  double x2, double y2, double z2,
  double rx2, double ry2, double rz2)
{
  char cmd[100];
  snprintf(
    cmd, sizeof(cmd),
    "Arc(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)",
    this->m2mm(x1),
    this->m2mm(y1),
    this->m2mm(z1),
    this->rad2Deg(rx1),
    this->rad2Deg(ry1),
    this->rad2Deg(rz1),
    this->m2mm(x2),
    this->m2mm(y2),
    this->m2mm(z2),
    this->rad2Deg(rx2),
    this->rad2Deg(ry2),
    this->rad2Deg(rz2));
  this->real_time_tcp_->send(cmd, strlen(cmd));
}

/**
 * @brief Move from the current position to a target position in an circle interpolated mode under the Cartesian coordinate system
 *
 * @param x1 X-axis coordinates, unit: m
 * @param y1 Y-axis coordinates, unit: m
 * @param z1 Z-axis coordinates, unit: m
 * @param rx1 X-axis coordinates, unit: rad
 * @param ry1 Y-axis coordinates, unit: rad
 * @param rz1 Z-axis coordinates, unit: rad
 * @param x2 X-axis coordinates, unit: m
 * @param y2 Y-axis coordinates, unit: m
 * @param z2 Z-axis coordinates, unit: m
 * @param rx2 X-axis coordinates, unit: rad
 * @param ry2 Y-axis coordinates, unit: rad
 * @param rz2 Z-axis coordinates, unit: rad
 */
void Commander::circle(
  double x1, double y1, double z1,
  double rx1, double ry1, double rz1,
  double x2, double y2, double z2,
  double rx2, double ry2, double rz2)
{
  char cmd[100];
  snprintf(
    cmd, sizeof(cmd),
    "Circle(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)",
    this->m2mm(x1),
    this->m2mm(y1),
    this->m2mm(z1),
    this->rad2Deg(rx1),
    this->rad2Deg(ry1),
    this->rad2Deg(rz1),
    this->m2mm(x2),
    this->m2mm(y2),
    this->m2mm(z2),
    this->rad2Deg(rx2),
    this->rad2Deg(ry2),
    this->rad2Deg(rz2));
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

RealTimeData Commander::getRealTimeData()
{
  return this->real_time_data_;
}
}  // namespace mg400_interface
