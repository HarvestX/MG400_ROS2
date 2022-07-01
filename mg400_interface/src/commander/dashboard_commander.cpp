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

#include "mg400_interface/commander/dashboard_commander.hpp"

namespace mg400_interface
{
using namespace std::chrono_literals;

DashboardCommander::DashboardCommander(
  DashboardTcpInterfaceBase * tcp_if,
  const std::chrono::nanoseconds timeout)
: tcp_if_(tcp_if),
  clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)),
  TIMEOUT(timeout)
{
}

// DOBOT MG400 Official Command ---------------------------------------------
bool DashboardCommander::enableRobot() const
{
  return this->sendCommand("EnableRobot()");
}

bool DashboardCommander::disableRobot() const
{
  return this->sendCommand("DisableRobot()");
}

bool DashboardCommander::clearError() const
{
  return this->sendCommand("ClearError()");
}

bool DashboardCommander::resetRobot() const
{
  return this->sendCommand("ResetRobot()");
}

bool DashboardCommander::speedFactor(const int ratio) const
{
  char buf[100];
  snprintf(buf, sizeof(buf), "SpeedFactor(%d)", ratio);
  return this->sendCommand(buf);
}

bool DashboardCommander::DO(
  const DOIndex && do_index,
  const DOStatus && do_status) const
{
  return this->DO(
    static_cast<int>(do_index),
    static_cast<int>(do_status));
}

bool DashboardCommander::DO(
  const int do_index,
  const int do_status) const
{
  char buf[100];
  snprintf(buf, sizeof(buf), "DO(%d,%d)", do_index, do_status);
  return this->sendCommand(buf);
}

std::array<std::vector<int>, 6> DashboardCommander::getErrorId() const
{
  this->tcp_if_->sendCommand("GetErrorID()");

  const auto start = this->clock_->now();
  while (this->clock_->now() - start < rclcpp::Duration(this->TIMEOUT)) {
    const std::string res = this->tcp_if_->recvResponse();
    if (res.find("GetErrorID()") != std::string::npos) {
      return ResponseParser::parseErrorMessage(res);
    }
  }
  std::array<std::vector<int>, 6> ret;
  return ret;
}
// End DOBOT MG400 Official Command -----------------------------------------

const rclcpp::Logger DashboardCommander::getLogger()
{
  return rclcpp::get_logger("DashboardCommander");
}

bool DashboardCommander::sendCommand(const std::string & command) const
{
  this->tcp_if_->sendCommand(command);

  const auto start = this->clock_->now();
  while (this->clock_->now() - start < rclcpp::Duration(this->TIMEOUT)) {
    const std::string res = this->tcp_if_->recvResponse();
    if (res.find(command) != std::string::npos) {
      return true;
    }
  }
  return false;
}
}  // namespace mg400_interface
