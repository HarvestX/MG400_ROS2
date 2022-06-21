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

DashboardCommander::DashboardCommander(DashboardTcpInterfaceBase * tcp_if)
: tcp_if_(tcp_if)
{
}

// DOBOT MG400 Official Command ---------------------------------------------
bool DashboardCommander::enableRobot() const
{
  this->tcp_if_->sendCommand("EnableRobot()");

  return this->tcp_if_->waitForResponseReceive("EnableRobot()");
}

bool DashboardCommander::disableRobot() const
{
  this->tcp_if_->sendCommand("DisableRobot()");

  return this->tcp_if_->waitForResponseReceive("DisableRobot()");
}

bool DashboardCommander::clearError() const
{
  this->tcp_if_->sendCommand("ClearError()");

  return this->tcp_if_->waitForResponseReceive("ClearError()");

}

bool DashboardCommander::resetRobot() const
{
  this->tcp_if_->sendCommand("ResetRobot()");

  return this->tcp_if_->waitForResponseReceive("ResetRobot()");
}

bool DashboardCommander::speedFactor(const int ratio) const
{
  char buf[100];
  snprintf(buf, sizeof(buf), "SpeedFactor(%d)", ratio);
  this->tcp_if_->sendCommand(buf);

  return this->tcp_if_->waitForResponseReceive(std::string(buf));
}

void DashboardCommander::getErrorId() const
{
  // TODO(anyone): Implement it
  this->tcp_if_->sendCommand("GetErrorID()");

  rclcpp::sleep_for(500ms);

  const std::string response = this->tcp_if_->recvResponse();
}
// End DOBOT MG400 Official Command -----------------------------------------

const rclcpp::Logger DashboardCommander::getLogger()
{
  return rclcpp::get_logger("DashboardCommander");
}
}  // namespace mg400_interface
