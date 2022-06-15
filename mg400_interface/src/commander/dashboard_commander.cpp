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
DashboardCommander::DashboardCommander(TcpInterfaceBase * tcp_if)
: tcp_if_(tcp_if)
{
}

// DOBOT MG400 Official Command ---------------------------------------------
void DashboardCommander::enableRobot() const
{
  this->tcp_if_->sendCommand("EnableRobot()");
}

void DashboardCommander::disableRobot()const
{
  this->tcp_if_->sendCommand("DisableRobot()");
}

void DashboardCommander::clearError() const
{
  this->tcp_if_->sendCommand("ClearError()");
}

void DashboardCommander::resetRobot() const
{
  this->tcp_if_->sendCommand("ResetRobot()");
}

void DashboardCommander::speedFactor(const int ratio) const
{
  char buf[100];
  snprintf(buf, sizeof(buf), "SpeedFactor(%d)", ratio);
  this->tcp_if_->sendCommand(buf);
}

void DashboardCommander::user(const UserIndex &) const
{
  // TODO(anyone): Implement it
}

void DashboardCommander::tool(const ToolIndex &) const
{
  // TODO(anyone): Implement it
}

RobotMode DashboardCommander::robotMode() const
{
  // TODO(anyone): Implement it
}

void DashboardCommander::payload(const double, const double) const
{
  // TODO(anyone): Implement it
}

void DashboardCommander::getErrorId() const
{
  this->tcp_if_->sendCommand("GetErrorID()");
}
// End DOBOT MG400 Official Command -----------------------------------------

}  // namespace mg400_interface
