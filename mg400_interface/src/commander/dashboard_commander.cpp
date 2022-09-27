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
void DashboardCommander::enableRobot() const
{
  this->sendCommand("EnableRobot()");
}

void DashboardCommander::disableRobot() const
{
  this->sendCommand("DisableRobot()");
}

void DashboardCommander::clearError() const
{
  this->sendCommand("ClearError()");
}

void DashboardCommander::resetRobot() const
{
  this->sendCommand("ResetRobot()");
}

void DashboardCommander::speedFactor(const int ratio) const
{
  char buf[100];
  snprintf(buf, sizeof(buf), "SpeedFactor(%d)", ratio);
  this->sendCommand(buf);
}

void DashboardCommander::user(const UserIndex & index) const
{
  this->user(static_cast<int>(index));
}

void DashboardCommander::user(const int index) const
{
  char buf[100];
  snprintf(buf, sizeof(buf), "User(%d)", index);
  this->sendCommand(buf);
}

void DashboardCommander::tool(const ToolIndex & index) const
{
  return this->tool(static_cast<int>(index));
}

void DashboardCommander::tool(const int index) const
{
  char buf[100];
  snprintf(buf, sizeof(buf), "Tool(%d)", index);
  this->sendCommand(buf);
}

RobotMode DashboardCommander::robotMode() const
{
  const std::string response = this->sendCommand("RobotMode()");
  int mode = ResponseParser::parseOneValue(response);
  switch (mode) {
    case 1:
      return RobotMode::INIT;
    case 2:
      return RobotMode::BRAKE_OPEN;
    case 4:
      return RobotMode::DISABLED;
    case 5:
      return RobotMode::ENABLE;
    case 6:
      return RobotMode::BACKDRIVE;
    case 7:
      return RobotMode::RUNNING;
    case 8:
      return RobotMode::RECORDING;
    case 9:
      return RobotMode::ERROR;
    case 10:
      return RobotMode::PAUSE;
    case 11:
      return RobotMode::JOG;
    default:
      return RobotMode::INVALID;
  }
}

void DashboardCommander::payload(
  const double weight,
  const double inertia) const
{
  char buf[100];
  snprintf(buf, sizeof(buf), "PayLoad(%.3lf,%.3lf)", weight, inertia);
  this->sendCommand(buf);
}

void DashboardCommander::DO(
  const DOIndex && do_index,
  const DOStatus && do_status) const
{
  this->DO(
    static_cast<int>(do_index),
    static_cast<int>(do_status));
}

void DashboardCommander::DO(
  const int do_index,
  const int do_status) const
{
  char buf[100];
  snprintf(buf, sizeof(buf), "DO(%d,%d)", do_index, do_status);
  this->sendCommand(buf);
}

void DashboardCommander::toolDOExecute(
  const ToolDOIndex && tool_do_index,
  const DOStatus && do_status) const
{
  this->toolDOExecute(
    static_cast<int>(tool_do_index),
    static_cast<int>(do_status));
}

void DashboardCommander::toolDOExecute(
  const int tool_do_index,
  const int do_status) const
{
  char buf[100];
  snprintf(
    buf, sizeof(buf), "ToolDOExecute(%d,%d)",
    tool_do_index, do_status);
  this->sendCommand(buf);
}

void DashboardCommander::accJ(const int R)
{
  char buf[100];
  snprintf(buf, sizeof(buf), "AccJ(%d)", R);
  this->sendCommand(buf);
}

void DashboardCommander::accL(const int R)
{
  char buf[100];
  snprintf(buf, sizeof(buf), "AccL(%d)", R);
  this->sendCommand(buf);
}

void DashboardCommander::speedJ(const int R)
{
  char buf[100];
  snprintf(buf, sizeof(buf), "SpeedJ(%d)", R);
  this->sendCommand(buf);
}

void DashboardCommander::speedL(const int R)
{
  char buf[100];
  snprintf(buf, sizeof(buf), "SpeedL(%d)", R);
  this->sendCommand(buf);
}
/*
bool DashboardCommander::arch(const ArchIndex & index)
{
  return this->arch(static_cast<int>(index));
}

bool DashboardCommander::arch(const int index)
{
  char buf[100];
  snprintf(buf, sizeof(buf), "Arch(%d)", index);
  return this->sendCommand(buf);
}
*/
void DashboardCommander::cp(const int R)
{
  char buf[100];
  snprintf(buf, sizeof(buf), "CP(%d)", R);
  this->sendCommand(buf);
}
/*
bool DashboardCommander::runScript(const std::string & name)
{
  char buf[100];
  snprintf(buf, sizeof(buf), "RunScript(%s)", name.c_str());
  return this->sendCommand(buf);
}

bool DashboardCommander::stopScript()
{
  return this->sendCommand("StopScript()");
}

bool DashboardCommander::pauseScript()
{
  return this->sendCommand("PauseScript()");
}

bool DashboardCommander::continueScript()
{
  return this->sendCommand("ContinueScript()");
}
*/
void DashboardCommander::setCollisionLevel(const CollisionLevel & level)
{
  this->setCollisionLevel(static_cast<int>(level));
}

void DashboardCommander::setCollisionLevel(const int level)
{
  char buf[100];
  snprintf(buf, sizeof(buf), "SetCollisionLevel(%d)", level);
  this->sendCommand(buf);
}

std::vector<double> DashboardCommander::getAngle()
{
  const std::string res = this->sendCommand("GetAngle()");
  std::vector<double> ret = ResponseParser::parseDouble(res);
  return ret;
}

std::vector<double> DashboardCommander::getPose()
{
  const std::string res = this->sendCommand("GetPose()");
  std::vector<double> ret = ResponseParser::parseDouble(res);
  return ret;
}

void DashboardCommander::emergencyStop()
{
  this->sendCommand("EmergencyStop()");
}
/*
int DashboardCommander::modbusCreate(
  const std::string & ip, const int port,
  const int slave_id, const int isRTU)
{
  char buf[100];
  snprintf(
    buf, sizeof(buf), "ModbusCreate(%s,%d,%d,%d)",
    ip.c_str(), port, slave_id, isRTU);
  this->tcp_if_->sendCommand(buf);
  std::string res = this->tcp_if_->recvResponse();
  int ret = ResponseParser::parseOneValue(res);
  return ret;
}

bool DashboardCommander::modbusClose(const std::string & index)
{
  char buf[100];
  snprintf(buf, sizeof(buf), "ModbusClose(%s)", index.c_str());
  return sendCommand(buf);
}

std::vector<int> DashboardCommander::getInBits(
  const int index, const int addr, const int count)
{
  char buf[100];
  snprintf(
    buf, sizeof(buf), "GetInBits(%d,%d,%d)",
    index, addr, count);
  this->tcp_if_->sendCommand(buf);
  std::string res = this->tcp_if_->recvResponse();
  return ResponseParser::parseArray(res, count);
}

std::vector<int> DashboardCommander::getInRegs(
  const int index, const int addr,
  const int count, const std::string & valType)
{
  char buf[100];
  snprintf(
    buf, sizeof(buf), "GetInRegs(%d,%d,%d,{%s})",
    index, addr, count, valType.c_str());
  this->tcp_if_->sendCommand(buf);
  std::string res = this->tcp_if_->recvResponse();
  std::vector<int> ret = ResponseParser::parseArray(res, count);
  return ret;
}

std::vector<int> DashboardCommander::getCoils(
  const int index, const int addr, const int count)
{
  char buf[100];
  snprintf(
    buf, sizeof(buf), "GetCoils(%d,%d,%d)",
    index, addr, count);
  this->tcp_if_->sendCommand(buf);
  std::string res = this->tcp_if_->recvResponse();
  std::vector<int> ret = ResponseParser::parseArray(res, count);
  return ret;
}

int DashboardCommander::setCoils(
  const int index, const int addr,
  const int count, const std::string & valTab)
{
  char buf[100];
  snprintf(
    buf, sizeof(buf), "SetCoils(%d,%d,%d,{%s})",
    index, addr, count, valTab.c_str());
  this->tcp_if_->sendCommand(buf);
  std::string res = this->tcp_if_->recvResponse();
  int ret = ResponseParser::parseOnlyErrorID(res);
  return ret;
}

std::vector<int> DashboardCommander::getHoldRegs(
  const int index, const int addr,
  const int count, const std::string & valType)
{
  char buf[100];
  snprintf(
    buf, sizeof(buf), "GetHoldRegs(%d,%d,%d,%s)",
    index, addr, count, valType.c_str());
  this->tcp_if_->sendCommand(buf);
  std::string res = this->tcp_if_->recvResponse();
  return ResponseParser::parseArray(res, count);
}

int DashboardCommander::setHoldRegs(
  const int index, const int addr,
  const int count, const std::string & valTab, const std::string & valType)
{
  char buf[100];
  snprintf(
    buf, sizeof(buf), "SetHoldRegs(%d,%d,%d,{%s},%s)",
    index, addr, count, valTab.c_str(), valType.c_str());
  this->tcp_if_->sendCommand(buf);
  std::string res = this->tcp_if_->recvResponse();
  return ResponseParser::parseOnlyErrorID(res);
}
*/

std::array<std::vector<int>, 6> DashboardCommander::getErrorId() const
{
  const std::string res = this->sendCommand("GetErrorID()");
  return ResponseParser::parseErrorMessage(res);
}

int DashboardCommander::DI(const int index) const
{
  char buf[100];
  snprintf(buf, sizeof(buf), "DI(%d)", index);
  const std::string res = this->sendCommand(buf);
  return ResponseParser::parseOneValue(res);
}
// End DOBOT MG400 Official Command -----------------------------------------

const rclcpp::Logger DashboardCommander::getLogger()
{
  return rclcpp::get_logger("DashboardCommander");
}

std::string DashboardCommander::sendCommand(
  const std::string & command) const
{
  this->tcp_if_->sendCommand(command);

  const auto start = this->clock_->now();
  const auto timeout = rclcpp::Duration(this->TIMEOUT);
  while (this->clock_->now() - start < timeout) {
    const std::string res = this->tcp_if_->recvResponse();
    if (res.find(command) != std::string::npos) {
      return res;
    }
  }
  throw std::runtime_error("Robot not responded.");
}
}  // namespace mg400_interface
