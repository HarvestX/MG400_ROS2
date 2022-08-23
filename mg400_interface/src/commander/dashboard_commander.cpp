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

int DashboardCommander::user(const UserIndex & index) const
{
  return this->user(static_cast<int>(index));
}

int DashboardCommander::user(const int index) const
{
  char buf[100];
  snprintf(buf, sizeof(buf), "User(%d)", index);
  this->tcp_if_->sendCommand(buf);
  std::string res = this->tcp_if_->recvResponse();
  return atoi(res.c_str());
}

int DashboardCommander::tool(const ToolIndex & index) const
{
  return this->tool(static_cast<int>(index));
}

int DashboardCommander::tool(const int index) const
{
  char buf[100];
  snprintf(buf, sizeof(buf), "Tool(%d)", index);
  this->tcp_if_->sendCommand(buf);
  std::string res = this->tcp_if_->recvResponse();
  return atoi(res.c_str());
}

RobotMode DashboardCommander::robotMode() const
{
  this->tcp_if_->sendCommand("RobotMode()");
  std::string res = this->tcp_if_->recvResponse();
  int mode = ResponseParser::parseOneValue(res);
  return this->robotMode(mode);
}

RobotMode DashboardCommander::robotMode(const int mode) const
{
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

bool DashboardCommander::payload(const double weight, const double inertia) const
{
  char buf[100];
  snprintf(buf, sizeof(buf), "PayLoad(%lf,%lf)", weight, inertia);
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

bool DashboardCommander::toolDOExecute(
  const ToolDOIndex && tool_do_index,
  const DOStatus && do_status) const
{
  return this->toolDOExecute(
    static_cast<int>(tool_do_index),
    static_cast<int>(do_status));
}

bool DashboardCommander::toolDOExecute(
  const int tool_do_index,
  const int do_status) const
{
  char buf[100];
  snprintf(
    buf, sizeof(buf), "ToolDOExecute(%d,%d)",
    tool_do_index, do_status);
  return this->sendCommand(buf);
}

bool DashboardCommander::accJ(const int R)
{
  char buf[100];
  snprintf(buf, sizeof(buf), "AccJ(%d)", R);
  return this->sendCommand(buf);
}

bool DashboardCommander::accL(const int R)
{
  char buf[100];
  snprintf(buf, sizeof(buf), "AccL(%d)", R);
  return this->sendCommand(buf);
}

bool DashboardCommander::speedJ(const int R)
{
  char buf[100];
  snprintf(buf, sizeof(buf), "SpeedJ(%d)", R);
  return this->sendCommand(buf);
}

bool DashboardCommander::speedL(const int R)
{
  char buf[100];
  snprintf(buf, sizeof(buf), "SpeedL(%d)", R);
  return this->sendCommand(buf);
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
bool DashboardCommander::cp(const int R)
{
  char buf[100];
  snprintf(buf, sizeof(buf), "CP(%d)", R);
  return this->sendCommand(buf);
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
bool DashboardCommander::setCollisionLevel(const CollisionLevel & level)
{
  return this->setCollisionLevel(static_cast<int>(level));
}

bool DashboardCommander::setCollisionLevel(const int level)
{
  char buf[100];
  snprintf(buf, sizeof(buf), "SetCollisionLevel(%d)", level);
  return this->sendCommand(buf);
}

std::vector<double> DashboardCommander::getAngle()
{
  this->tcp_if_->sendCommand("GetAngle()");
  std::string res = this->tcp_if_->recvResponse();
  std::vector<double> ret = ResponseParser::parsedouble(res);
  return ret;
}

std::vector<double> DashboardCommander::getPose()
{
  this->tcp_if_->sendCommand("GetPose()");
  std::string res = this->tcp_if_->recvResponse();
  std::vector<double> ret = ResponseParser::parsedouble(res);
  return ret;
}

bool DashboardCommander::emergencyStop()
{
  return this->sendCommand("EmergencyStop()");
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
  return ResponseParser::parsearray(res, count);
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
  std::vector<int> ret = ResponseParser::parsearray(res, count);
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
  std::vector<int> ret = ResponseParser::parsearray(res, count);
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
  return ResponseParser::parsearray(res, count);
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

int DashboardCommander::DI(const int index) const
{
  char buf[100];
  snprintf(buf, sizeof(buf), "DI(%d)", index);
  this->tcp_if_->sendCommand(buf);
  std::string res = this->tcp_if_->recvResponse();
  int ret = ResponseParser::parseOneValue(res);
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
