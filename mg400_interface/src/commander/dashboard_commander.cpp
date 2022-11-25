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
  this->evaluateResponse(
    this->sendAndWaitResponse("EnableRobot()"));
}

void DashboardCommander::disableRobot() const
{
  this->evaluateResponse(
    this->sendAndWaitResponse("DisableRobot()"));
}

void DashboardCommander::clearError() const
{
  this->evaluateResponse(
    this->sendAndWaitResponse("ClearError()"));
}

void DashboardCommander::resetRobot() const
{
  this->evaluateResponse(
    this->sendAndWaitResponse("ResetRobot()"));
}

void DashboardCommander::speedFactor(const int ratio) const
{
  static char buf[128];
  const int cx = snprintf(buf, sizeof(buf), "SpeedFactor(%d)", ratio);
  this->evaluateResponse(this->sendAndWaitResponse(std::string(buf, cx)));
}

void DashboardCommander::user(const User::SharedPtr & user) const
{
  this->user(user->user);
}

void DashboardCommander::user(const User::_user_type & index) const
{
  static char buf[128];
  const int cx = snprintf(buf, sizeof(buf), "User(%u)", index);
  this->evaluateResponse(this->sendAndWaitResponse(std::string(buf, cx)));
}

void DashboardCommander::tool(const Tool::SharedPtr & tool) const
{
  return this->tool(tool->tool);
}

void DashboardCommander::tool(const Tool::_tool_type & index) const
{
  static char buf[128];
  const int cx = snprintf(buf, sizeof(buf), "Tool(%u)", index);
  this->evaluateResponse(this->sendAndWaitResponse(std::string(buf, cx)));
}

uint64_t DashboardCommander::robotMode() const
{
  static DashboardResponse response;
  ResponseParser::parseResponse(
    this->sendAndWaitResponse("RobotMode()"), response);
  if (!response.result) {
    throw std::runtime_error("Dobot not return 0");
  }
  return static_cast<uint64_t>(ResponseParser::takeInt(response.ret_val));
}

void DashboardCommander::payload(
  const double weight,
  const double inertia) const
{
  static char buf[128];
  const int cx = snprintf(
    buf, sizeof(buf), "PayLoad(%.3lf,%.3lf)", weight, inertia);
  this->evaluateResponse(this->sendAndWaitResponse(std::string(buf, cx)));
}

void DashboardCommander::DO(
  const DOIndex & do_index, const DOStatus & do_status) const
{
  this->DO(do_index.index, do_status.status);
}

void DashboardCommander::DO(
  const DOIndex::_index_type & do_index,
  const DOStatus::_status_type & do_status) const
{
  static char buf[128];
  const int cx = snprintf(buf, sizeof(buf), "DO(%u,%u)", do_index, do_status);
  this->evaluateResponse(this->sendAndWaitResponse(std::string(buf, cx)));
}

void DashboardCommander::toolDOExecute(
  const ToolDOIndex & tool_do_index, const DOStatus & do_status) const
{
  this->toolDOExecute(tool_do_index.index, do_status.status);
}

void DashboardCommander::toolDOExecute(
  const ToolDOIndex::_index_type & tool_do_index,
  const DOStatus::_status_type & do_status) const
{
  static char buf[128];
  const int cx = snprintf(
    buf, sizeof(buf), "ToolDOExecute(%u,%u)",
    tool_do_index, do_status);
  this->evaluateResponse(this->sendAndWaitResponse(std::string(buf, cx)));
}

void DashboardCommander::accJ(const int R)
{
  static char buf[128];
  const int cx = snprintf(buf, sizeof(buf), "AccJ(%d)", R);
  this->evaluateResponse(this->sendAndWaitResponse(std::string(buf, cx)));
}

void DashboardCommander::accL(const int R)
{
  static char buf[128];
  const int cx = snprintf(buf, sizeof(buf), "AccL(%d)", R);
  this->evaluateResponse(this->sendAndWaitResponse(std::string(buf, cx)));
}

void DashboardCommander::speedJ(const int R)
{
  static char buf[128];
  const int cx = snprintf(buf, sizeof(buf), "SpeedJ(%d)", R);
  this->evaluateResponse(this->sendAndWaitResponse(std::string(buf, cx)));
}

void DashboardCommander::speedL(const int R)
{
  static char buf[128];
  const int cx = snprintf(buf, sizeof(buf), "SpeedL(%d)", R);
  this->evaluateResponse(this->sendAndWaitResponse(std::string(buf, cx)));
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
  static char buf[128];
  const int cx = snprintf(buf, sizeof(buf), "CP(%d)", R);
  this->evaluateResponse(this->sendAndWaitResponse(std::string(buf, cx)));
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
  this->setCollisionLevel(level.level);
}

void DashboardCommander::setCollisionLevel(
  const CollisionLevel::_level_type & level)
{
  static char buf[128];
  const int cx = snprintf(buf, sizeof(buf), "SetCollisionLevel(%u)", level);
  this->evaluateResponse(this->sendAndWaitResponse(std::string(buf, cx)));
}

std::vector<double> DashboardCommander::getAngle()
{
  static DashboardResponse response;
  ResponseParser::parseResponse(
    this->sendAndWaitResponse("GetAngle()"), response);
  if (!response.result) {
    throw std::runtime_error("Dobot not return 0");
  }
  return ResponseParser::takeAngleArray(response.ret_val);
}

std::vector<double> DashboardCommander::getPose()
{
  static DashboardResponse response;
  ResponseParser::parseResponse(
    this->sendAndWaitResponse("GetPose()"), response);
  if (!response.result) {
    throw std::runtime_error("Dobot not return 0");
  }
  return ResponseParser::takePoseArray(response.ret_val);
}

void DashboardCommander::emergencyStop()
{
  this->evaluateResponse(this->sendAndWaitResponse("EmergencyStop()"));
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
  static DashboardResponse response;
  ResponseParser::parseResponse(
    this->sendAndWaitResponse("GetErrorID()"), response);
  if (!response.result) {
    throw std::runtime_error("Dobot Not return 0");
  }
  return ResponseParser::takeErrorMessage(response.ret_val);
}

int DashboardCommander::DI(const int index) const
{
  static DashboardResponse response;
  static char buf[128];
  const int cx = snprintf(buf, sizeof(buf), "DI(%d)", index);
  ResponseParser::parseResponse(
    this->sendAndWaitResponse(std::string(buf, cx)), response);
  if (!response.result) {
    throw std::runtime_error("Dobot Not return 0");
  }
  return ResponseParser::takeInt(response.ret_val);
}
// End DOBOT MG400 Official Command -----------------------------------------

const rclcpp::Logger DashboardCommander::getLogger()
{
  return rclcpp::get_logger("DashboardCommander");
}

std::string DashboardCommander::sendAndWaitResponse(
  const std::string & command) const
{
  this->tcp_if_->sendCommand(command);

  const auto start = this->clock_->now();
  const auto timeout = rclcpp::Duration(this->TIMEOUT);
  while (this->clock_->now() - start < timeout) {
    const std::string res = this->tcp_if_->recvResponse();
    if (res.find(command) != std::string::npos) {
      // Could find response candidates
      return res;
    }
  }
  throw std::runtime_error("Robot not responded.");
}

void DashboardCommander::evaluateResponse(const std::string & packet) const
{
  static DashboardResponse response;
  ResponseParser::parseResponse(packet, response);
  if (!response.result) {
    throw std::runtime_error("Dobot Not return 0");
  }
}
}  // namespace mg400_interface
