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

#pragma once

#include <array>
#include <vector>
#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "mg400_interface/commander/response_parser.hpp"
#include "mg400_interface/command_utils.hpp"
#include "mg400_interface/tcp_interface/dashboard_tcp_interface.hpp"

namespace mg400_interface
{

using namespace std::chrono_literals;

class DashboardCommander
{
private:
  DashboardTcpInterfaceBase * tcp_if_;
  rclcpp::Clock::SharedPtr clock_;
  const std::chrono::nanoseconds TIMEOUT;

public:
  DashboardCommander() = delete;
  explicit DashboardCommander(
    DashboardTcpInterfaceBase *,
    const std::chrono::nanoseconds = 5s);

  // DOBOT MG400 Official Command ---------------------------------------------
  bool enableRobot() const;

  bool disableRobot() const;

  bool clearError() const;

  bool resetRobot() const;

  bool speedFactor(const int) const;

  // TODO(anyone): uncomment following function and implement it
  /**
  void user(const UserIndex &) const;

  void tool(const ToolIndex &) const;

  RobotMode robotMode() const;

  void payload(const double, const double) const;
  **/

  bool DO(const DOIndex &&, const DOStatus &&) const;
  bool DO(const int, const int) const;

  bool toolDOExecute(const ToolDOIndex &&, const DOStatus &&) const;
  bool toolDOExecute(const int, const int) const;

  // TODO(anyone): uncomment following function and implement it
  /**
  void accJ(const int);

  void accL(const int);

  void speedJ(const int);

  void speedL(const int);

  void arch(const ArchIndex &);

  void cp(const int);

  void runScript(const std::string &);

  // void stopScript();

  // void pauseScript();

  void continueScript();

  void setCollisionLevel(const CollisionLevel &);

  void getAngle();

  void getPose();

  void emergencyStop();

  // void modbusCreate();

  // void modbusClose();

  // void getInBits();

  // void getInRegs();

  // void getCoils();

  // void setCoils();

  // void getHoldRegs();

  // void setHoldRegs();
  **/

  std::array<std::vector<int>, 6> getErrorId() const;

  /**
  int DI(const int) const;
  **/

  // --------------------------------------------------------------------------

private:
  static const rclcpp::Logger getLogger();
  bool sendCommand(const std::string &) const;
};
}  // namespace mg400_interface
