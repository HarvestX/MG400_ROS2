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

using namespace std::chrono_literals;  // NOLINT

class DashboardCommander
{
public:
  using SharedPtr = std::shared_ptr<DashboardCommander>;

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
  void enableRobot() const;

  void disableRobot() const;

  void clearError() const;

  void resetRobot() const;

  void speedFactor(const int) const;

  void user(const UserIndex &) const;
  void user(const int) const;

  void tool(const ToolIndex &) const;
  void tool(const int) const;

  RobotMode robotMode() const;

  void payload(const double, const double) const;

  void DO(const DOIndex &&, const DOStatus &&) const;
  void DO(const int, const int) const;

  void toolDOExecute(const ToolDOIndex &&, const DOStatus &&) const;
  void toolDOExecute(const int, const int) const;

  void accJ(const int);

  void accL(const int);

  void speedJ(const int);

  void speedL(const int);
/*
  void arch(const ArchIndex &);
  void arch(const int);
*/
  void cp(const int);
/*
  void runScript(const std::string &);

  void stopScript();

  void pauseScript();

  void continueScript();
*/
  void setCollisionLevel(const CollisionLevel &);
  void setCollisionLevel(const int);

  std::vector<double> getAngle();

  std::vector<double> getPose();

  void emergencyStop();
/*
  int modbusCreate(const std::string &, const int, const int, const int);

  bool modbusClose(const std::string &);

  std::vector<int> getInBits(const int, const int, const int);

  std::vector<int> getInRegs(const int, const int, const int, const std::string & = "U16");

  std::vector<int> getCoils(const int, const int, const int);

  int setCoils(const int, const int, const int, const std::string &);

  std::vector<int> getHoldRegs(const int, const int, const int, const std::string & = "U16");

  int setHoldRegs(const int, const int, const int, const std::string &, const std::string &);
*/
  std::array<std::vector<int>, 6> getErrorId() const;

  int DI(const int) const;

  // --------------------------------------------------------------------------

private:
  static const rclcpp::Logger getLogger();
  std::string sendAndWaitResponse(const std::string &) const;
  void evaluateResponse(const std::string &) const;
};
}  // namespace mg400_interface
