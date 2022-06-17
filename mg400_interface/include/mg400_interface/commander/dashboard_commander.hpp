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

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "mg400_interface/command_utils.hpp"
#include "mg400_interface/tcp_interface/dashboard_tcp_interface.hpp"

namespace mg400_interface
{

class DashboardCommander
{
private:
  DashboardTcpInterfaceBase * tcp_if_;

public:
  DashboardCommander() = delete;
  explicit DashboardCommander(DashboardTcpInterfaceBase *);

  // DOBOT MG400 Official Command ---------------------------------------------
  bool enableRobot() const;

  bool disableRobot() const;

  void clearError() const;

  void resetRobot() const;

  void speedFactor(const int) const;

  // TODO(anyone): uncomment following function and implement it
  /**
  void user(const UserIndex &) const;

  void tool(const ToolIndex &) const;

  RobotMode robotMode() const;

  void payload(const double, const double) const;

  void DO(const DOIndex &, const DOStatus &);

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

  void getErrorId() const;

  /**
  int DI(const int) const;
  **/

  // --------------------------------------------------------------------------

private:
  static const rclcpp::Logger getLogger();
};
}  // namespace mg400_interface
