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

#include <mg400_msgs/msg/arch.hpp>
#include <mg400_msgs/msg/collision_level.hpp>
#include <mg400_msgs/msg/di_index.hpp>
#include <mg400_msgs/msg/do_index.hpp>
#include <mg400_msgs/msg/do_status.hpp>
#include <mg400_msgs/msg/tool_do_index.hpp>
#include <mg400_msgs/msg/tool.hpp>
#include <mg400_msgs/msg/user.hpp>
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
  RCLCPP_SHARED_PTR_DEFINITIONS(DashboardCommander)
  RCLCPP_UNIQUE_PTR_DEFINITIONS(DashboardCommander)

private:
  using ArchIndex = mg400_msgs::msg::Arch;
  using CollisionLevel = mg400_msgs::msg::CollisionLevel;
  using DIIndex = mg400_msgs::msg::DIIndex;
  using DOIndex = mg400_msgs::msg::DOIndex;
  using DOStatus = mg400_msgs::msg::DOStatus;
  using Tool = mg400_msgs::msg::Tool;
  using ToolDOIndex = mg400_msgs::msg::ToolDOIndex;
  using User = mg400_msgs::msg::User;

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

  void user(const User &) const;
  void user(const User::_user_type &) const;

  void tool(const Tool &) const;
  void tool(const Tool::_tool_type &) const;

  uint64_t robotMode() const;

  void payLoad(const double, const double) const;

  void DO(
    const DOIndex &,
    const DOStatus &) const;
  void DO(
    const DOIndex::_index_type &,
    const DOStatus::_status_type &) const;

  void toolDOExecute(
    const ToolDOIndex &,
    const DOStatus &) const;
  void toolDOExecute(
    const ToolDOIndex::_index_type &,
    const DOStatus::_status_type &) const;

  void accJ(const int);

  void accL(const int);

  void speedJ(const int);

  void speedL(const int);

  void arch(const ArchIndex &);
  void arch(const ArchIndex::_index_type &);

  void cp(const int);
/*
  void runScript(const std::string &);

  void stopScript();

  void pauseScript();

  void continueScript();
*/
  void setCollisionLevel(const CollisionLevel &);
  void setCollisionLevel(const CollisionLevel::_level_type &);

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

  int DI(const DIIndex &) const;
  int DI(const DIIndex::_index_type &) const;

  // --------------------------------------------------------------------------

private:
  static const rclcpp::Logger getLogger();
  std::string sendAndWaitResponse(const std::string &) const;
  void evaluateResponse(const std::string &) const;
};
}  // namespace mg400_interface
