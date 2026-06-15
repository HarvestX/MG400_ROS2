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

#ifndef __MG400_INTERFACE_COMMAND_BUILDER_HPP__
#define __MG400_INTERFACE_COMMAND_BUILDER_HPP__

#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

#include "mg400_interface/command_utils.hpp"

namespace mg400_interface
{

class CommandBuilder
{
public:
  CommandBuilder() = default;
  explicit CommandBuilder(const std::string & command_name);

  void setCommandName(const std::string & command_name);

  CommandBuilder & addParameter(const int & value);
  CommandBuilder & addParameter(const double & value, int precision = 3);
  CommandBuilder & addParameter(const std::string & value);
  CommandBuilder & addParameter(const char * value);

  CommandBuilder & addPositionParameter(const si_m & x, const si_m & y, const si_m & z);
  CommandBuilder & addRotationParameter(const si_rad & rotation);
  CommandBuilder & addJointParameters(
    const si_rad & j1, const si_rad & j2, const si_rad & j3,
    const si_rad & j4);

  CommandBuilder & addOptionalParameter(const std::string & name, const int & value);
  CommandBuilder & addOptionalParameter(
    const std::string & name, const double & value,
    int precision = 3);

  std::string build() const;

  void clear();

  static CommandBuilder create(const std::string & command_name);

  // ========================================================================
  // Dashboard Port (29999) Commands
  // ========================================================================

  // Dashboard Port - Non-Queue Commands (Immediate execution)
  static std::string buildEnableRobot();
  static std::string buildEnableRobot(const double & load);
  static std::string buildEnableRobot(
    const double & load, const double & center_x,
    const double & center_y, const double & center_z);
  static std::string buildDisableRobot();
  static std::string buildClearError();
  static std::string buildResetRobot();
  static std::string buildSpeedFactor(const int & ratio);
  static std::string buildUser(const int & user_index);
  static std::string buildTool(const int & tool_index);
  static std::string buildRobotMode();
  static std::string buildPayLoad(const double & weight, const double & inertia);
  static std::string buildToolDOExecute(const int & index, const int & status);
  static std::string buildToolDI(const int & index);
  static std::string buildAccJ(const int & acceleration);
  static std::string buildAccL(const int & acceleration);
  static std::string buildSpeedJ(const int & speed);
  static std::string buildSpeedL(const int & speed);
  static std::string buildArch(const int & arch_index);
  static std::string buildCP(const int & cp_value);
  static std::string buildSetCollisionLevel(const int & level);
  static std::string buildGetAngle();
  static std::string buildGetPose();
  static std::string buildEmergencyStop();
  static std::string buildGetErrorID();
  static std::string buildDI(const int & index);

  // Dashboard Port - Queue Commands (Added to execution queue)
  static std::string buildDO(const int & index, const int & status);
  static std::string buildToolDO(const int & index, const int & status);
  static std::string buildAO(const int & index, const double & value);

  // ========================================================================
  // Motion Port (30003) Commands - All Queue Commands
  // ========================================================================

  static std::string buildServoJ(
    const si_rad & j1, const si_rad & j2, const si_rad & j3,
    const si_rad & j4, const double & t, const double & aheadtime,
    const double & gain);
  static std::string buildServoP(
    const si_m & x, const si_m & y, const si_m & z,
    const si_rad & rx, const si_rad & ry, const si_rad & rz,
    const double & t, const double & aheadtime, const double & gain);
  static std::string buildMovJ(
    const si_m & x, const si_m & y, const si_m & z, const si_rad & r,
    const int & speed_j = -1, const int & acc_j = -1, const int & cp = -1);
  static std::string buildMovL(
    const si_m & x, const si_m & y, const si_m & z, const si_rad & r,
    const int & speed_l = -1, const int & acc_l = -1, const int & cp = -1);
  static std::string buildJointMovJ(
    const si_rad & j1, const si_rad & j2, const si_rad & j3, const si_rad & j4,
    const int & speed_j = -1, const int & acc_j = -1, const int & cp = -1);
  static std::string buildMovLIO(
    const si_m & x, const si_m & y, const si_m & z, const si_rad & r,
    const int & distance_mode, const int & distance,
    const int & do_index, const int & do_status,
    const int & speed_l = -1, const int & acc_l = -1, const int & cp = -1);
  static std::string buildMovJIO(
    const si_m & x, const si_m & y, const si_m & z, const si_rad & r,
    const int & distance_mode, const int & distance,
    const int & do_index, const int & do_status,
    const int & speed_j = -1, const int & acc_j = -1, const int & cp = -1);
  static std::string buildArc(
    const si_m & x1, const si_m & y1, const si_m & z1, const si_rad & rx1, const si_rad & ry1,
    const si_rad & rz1,
    const si_m & x2, const si_m & y2, const si_m & z2, const si_rad & rx2, const si_rad & ry2,
    const si_rad & rz2);
  static std::string buildRelMovJTool(
    const si_m & offset_x, const si_m & offset_y, const si_m & offset_z,
    const si_rad & offset_rx, const si_rad & offset_ry, const si_rad & offset_rz,
    const int & tool_index);
  static std::string buildRelMovLTool(
    const si_m & offset_x, const si_m & offset_y, const si_m & offset_z,
    const si_rad & offset_rx, const si_rad & offset_ry, const si_rad & offset_rz,
    const int & tool_index);
  static std::string buildRelMovJUser(
    const si_m & offset_x, const si_m & offset_y, const si_m & offset_z,
    const si_rad & offset_rx, const si_rad & offset_ry, const si_rad & offset_rz,
    const int & user_index);
  static std::string buildRelMovLUser(
    const si_m & offset_x, const si_m & offset_y, const si_m & offset_z,
    const si_rad & offset_rx, const si_rad & offset_ry, const si_rad & offset_rz,
    const int & user_index);
  static std::string buildRelJointMovJ(
    const si_rad & offset1, const si_rad & offset2, const si_rad & offset3,
    const si_rad & offset4, const si_rad & offset5, const si_rad & offset6);

  // Motion Port - Non-Queue Commands (if any)
  static std::string buildMoveJog(const int & jog_mode);
  static std::string buildSync();

private:
  std::string command_name_;
  std::vector<std::string> parameters_;
  std::vector<std::string> optional_parameters_;

  std::string formatDouble(const double & value, int precision) const;
};

}  // namespace mg400_interface

#endif
