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

#include "mg400_interface/command_builder.hpp"

namespace mg400_interface
{

CommandBuilder::CommandBuilder(const std::string & command_name)
: command_name_(command_name)
{
}

void CommandBuilder::setCommandName(const std::string & command_name)
{
  command_name_ = command_name;
}

CommandBuilder & CommandBuilder::addParameter(const int & value)
{
  parameters_.push_back(std::to_string(value));
  return *this;
}

CommandBuilder & CommandBuilder::addParameter(const double & value, int precision)
{
  parameters_.push_back(formatDouble(value, precision));
  return *this;
}

CommandBuilder & CommandBuilder::addParameter(const std::string & value)
{
  parameters_.push_back(value);
  return *this;
}

CommandBuilder & CommandBuilder::addParameter(const char * value)
{
  parameters_.push_back(std::string(value));
  return *this;
}

CommandBuilder & CommandBuilder::addPositionParameter(
  const si_m & x, const si_m & y,
  const si_m & z)
{
  parameters_.push_back(formatDouble(m2mm(x), 3));
  parameters_.push_back(formatDouble(m2mm(y), 3));
  parameters_.push_back(formatDouble(m2mm(z), 3));
  return *this;
}

CommandBuilder & CommandBuilder::addRotationParameter(const si_rad & rotation)
{
  parameters_.push_back(formatDouble(rad2degree(rotation), 3));
  return *this;
}

CommandBuilder & CommandBuilder::addJointParameters(
  const si_rad & j1, const si_rad & j2,
  const si_rad & j3, const si_rad & j4)
{
  parameters_.push_back(formatDouble(rad2degree(j1), 3));
  parameters_.push_back(formatDouble(rad2degree(j2), 3));
  parameters_.push_back(formatDouble(rad2degree(j3), 3));
  parameters_.push_back(formatDouble(rad2degree(j4), 3));
  return *this;
}

CommandBuilder & CommandBuilder::addOptionalParameter(const std::string & name, const int & value)
{
  optional_parameters_.push_back(name + "=" + std::to_string(value));
  return *this;
}

CommandBuilder & CommandBuilder::addOptionalParameter(
  const std::string & name,
  const double & value, int precision)
{
  optional_parameters_.push_back(name + "=" + formatDouble(value, precision));
  return *this;
}

std::string CommandBuilder::build() const
{
  std::ostringstream oss;
  oss << command_name_ << "(";

  for (size_t i = 0; i < parameters_.size(); ++i) {
    if (i > 0) {
      oss << ",";
    }
    oss << parameters_[i];
  }

  for (size_t i = 0; i < optional_parameters_.size(); ++i) {
    if (!parameters_.empty() || i > 0) {
      oss << ",";
    }
    oss << optional_parameters_[i];
  }

  oss << ")";
  return oss.str();
}

void CommandBuilder::clear()
{
  command_name_.clear();
  parameters_.clear();
  optional_parameters_.clear();
}

CommandBuilder CommandBuilder::create(const std::string & command_name)
{
  return CommandBuilder(command_name);
}

// Dashboard command builders
std::string CommandBuilder::buildEnableRobot()
{
  return "EnableRobot()";
}

std::string CommandBuilder::buildEnableRobot(const double & load)
{
  return CommandBuilder("EnableRobot")
         .addParameter(load, 3)
         .build();
}

std::string CommandBuilder::buildEnableRobot(
  const double & load, const double & center_x,
  const double & center_y, const double & center_z)
{
  return CommandBuilder("EnableRobot")
         .addParameter(load, 3)
         .addParameter(center_x, 3)
         .addParameter(center_y, 3)
         .addParameter(center_z, 3)
         .build();
}

std::string CommandBuilder::buildDisableRobot()
{
  return "DisableRobot()";
}

std::string CommandBuilder::buildClearError()
{
  return "ClearError()";
}

std::string CommandBuilder::buildResetRobot()
{
  return "ResetRobot()";
}

std::string CommandBuilder::buildSpeedFactor(const int & ratio)
{
  return CommandBuilder("SpeedFactor")
         .addParameter(ratio)
         .build();
}

std::string CommandBuilder::buildUser(const int & user_index)
{
  return CommandBuilder("User")
         .addParameter(user_index)
         .build();
}

std::string CommandBuilder::buildTool(const int & tool_index)
{
  return CommandBuilder("Tool")
         .addParameter(tool_index)
         .build();
}

std::string CommandBuilder::buildRobotMode()
{
  return "RobotMode()";
}

std::string CommandBuilder::buildPayLoad(const double & weight, const double & inertia)
{
  return CommandBuilder("PayLoad")
         .addParameter(weight, 3)
         .addParameter(inertia, 3)
         .build();
}

std::string CommandBuilder::buildDO(const int & index, const int & status)
{
  return CommandBuilder("DO")
         .addParameter(index)
         .addParameter(status)
         .build();
}

std::string CommandBuilder::buildToolDOExecute(const int & index, const int & status)
{
  return CommandBuilder("ToolDOExecute")
         .addParameter(index)
         .addParameter(status)
         .build();
}

std::string CommandBuilder::buildToolDI(const int & index)
{
  return CommandBuilder("ToolDI")
         .addParameter(index)
         .build();
}

std::string CommandBuilder::buildAccJ(const int & acceleration)
{
  return CommandBuilder("AccJ")
         .addParameter(acceleration)
         .build();
}

std::string CommandBuilder::buildAccL(const int & acceleration)
{
  return CommandBuilder("AccL")
         .addParameter(acceleration)
         .build();
}

std::string CommandBuilder::buildSpeedJ(const int & speed)
{
  return CommandBuilder("SpeedJ")
         .addParameter(speed)
         .build();
}

std::string CommandBuilder::buildSpeedL(const int & speed)
{
  return CommandBuilder("SpeedL")
         .addParameter(speed)
         .build();
}

std::string CommandBuilder::buildArch(const int & arch_index)
{
  return CommandBuilder("Arch")
         .addParameter(arch_index)
         .build();
}

std::string CommandBuilder::buildCP(const int & cp_value)
{
  return CommandBuilder("CP")
         .addParameter(cp_value)
         .build();
}

std::string CommandBuilder::buildSetCollisionLevel(const int & level)
{
  return CommandBuilder("SetCollisionLevel")
         .addParameter(level)
         .build();
}

std::string CommandBuilder::buildGetAngle()
{
  return "GetAngle()";
}

std::string CommandBuilder::buildGetPose()
{
  return "GetPose()";
}

std::string CommandBuilder::buildEmergencyStop()
{
  return "EmergencyStop()";
}

std::string CommandBuilder::buildGetErrorID()
{
  return "GetErrorID()";
}

std::string CommandBuilder::buildDI(const int & index)
{
  return CommandBuilder("DI")
         .addParameter(index)
         .build();
}

// Dashboard Port - Queue Commands
std::string CommandBuilder::buildToolDO(const int & index, const int & status)
{
  return CommandBuilder("ToolDO")
         .addParameter(index)
         .addParameter(status)
         .build();
}

std::string CommandBuilder::buildAO(const int & index, const double & value)
{
  return CommandBuilder("AO")
         .addParameter(index)
         .addParameter(value, 1)
         .build();
}

// Motion command builders
std::string CommandBuilder::buildServoJ(
  const si_rad & j1, const si_rad & j2, const si_rad & j3,
  const si_rad & j4, const double & t, const double & aheadtime,
  const double & gain)
{
  (void)t;
  (void)aheadtime;
  (void)gain;

  return CommandBuilder("ServoJ")
         .addParameter(rad2degree(j1), 3)
         .addParameter(rad2degree(j2), 3)
         .addParameter(rad2degree(j3), 3)
         .addParameter(rad2degree(j4), 3)
         .addParameter(0.0, 3)
         .addParameter(0.0, 3)
         .build();
}

std::string CommandBuilder::buildServoP(
  const si_m & x, const si_m & y, const si_m & z,
  const si_rad & rx, const si_rad & ry, const si_rad & rz,
  const double & t, const double & aheadtime, const double & gain)
{
  (void)t;
  (void)aheadtime;
  (void)gain;

  return CommandBuilder("ServoP")
         .addPositionParameter(x, y, z)
         .addParameter(rad2degree(rx), 3)
         .addParameter(rad2degree(ry), 3)
         .addParameter(rad2degree(rz), 3)
         .build();
}

std::string CommandBuilder::buildMovJ(
  const si_m & x, const si_m & y, const si_m & z, const si_rad & r,
  const int & speed_j, const int & acc_j, const int & cp)
{
  CommandBuilder builder("MovJ");
  builder.addPositionParameter(x, y, z)
  .addRotationParameter(r);

  if (speed_j >= 0) {
    builder.addOptionalParameter("SpeedJ", speed_j);
  }
  if (acc_j >= 0) {
    builder.addOptionalParameter("AccJ", acc_j);
  }
  if (cp >= 0) {
    builder.addOptionalParameter("CP", cp);
  }

  return builder.build();
}

std::string CommandBuilder::buildMovL(
  const si_m & x, const si_m & y, const si_m & z, const si_rad & r,
  const int & speed_l, const int & acc_l, const int & cp)
{
  CommandBuilder builder("MovL");
  builder.addPositionParameter(x, y, z)
  .addRotationParameter(r);

  if (speed_l >= 0) {
    builder.addOptionalParameter("SpeedL", speed_l);
  }
  if (acc_l >= 0) {
    builder.addOptionalParameter("AccL", acc_l);
  }
  if (cp >= 0) {
    builder.addOptionalParameter("CP", cp);
  }

  return builder.build();
}

std::string CommandBuilder::buildJointMovJ(
  const si_rad & j1, const si_rad & j2, const si_rad & j3, const si_rad & j4,
  const int & speed_j, const int & acc_j, const int & cp)
{
  CommandBuilder builder("JointMovJ");
  builder.addJointParameters(j1, j2, j3, j4);

  if (speed_j >= 0) {
    builder.addOptionalParameter("SpeedJ", speed_j);
  }
  if (acc_j >= 0) {
    builder.addOptionalParameter("AccJ", acc_j);
  }
  if (cp >= 0) {
    builder.addOptionalParameter("CP", cp);
  }

  return builder.build();
}

std::string CommandBuilder::buildMovLIO(
  const si_m & x, const si_m & y, const si_m & z, const si_rad & r,
  const int & distance_mode, const int & distance,
  const int & do_index, const int & do_status,
  const int & speed_l, const int & acc_l, const int & cp)
{
  CommandBuilder builder("MovLIO");
  builder.addPositionParameter(x, y, z)
  .addRotationParameter(r)
  .addParameter(distance_mode)
  .addParameter(distance)
  .addParameter(do_index)
  .addParameter(do_status);

  if (speed_l >= 0) {
    builder.addOptionalParameter("SpeedL", speed_l);
  }
  if (acc_l >= 0) {
    builder.addOptionalParameter("AccL", acc_l);
  }
  if (cp >= 0) {
    builder.addOptionalParameter("CP", cp);
  }

  return builder.build();
}

std::string CommandBuilder::buildMovJIO(
  const si_m & x, const si_m & y, const si_m & z, const si_rad & r,
  const int & distance_mode, const int & distance,
  const int & do_index, const int & do_status,
  const int & speed_j, const int & acc_j, const int & cp)
{
  CommandBuilder builder("MovJIO");
  builder.addPositionParameter(x, y, z)
  .addRotationParameter(r)
  .addParameter(distance_mode)
  .addParameter(distance)
  .addParameter(do_index)
  .addParameter(do_status);

  if (speed_j >= 0) {
    builder.addOptionalParameter("SpeedJ", speed_j);
  }
  if (acc_j >= 0) {
    builder.addOptionalParameter("AccJ", acc_j);
  }
  if (cp >= 0) {
    builder.addOptionalParameter("CP", cp);
  }

  return builder.build();
}

std::string CommandBuilder::buildMoveJog(const int & jog_mode)
{
  return CommandBuilder("MoveJog")
         .addParameter(jog_mode)
         .build();
}

std::string CommandBuilder::buildSync()
{
  return "Sync()";
}

std::string CommandBuilder::buildArc(
  const si_m & x1, const si_m & y1, const si_m & z1, const si_rad & rx1, const si_rad & ry1,
  const si_rad & rz1,
  const si_m & x2, const si_m & y2, const si_m & z2, const si_rad & rx2, const si_rad & ry2,
  const si_rad & rz2)
{
  CommandBuilder builder("Arc");
  builder.addPositionParameter(x1, y1, z1)
  .addParameter(rad2degree(rx1), 3)
  .addParameter(rad2degree(ry1), 3)
  .addParameter(rad2degree(rz1), 3)
  .addPositionParameter(x2, y2, z2)
  .addParameter(rad2degree(rx2), 3)
  .addParameter(rad2degree(ry2), 3)
  .addParameter(rad2degree(rz2), 3);
  return builder.build();
}

std::string CommandBuilder::buildRelMovJTool(
  const si_m & offset_x, const si_m & offset_y, const si_m & offset_z,
  const si_rad & offset_rx, const si_rad & offset_ry, const si_rad & offset_rz,
  const int & tool_index)
{
  CommandBuilder builder("RelMovJTool");
  builder.addPositionParameter(offset_x, offset_y, offset_z)
  .addParameter(rad2degree(offset_rx), 3)
  .addParameter(rad2degree(offset_ry), 3)
  .addParameter(rad2degree(offset_rz), 3)
  .addParameter(tool_index);
  return builder.build();
}

std::string CommandBuilder::buildRelMovLTool(
  const si_m & offset_x, const si_m & offset_y, const si_m & offset_z,
  const si_rad & offset_rx, const si_rad & offset_ry, const si_rad & offset_rz,
  const int & tool_index)
{
  CommandBuilder builder("RelMovLTool");
  builder.addPositionParameter(offset_x, offset_y, offset_z)
  .addParameter(rad2degree(offset_rx), 3)
  .addParameter(rad2degree(offset_ry), 3)
  .addParameter(rad2degree(offset_rz), 3)
  .addParameter(tool_index);
  return builder.build();
}

std::string CommandBuilder::buildRelMovJUser(
  const si_m & offset_x, const si_m & offset_y, const si_m & offset_z,
  const si_rad & offset_rx, const si_rad & offset_ry, const si_rad & offset_rz,
  const int & user_index)
{
  CommandBuilder builder("RelMovJUser");
  builder.addPositionParameter(offset_x, offset_y, offset_z)
  .addParameter(rad2degree(offset_rx), 3)
  .addParameter(rad2degree(offset_ry), 3)
  .addParameter(rad2degree(offset_rz), 3)
  .addParameter(user_index);
  return builder.build();
}

std::string CommandBuilder::buildRelMovLUser(
  const si_m & offset_x, const si_m & offset_y, const si_m & offset_z,
  const si_rad & offset_rx, const si_rad & offset_ry, const si_rad & offset_rz,
  const int & user_index)
{
  CommandBuilder builder("RelMovLUser");
  builder.addPositionParameter(offset_x, offset_y, offset_z)
  .addParameter(rad2degree(offset_rx), 3)
  .addParameter(rad2degree(offset_ry), 3)
  .addParameter(rad2degree(offset_rz), 3)
  .addParameter(user_index);
  return builder.build();
}

std::string CommandBuilder::buildRelJointMovJ(
  const si_rad & offset1, const si_rad & offset2, const si_rad & offset3,
  const si_rad & offset4, const si_rad & offset5, const si_rad & offset6)
{
  CommandBuilder builder("RelJointMovJ");
  builder.addParameter(rad2degree(offset1), 3)
  .addParameter(rad2degree(offset2), 3)
  .addParameter(rad2degree(offset3), 3)
  .addParameter(rad2degree(offset4), 3)
  .addParameter(rad2degree(offset5), 3)
  .addParameter(rad2degree(offset6), 3);
  return builder.build();
}

std::string CommandBuilder::formatDouble(const double & value, int precision) const
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << value;
  return oss.str();
}

}  // namespace mg400_interface
