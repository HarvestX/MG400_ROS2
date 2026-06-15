/// Copyright 2025 HarvestX Inc.
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

#ifndef __MG400_PLUGIN_PLUGIN_UTILS_HPP__
#define __MG400_PLUGIN_PLUGIN_UTILS_HPP__

#include <string>
#include <mg400_interface/mg400_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mg400_plugin
{

namespace plugin_utils
{


const uint8_t SPEED_FACTOR_MIN = 1, SPEED_FACTOR_MAX = 100;
const uint8_t SPEED_J_MIN = 1, SPEED_J_MAX = 100;
const uint8_t SPEED_L_MIN = 1, SPEED_L_MAX = 100;
const uint8_t ACC_J_MIN = 1, ACC_J_MAX = 100;
const uint8_t ACC_L_MIN = 1, ACC_L_MAX = 100;
const uint8_t CP_MIN = 0, CP_MAX = 100;


template<typename T>
inline T clampWithWarning(
  T value, T min, T max, const rclcpp::Logger & logger, const std::string & arg_name)
{
  if (value < min) {
    RCLCPP_WARN(
      logger, "%s clamped from %d to %d",
      arg_name.c_str(), static_cast<int>(value), static_cast<int>(min));
    return min;
  } else if (value > max) {
    RCLCPP_WARN(
      logger, "%s clamped from %d to %d",
      arg_name.c_str(), static_cast<int>(value), static_cast<int>(max));
    return max;
  }
  return value;
}

inline int8_t clampParam(
  bool should_set, int8_t value, int8_t min, int8_t max,
  rclcpp::Logger logger, const std::string & name)
{
  return should_set ? plugin_utils::clampWithWarning(value, min, max, logger, name) : -1;
}

inline int8_t clampSpeedJ(bool should_set, int8_t value, rclcpp::Logger logger)
{
  return clampParam(
    should_set, value, plugin_utils::SPEED_J_MIN, plugin_utils::SPEED_J_MAX, logger, "speed_j");
}

inline int8_t clampAccJ(bool should_set, int8_t value, rclcpp::Logger logger)
{
  return clampParam(
    should_set, value, plugin_utils::ACC_J_MIN, plugin_utils::ACC_J_MAX, logger, "acc_j");
}

inline int8_t clampSpeedL(bool should_set, int8_t value, rclcpp::Logger logger)
{
  return clampParam(
    should_set, value, plugin_utils::SPEED_L_MIN, plugin_utils::SPEED_L_MAX, logger, "speed_l");
}

inline int8_t clampAccL(bool should_set, int8_t value, rclcpp::Logger logger)
{
  return clampParam(
    should_set, value, plugin_utils::ACC_L_MIN, plugin_utils::ACC_L_MAX, logger, "acc_l");
}

inline int8_t clampCP(bool should_set, int8_t value, rclcpp::Logger logger)
{
  return clampParam(should_set, value, plugin_utils::CP_MIN, plugin_utils::CP_MAX, logger, "cp");
}

inline bool rejectMotionCommandDuringServoMode(
  const mg400_interface::MG400Interface::SharedPtr & interface,
  const rclcpp::Logger & logger,
  const std::string & command_name)
{
  if (interface && interface->isServoModeActive()) {
    RCLCPP_ERROR(
      logger,
      "%s rejected because ServoMode is active",
      command_name.c_str());
    return true;
  }
  return false;
}

inline void exitServoModeBeforeDashboardStopCommand(
  const mg400_interface::MG400Interface::SharedPtr & interface,
  const rclcpp::Logger & logger,
  const std::string & command_name)
{
  if (!interface ||
    !interface->shouldExitServoModeOnDashboardStopCommand() ||
    !interface->isServoModeActive())
  {
    return;
  }

  RCLCPP_WARN(
    logger,
    "Exiting ServoMode before %s",
    command_name.c_str());
  interface->requestServoModeExit(command_name + " command requested", true);
}


}  // namespace plugin_utils

}  // namespace mg400_plugin

#endif  // __MG400_PLUGIN_PLUGIN_UTILS_HPP__
