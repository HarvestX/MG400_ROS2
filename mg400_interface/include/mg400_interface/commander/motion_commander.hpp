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

#ifndef __MG400_INTERFACE_COMMANDER_MOTION_COMMANDER_HPP__
#define __MG400_INTERFACE_COMMANDER_MOTION_COMMANDER_HPP__

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <mg400_msgs/msg/distance_mode.hpp>
#include <mg400_msgs/msg/do_index.hpp>
#include <mg400_msgs/msg/do_status.hpp>
#include <mg400_msgs/msg/move_jog.hpp>
#include <mg400_msgs/msg/tool_do_index.hpp>
#include <mg400_msgs/msg/user.hpp>

#include "mg400_interface/command_utils.hpp"
#include "mg400_interface/commander/response_parser.hpp"
#include "mg400_interface/tcp_interface/motion_tcp_interface.hpp"

namespace mg400_interface
{
class MotionCommander
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MotionCommander)
  RCLCPP_UNIQUE_PTR_DEFINITIONS(MotionCommander)

private:
  using DistanceMode = mg400_msgs::msg::DistanceMode;
  using DOIndex = mg400_msgs::msg::DOIndex;
  using DOStatus = mg400_msgs::msg::DOStatus;
  using MoveJog = mg400_msgs::msg::MoveJog;
  using User = mg400_msgs::msg::User;

  MotionTcpInterfaceBase * tcp_if_;
  mutable std::mutex mutex_tcp_if_;
  inline static constexpr std::chrono::milliseconds MOTION_RESPONSE_TIMEOUT{1000};

  void execute(const std::string &, std::chrono::nanoseconds) const;

public:
  MotionCommander() = delete;
  explicit MotionCommander(MotionTcpInterfaceBase *);

  // Provisional MG400 Servo command format. These APIs use the same SI-unit
  // convention and port 30003 connection as the established motion commands.
  void servoJ(const si_rad, const si_rad, const si_rad, const si_rad);

  // DOBOT MG400 Official Command ---------------------------------------------
  void movJ(
    const si_m, const si_m, const si_m, const si_rad,
    const int8_t = -1, const int8_t = -1, const int8_t = -1);

  void movL(
    const si_m, const si_m, const si_m, const si_rad,
    const int8_t = -1, const int8_t = -1, const int8_t = -1);

  void jointMovJ(
    const si_rad, const si_rad, const si_rad, const si_rad,
    const int8_t = -1, const int8_t = -1, const int8_t = -1);

  void movLIO(
    const si_m, const si_m, const si_m, const si_rad,
    const DistanceMode &, const int &,
    const DOIndex &, const DOStatus &,
    const int8_t = -1, const int8_t = -1, const int8_t = -1);

  void movLIO(
    const si_m, const si_m, const si_m, const si_rad,
    const DistanceMode::_mode_type &, const int &,
    const DOIndex::_index_type &,
    const DOStatus::_status_type &,
    const int8_t = -1, const int8_t = -1, const int8_t = -1);

  void movJIO(
    const si_m, const si_m, const si_m, const si_rad,
    const DistanceMode &, const int &,
    const DOIndex &, const DOStatus &,
    const int8_t = -1, const int8_t = -1, const int8_t = -1);

  void movJIO(
    const si_m, const si_m, const si_m, const si_rad,
    const DistanceMode::_mode_type &, const int &,
    const DOIndex::_index_type &,
    const DOStatus::_status_type &,
    const int8_t = -1, const int8_t = -1, const int8_t = -1);

/* https://github.com/Dobot-Arm/TCP-IP-CR-Python/issues/4#:~:text=The%20arc%20function%20needs%20to%20be%20fixed%20by%20Dobot
  void arc(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad,
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad);
*/

  void moveJog(const MoveJog::SharedPtr &);
  void moveJog(const MoveJog::_jog_mode_type &);

  void sync();

  void relMovJUser(
    const si_m, const si_m, const si_m, const si_rad,
    const User &,
    const int8_t = -1, const int8_t = -1, const int8_t = -1);
  void relMovJUser(
    const si_m, const si_m, const si_m, const si_rad,
    const User::_user_type &,
    const int8_t = -1, const int8_t = -1, const int8_t = -1);

  void relMovLUser(
    const si_m, const si_m, const si_m, const si_rad,
    const User &,
    const int8_t = -1, const int8_t = -1, const int8_t = -1);
  void relMovLUser(
    const si_m, const si_m, const si_m, const si_rad,
    const User::_user_type &,
    const int8_t = -1, const int8_t = -1, const int8_t = -1);

  void relJointMovJ(
    const si_rad, const si_rad, const si_rad, const si_rad,
    const int8_t = -1, const int8_t = -1, const int8_t = -1);

  // End DOBOT MG400 Official Command -----------------------------------------
};
}  // namespace mg400_interface
#endif
