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

#include <memory>
#include <string>

#include <mg400_msgs/msg/distance_mode.hpp>
#include <mg400_msgs/msg/do_index.hpp>
#include <mg400_msgs/msg/do_status.hpp>
#include <mg400_msgs/msg/move_jog.hpp>
#include <mg400_msgs/msg/tool_do_index.hpp>
#include <mg400_msgs/msg/user.hpp>

#include "mg400_interface/command_utils.hpp"
#include "mg400_interface/tcp_interface/motion_tcp_interface.hpp"

namespace mg400_interface
{
class MotionCommander
{
public:
  using SharedPtr = std::shared_ptr<MotionCommander>;

private:
  using DistanceMode = mg400_msgs::msg::DistanceMode;
  using DOIndex = mg400_msgs::msg::DOIndex;
  using DOStatus = mg400_msgs::msg::DOStatus;
  using MoveJog = mg400_msgs::msg::MoveJog;
  using User = mg400_msgs::msg::User;

  MotionTcpInterfaceBase * tcp_if_;

public:
  MotionCommander() = delete;
  explicit MotionCommander(MotionTcpInterfaceBase *);

  // DOBOT MG400 Official Command ---------------------------------------------
  void movJ(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad = 0.0, const si_rad = 0.0);

  void movL(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad = 0.0, const si_rad = 0.0);

  void jointMovJ(
    const si_rad, const si_rad, const si_rad,
    const si_rad, const si_rad, const si_rad);

  void movLIO(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad,
    const DistanceMode &, const int &,
    const DOIndex &, const DOStatus &);
  void movLIO(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad,
    const DistanceMode::_mode_type &, const int &,
    const DOIndex::_index_type &,
    const DOStatus::_status_type &);

  void movJIO(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad,
    const DistanceMode &, const int &,
    const DOIndex &, const DOStatus &);
  void movJIO(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad,
    const DistanceMode::_mode_type &, const int &,
    const DOIndex::_index_type &,
    const DOStatus::_status_type &);

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
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad,
    const User &);
  void relMovJUser(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad,
    const User::_user_type &);

  void relMovLUser(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad,
    const User &);
  void relMovLUser(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad,
    const User::_user_type &);

  void relJointMovJ(
    const si_rad, const si_rad, const si_rad,
    const si_rad, const si_rad, const si_rad);


  // End DOBOT MG400 Official Command -----------------------------------------
};
}  // namespace mg400_interface
