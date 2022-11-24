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

#include <mg400_msgs/msg/move_jog.hpp>

#include "mg400_interface/command_utils.hpp"
#include "mg400_interface/tcp_interface/motion_tcp_interface.hpp"

namespace mg400_interface
{
class MotionCommander
{
public:
  using SharedPtr = std::shared_ptr<MotionCommander>;

private:
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
    const si_rad, const si_rad, const si_rad);

  void jointMovJ(
    const si_rad, const si_rad, const si_rad,
    const si_rad, const si_rad, const si_rad);

  void movLIO(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad,
    const DistanceMode &, const int, const DOIndex &, const DOStatus &);
  void movLIO(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad,
    const int, const int, const int, const int);

  void movJIO(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad,
    const DistanceMode &, const int, const DOIndex &, const DOStatus &);
  void movJIO(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad,
    const int, const int, const int, const int);

/* https://github.com/Dobot-Arm/TCP-IP-CR-Python/issues/4#:~:text=The%20arc%20function%20needs%20to%20be%20fixed%20by%20Dobot
  void arc(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad,
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad);
*/
  void moveJog(const mg400_msgs::msg::MoveJog::SharedPtr &);
  void moveJog(const std::string &);

  void sync();

  void relMovJUser(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad,
    const UserIndex &);
  void relMovJUser(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad,
    const int);

  void relMovLUser(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad,
    const UserIndex &);
  void relMovLUser(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad,
    const int);

  void relJointMovJ(
    const si_rad, const si_rad, const si_rad,
    const si_rad, const si_rad, const si_rad);


  // End DOBOT MG400 Official Command -----------------------------------------
};
}  // namespace mg400_interface
