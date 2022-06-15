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

#include "mg400_interface/command_utils.hpp"
#include "mg400_interface/tcp_interface/tcp_interface_base.hpp"

namespace mg400_interface
{
class RealtimeCommander
{
private:
  TcpInterfaceBase * tcp_if_;

public:
  RealtimeCommander() = delete;
  explicit RealtimeCommander(TcpInterfaceBase *);

  void test();

  // DOBOT MG400 Official Command ---------------------------------------------
  void movJ(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad);

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

  void movJIO(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad,
    const DistanceMode &, const int, const DOIndex &, const DOStatus &);


  void arc(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad,
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad);

  void moveJog(const JogMode &);

  void sync();

  void relMovJUser(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad,
    const UserIndex &);

  void relMovLUser(
    const si_m, const si_m, const si_m,
    const si_rad, const si_rad, const si_rad,
    const UserIndex &);

  void relJointMovJ(
    const si_rad, const si_rad, const si_rad,
    const si_rad, const si_rad, const si_rad);

  // End DOBOT MG400 Official Command -----------------------------------------
};
}  // namespace mg400_interface
