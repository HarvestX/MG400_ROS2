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

#include "mg400_interface/commander/realtime_commander.hpp"

namespace mg400_interface
{
RealtimeCommander::RealtimeCommander(TcpInterfaceBase * tcp_if)
: tcp_if_(tcp_if)
{
}

void RealtimeCommander::test()
{
  this->tcp_if_->sendCommand("Hello");
}

// DOBOT MG400 Official Command ---------------------------------------------
void RealtimeCommander::movJ(
  const si_m x, const si_m y, const si_m z,
  const si_rad rx, const si_rad ry, const si_rad rz)
{
  char buf[100];
  snprintf(
    buf, sizeof(buf),
    "MovJ(%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf)",
    m2mm(x), m2mm(y), m2mm(z),
    rad2degree(rx), rad2degree(ry), rad2degree(rz));
  this->tcp_if_->sendCommand(buf);
}

void RealtimeCommander::movL(
  const si_m x, const si_m y, const si_m z,
  const si_rad rx, const si_rad ry, const si_rad rz
)
{
  // TODO(anyone): Implement it
}

void RealtimeCommander::jointMovJ(
  const si_rad j1, const si_rad j2, const si_rad j3,
  const si_rad j4, const si_rad j5, const si_rad j6)
{
  // TODO(anyone): Implement it
}

void RealtimeCommander::movLIO(
  const si_m x, const si_m y, const si_m z,
  const si_rad rx, const si_rad ry, const si_rad rz,
  const DistanceMode & mode, const int distance,
  const DOIndex & index, const DOStatus & status)
{
  // TODO(anyone): Implement it
}

void RealtimeCommander::movJIO(
  const si_m x, const si_m y, const si_m z,
  const si_rad rx, const si_rad ry, const si_rad rz,
  const DistanceMode & mode, const int distance,
  const DOIndex & index, const DOStatus & status)
{

  // TODO(anyone): Implement it
}

void RealtimeCommander::arc(
  const si_m x1, const si_m y1, const si_m z1,
  const si_rad rx1, const si_rad ry1, const si_rad rz1,
  const si_m x2, const si_m y2, const si_m z2,
  const si_rad rx2, const si_rad ry2, const si_rad rz2)
{

  // TODO(anyone): Implement it
}

void RealtimeCommander::moveJog(const JogMode & axis_id)
{
  char buf[100];
  snprintf(buf, sizeof(buf), "MoveJog(%s)", getAxisIdStr(axis_id).c_str());
  this->tcp_if_->sendCommand(buf);
}

void RealtimeCommander::sync()
{
  // TODO(anyone): Implement it
}

void RealtimeCommander::relMovJUser(
  const si_m, const si_m, const si_m,
  const si_rad, const si_rad, const si_rad,
  const UserIndex &)
{
  // TODO(anyone): Implement it
}

void RealtimeCommander::relMovLUser(
  const si_m, const si_m, const si_m,
  const si_rad, const si_rad, const si_rad,
  const UserIndex &)
{
  // TODO(anyone): Implement it
}

void RealtimeCommander::relJointMovJ(
  const si_rad, const si_rad, const si_rad,
  const si_rad, const si_rad, const si_rad)
{
  // TODO(anyone): Implement it
}
// End DOBOT MG400 Official Command -----------------------------------------

}  // namespace mg400_interface
