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

#include "mg400_interface/commander/motion_commander.hpp"

namespace mg400_interface
{
MotionCommander::MotionCommander(MotionTcpInterfaceBase * tcp_if)
: tcp_if_(tcp_if)
{
}

// DOBOT MG400 Official Command ---------------------------------------------
void MotionCommander::movJ(
  const si_m x, const si_m y, const si_m z,
  const si_rad rx, const si_rad ry, const si_rad rz)
{
  char buf[100];
  snprintf(
    buf, sizeof(buf),
    "MovJ(%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf)",
    m2mm(x), m2mm(y), m2mm(z),
    rad2degree(rx), rad2degree(ry), rad2degree(rz));
  this->tcp_if_->sendCommand(buf);
}

void MotionCommander::movL(
  const si_m x, const si_m y, const si_m z,
  const si_rad rx, const si_rad ry, const si_rad rz)
{
  char buf[100];
  snprintf(
    buf, sizeof(buf),
    "MovL(%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf)",
    m2mm(x), m2mm(y), m2mm(z),
    rad2degree(rx), rad2degree(ry), rad2degree(rz));
  this->tcp_if_->sendCommand(buf);
}

void MotionCommander::jointMovJ(
  const si_rad j1, const si_rad j2, const si_rad j3,
  const si_rad j4, const si_rad j5, const si_rad j6)
{
  char buf[100];
  snprintf(
    buf, sizeof(buf),
    "JointMovJ(%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf)",
    rad2degree(j1), rad2degree(j2), rad2degree(j3),
    rad2degree(j4), rad2degree(j5), rad2degree(j6));
  this->tcp_if_->sendCommand(buf);
}

void MotionCommander::movLIO(
  const si_m x, const si_m y, const si_m z,
  const si_rad rx, const si_rad ry, const si_rad rz,
  const DistanceMode & mode, const int & distance,
  const DOIndex & index, const DOStatus & status)
{
  this->movLIO(
    x, y, z, rx, ry, rz,
    mode.mode, distance,
    index.index, status.status);
}

void MotionCommander::movLIO(
  const si_m x, const si_m y, const si_m z,
  const si_rad rx, const si_rad ry, const si_rad rz,
  const DistanceMode::_mode_type & mode,
  const int & distance,
  const DOIndex::_index_type & index,
  const DOStatus::_status_type & status)
{
  char buf[100];
  snprintf(
    buf, sizeof(buf),
    "MovLIO(%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,{%u,%d,%u,%u})",
    m2mm(x), m2mm(y), m2mm(z),
    rad2degree(rx), rad2degree(ry), rad2degree(rz),
    mode, distance, index, status);
  this->tcp_if_->sendCommand(buf);
}

void MotionCommander::movJIO(
  const si_m x, const si_m y, const si_m z,
  const si_rad rx, const si_rad ry, const si_rad rz,
  const DistanceMode & mode, const int & distance,
  const DOIndex & index, const DOStatus & status)
{
  this->movJIO(
    x, y, z, rx, ry, rz,
    mode.mode, distance, index.index, status.status);
}

void MotionCommander::movJIO(
  const si_m x, const si_m y, const si_m z,
  const si_rad rx, const si_rad ry, const si_rad rz,
  const DistanceMode::_mode_type & mode, const int & distance,
  const DOIndex::_index_type & index,
  const DOStatus::_status_type & status)
{
  char buf[100];
  snprintf(
    buf, sizeof(buf),
    "MovJIO(%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,{%u,%d,%u,%u})",
    m2mm(x), m2mm(y), m2mm(z),
    rad2degree(rx), rad2degree(ry), rad2degree(rz),
    mode, distance, index, status);
  this->tcp_if_->sendCommand(buf);
}

/* https://github.com/Dobot-Arm/TCP-IP-CR-Python/issues/4#:~:text=The%20arc%20function%20needs%20to%20be%20fixed%20by%20Dobot
void MotionCommander::arc(
  const si_m x1, const si_m y1, const si_m z1,
  const si_rad rx1, const si_rad ry1, const si_rad rz1,
  const si_m x2, const si_m y2, const si_m z2,
  const si_rad rx2, const si_rad ry2, const si_rad rz2)
{
  char buf[100];
  snprintf(
    buf, sizeof(buf),
    "Arc(%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf)",
    m2mm(x1), m2mm(y1), m2mm(z1),
    rad2degree(rx1), rad2degree(ry1), rad2degree(rz1),
    m2mm(x2), m2mm(y2), m2mm(z2),
    rad2degree(rx2), rad2degree(ry2), rad2degree(rz2));
  this->tcp_if_->sendCommand(buf);
}
*/

void MotionCommander::moveJog(const MoveJog::SharedPtr & jog_mode)
{
  this->moveJog(jog_mode->jog_mode);
}

void MotionCommander::moveJog(const MoveJog::_jog_mode_type & jog_mode)
{
  char buf[100];
  snprintf(buf, sizeof(buf), "MoveJog(%s)", jog_mode.c_str());
  this->tcp_if_->sendCommand(buf);
}


void MotionCommander::sync()
{
  char buf[100];
  snprintf(buf, sizeof(buf), "Sync()");
  this->tcp_if_->sendCommand(buf);
}

void MotionCommander::relMovJUser(
  const si_m x, const si_m y, const si_m z,
  const si_rad rx, const si_rad ry, const si_rad rz,
  const User & user)
{
  this->relMovJUser(
    x, y, z, rx, ry, rz,
    user.user);
}

void MotionCommander::relMovJUser(
  const si_m x, const si_m y, const si_m z,
  const si_rad rx, const si_rad ry, const si_rad rz,
  const User::_user_type & user)
{
  char buf[100];
  snprintf(
    buf, sizeof(buf),
    "RelMovJUser(%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%u)",
    m2mm(x), m2mm(y), m2mm(z),
    rad2degree(rx), rad2degree(ry), rad2degree(rz), user);
  this->tcp_if_->sendCommand(buf);
}

void MotionCommander::relMovLUser(
  const si_m x, const si_m y, const si_m z,
  const si_rad rx, const si_rad ry, const si_rad rz,
  const User & user)
{
  this->relMovLUser(
    x, y, z, rx, ry, rz,
    user.user);
}

void MotionCommander::relMovLUser(
  const si_m x, const si_m y, const si_m z,
  const si_rad rx, const si_rad ry, const si_rad rz,
  const User::_user_type & user)
{
  char buf[100];
  snprintf(
    buf, sizeof(buf),
    "RelMovLUser(%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%u)",
    m2mm(x), m2mm(y), m2mm(z),
    rad2degree(rx), rad2degree(ry), rad2degree(rz), user);
  this->tcp_if_->sendCommand(buf);
}

void MotionCommander::relJointMovJ(
  const si_rad j1, const si_rad j2, const si_rad j3,
  const si_rad j4, const si_rad j5, const si_rad j6)
{
  char buf[100];
  snprintf(
    buf, sizeof(buf),
    "RelJointMovJ(%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf)",
    rad2degree(j1), rad2degree(j2), rad2degree(j3),
    rad2degree(j4), rad2degree(j5), rad2degree(j6));
  this->tcp_if_->sendCommand(buf);
}

// End DOBOT MG400 Official Command -----------------------------------------

}  // namespace mg400_interface
