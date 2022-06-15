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
  const double x, const double y, const double z,
  const double rx, const double ry, const double rz)
{
  char buf[100];
  snprintf(
    buf, sizeof(buf),
    "MovJ(%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf)",
    x, y, z, rx, ry, rz);
  this->tcp_if_->sendCommand(buf);
}

void RealtimeCommander::moveJog(const std::string & axis_id)
{
  char buf[100];
  snprintf(buf, sizeof(buf), "MoveJog(%s)", axis_id.c_str());
  this->tcp_if_->sendCommand(buf);
}
// End DOBOT MG400 Official Command -----------------------------------------

}  // namespace mg400_interface
