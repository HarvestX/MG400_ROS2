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

#include "mg400_interface/command_utils/move_jog.hpp"
#include <stdexcept>

namespace mg400_interface
{

std::string getAxisIdStr(const JogMode axis_id)
{
  switch (axis_id) {
    case JogMode::J1_NEGATIVE:
      return "j1-";
    case JogMode::J1_POSITIVE:
      return "j1+";
    case JogMode::J2_NEGATIVE:
      return "j2-";
    case JogMode::J2_POSITIVE:
      return "j2+";
    case JogMode::J3_NEGATIVE:
      return "j3-";
    case JogMode::J3_POSITIVE:
      return "j3+";
    case JogMode::J4_NEGATIVE:
      return "j4-";
    case JogMode::J4_POSITIVE:
      return "j4+";

    case JogMode::X_NEGATIVE:
      return "X-";
    case JogMode::X_POSITIVE:
      return "X+";
    case JogMode::Y_NEGATIVE:
      return "Y-";
    case JogMode::Y_POSITIVE:
      return "Y+";
    case JogMode::Z_NEGATIVE:
      return "Z-";
    case JogMode::Z_POSITIVE:
      return "Z+";

    case JogMode::RX_NEGATIVE:
      return "Rx-";
    case JogMode::RX_POSITIVE:
      return "Rx+";
    case JogMode::RY_NEGATIVE:
      return "Ry-";
    case JogMode::RY_POSITIVE:
      return "Ry+";
    case JogMode::RZ_NEGATIVE:
      return "Rz-";
    case JogMode::RZ_POSITIVE:
      return "Rz+";

    case JogMode::STOP:
      return "";
    case JogMode::INVALID:
    default:
      break;
  }
  throw std::runtime_error("Invalid axis_id given.");
}
}  // namespace mg400_interface
