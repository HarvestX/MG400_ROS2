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

#include <cmath>

#include "mg400_interface/command_utils/arch.hpp"
#include "mg400_interface/command_utils/collision_level.hpp"
#include "mg400_interface/command_utils/digital_out.hpp"
#include "mg400_interface/command_utils/distance_mode.hpp"
#include "mg400_interface/command_utils/move_jog.hpp"
#include "mg400_interface/command_utils/tool.hpp"
#include "mg400_interface/command_utils/user.hpp"

namespace mg400_interface
{

using si_mm = double;
using si_m = double;
using si_rad = double;
using si_degree = double;


constexpr const double TO_RADIAN = M_PI / 180.0;
constexpr const double TO_DEGREE = 180.0 / M_PI;

constexpr const double TO_M = 1e-3;
constexpr const double TO_MM = 1e3;

inline si_rad degree2rad(const si_degree val)
{
  return val * TO_RADIAN;
}

inline si_degree rad2degree(const si_rad val)
{
  return val * TO_DEGREE;
}

inline si_mm m2mm(const si_m val)
{
  return val * TO_MM;
}

inline si_m mm2m(const si_mm val)
{
  return val * TO_M;
}

}  // namespace mg400_interface
