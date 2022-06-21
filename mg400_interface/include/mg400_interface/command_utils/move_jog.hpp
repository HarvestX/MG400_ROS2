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

#include <string>

namespace mg400_interface
{
enum class JogMode
{
  J1_NEGATIVE,
  J1_POSITIVE,
  J2_NEGATIVE,
  J2_POSITIVE,
  J3_NEGATIVE,
  J3_POSITIVE,
  J4_NEGATIVE,
  J4_POSITIVE,

  X_NEGATIVE,
  X_POSITIVE,
  Y_NEGATIVE,
  Y_POSITIVE,
  Z_NEGATIVE,
  Z_POSITIVE,

  RX_NEGATIVE,
  RX_POSITIVE,
  RY_NEGATIVE,
  RY_POSITIVE,
  RZ_NEGATIVE,
  RZ_POSITIVE,

  STOP,
  INVALID
};

std::string getAxisIdStr(const JogMode axis_id);
}  // namespace mg400_interface
