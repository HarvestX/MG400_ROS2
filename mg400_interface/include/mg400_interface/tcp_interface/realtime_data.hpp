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

#ifndef __MG400_INTERFACE_TCP_INTERFACE_REALTIME_DATA_HPP__
#define __MG400_INTERFACE_TCP_INTERFACE_REALTIME_DATA_HPP__

#include <inttypes.h>
#include <cstddef>

namespace mg400_interface
{
#pragma pack(push, 1)
struct RealTimeData
{
  uint16_t len;
  uint16_t not_used_01[3];
  uint64_t digital_inputs;
  uint64_t digital_outputs;
  uint64_t robot_mode;
  uint64_t time_stamp;
  uint64_t not_used_02;
  uint64_t test_value;
  double not_used_03;
  double speed_scaling;
  double not_used_04;
  double not_used_05;
  double not_used_06;
  double not_used_07;
  double not_used_08;
  double not_used_09;
  double not_used_10[3];
  double not_used_11[3];
  double not_used_12[3];
  double q_target[6];
  double qd_target[6];
  double qdd_target[6];
  double i_target[6];
  double m_target[6];
  double q_actual[6];
  double qd_actual[6];
  double i_actual[6];
  double actual_i_TCP_force[6];
  double tool_vector_actual[6];
  double TCP_speed_actual[6];
  double TCP_force[6];
  double tool_vector_target[6];
  double TCP_speed_target[6];
  double not_used_13[6];
  double not_used_14[6];
  double not_used_15[6];
  uint8_t not_used_16[17];
  uint8_t brake_status;
  uint8_t enable_status;
  uint8_t drag_status;
  uint8_t running_status;
  uint8_t error_status;
  uint8_t not_used_17[90];
  double not_used_18[6];
  double load;
  double center_x;
  double center_y;
  double center_z;
  double not_used_19[6];
  double not_used_20[6];
  double not_used_21;
  double not_used_22[6];
  double not_used_23[4];
  double not_used_24[4];
  uint8_t not_used_25[24];
};
#pragma pack(pop)

static_assert(sizeof(RealTimeData) == 1440, "Unexpected feedback packet size");
static_assert(offsetof(RealTimeData, time_stamp) == 32, "Unexpected timestamp offset");
static_assert(offsetof(RealTimeData, brake_status) == 1025, "Unexpected brake status offset");
static_assert(offsetof(RealTimeData, enable_status) == 1026, "Unexpected enable status offset");
static_assert(offsetof(RealTimeData, drag_status) == 1027, "Unexpected drag status offset");
static_assert(offsetof(RealTimeData, running_status) == 1028, "Unexpected running status offset");
static_assert(offsetof(RealTimeData, error_status) == 1029, "Unexpected error status offset");
static_assert(offsetof(RealTimeData, load) == 1168, "Unexpected load offset");

}  // namespace mg400_interface
#endif
