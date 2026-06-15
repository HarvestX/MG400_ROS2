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

#include <cstddef>
#include <cstdint>

namespace mg400_interface
{
#pragma pack(push, 1)
struct RealTimeData
{
  uint16_t message_size;
  uint8_t reserved_0002[6];
  uint64_t digital_inputs;
  uint64_t digital_outputs;
  uint64_t robot_mode;
  uint64_t time_stamp;
  uint64_t run_time;
  uint64_t test_value;
  uint8_t reserved_0056[8];
  double speed_scaling;
  uint8_t reserved_0072[16];
  double v_robot;
  double i_robot;
  double program_state;
  uint8_t safety_io_in[2];
  uint8_t safety_io_out[2];
  uint8_t reserved_0116[76];
  double q_target[6];
  double qd_target[6];
  double qdd_target[6];
  double i_target[6];
  double m_target[6];
  double q_actual[6];
  double qd_actual[6];
  double i_actual[6];
  double actual_tcp_force[6];
  double tool_vector_actual[6];
  double tcp_speed_actual[6];
  double tcp_force[6];
  double tool_vector_target[6];
  double tcp_speed_target[6];
  double motor_temperatures[6];
  double joint_modes[6];
  double v_actual[6];
  uint8_t hand_type[4];
  uint8_t user;
  uint8_t tool;
  uint8_t run_queued_cmd;
  uint8_t pause_cmd_flag;
  uint8_t velocity_ratio;
  uint8_t acceleration_ratio;
  uint8_t reserved_1018;
  uint8_t xyz_velocity_ratio;
  uint8_t r_velocity_ratio;
  uint8_t xyz_acceleration_ratio;
  uint8_t r_acceleration_ratio;
  uint8_t reserved_1023[2];
  uint8_t brake_status;
  uint8_t enable_status;
  uint8_t drag_status;
  uint8_t running_status;
  uint8_t error_status;
  uint8_t jog_status_cr;
  uint8_t cr_robot_type;
  uint8_t drag_button_signal;
  uint8_t enable_button_signal;
  uint8_t record_button_signal;
  uint8_t reappear_button_signal;
  uint8_t jaw_button_signal;
  uint8_t six_force_online;
  uint8_t collision_state;
  uint8_t arm_approach_state;
  uint8_t j4_approach_state;
  uint8_t j5_approach_state;
  uint8_t j6_approach_state;
  uint8_t reserved_1043[61];
  double vibration_dis_z;
  uint64_t current_command_id;
  double m_actual[6];
  double load;
  double center_x;
  double center_y;
  double center_z;
  double user_coordinates[6];
  double tool_coordinates[6];
  uint8_t reserved_1296[8];
  double six_force_value[6];
  double target_quaternion[4];
  double actual_quaternion[4];
  uint16_t auto_manual_mode;
  uint16_t export_status;
  uint8_t safety_state;
  uint8_t safe_state;
  uint8_t reserved_1422[18];
};
#pragma pack(pop)

static_assert(sizeof(RealTimeData) == 1440, "RealTimeData must match the 1440-byte packet");
static_assert(offsetof(RealTimeData, digital_inputs) == 8, "Unexpected DigitalInputs offset");
static_assert(offsetof(RealTimeData, q_target) == 192, "Unexpected QTarget offset");
static_assert(offsetof(RealTimeData, hand_type) == 1008, "Unexpected HandType offset");
static_assert(offsetof(RealTimeData, vibration_dis_z) == 1104, "Unexpected VibrationDisZ offset");
static_assert(offsetof(RealTimeData, six_force_value) == 1304, "Unexpected SixForceValue offset");
static_assert(offsetof(RealTimeData, safety_state) == 1420, "Unexpected SafetyState offset");

}  // namespace mg400_interface
#endif
