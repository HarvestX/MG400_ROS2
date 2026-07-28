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
#include <type_traits>

namespace mg400_interface
{

/// Wire representation of one MG400 port-30004 feedback packet.
///
/// This layout follows the Dobot four-axis TCP/IP protocol revision 2024-04-19.
/// Values remain in Dobot controller units and are decoded from little-endian
/// wire data by directly receiving into this packed structure.
#pragma pack(push, 1)
struct RealTimeData
{
  static constexpr std::uint16_t EXPECTED_MESSAGE_SIZE = 1440;
  static constexpr std::uint64_t EXPECTED_TEST_VALUE = UINT64_C(0x0123456789ABCDEF);

  std::uint16_t message_size;
  std::uint16_t reserved_0002_0007[3];
  std::uint64_t digital_inputs;
  std::uint64_t digital_outputs;
  std::uint64_t robot_mode;
  std::uint64_t controller_timestamp;
  std::uint64_t reserved_0040_0047;
  std::uint64_t test_value;
  double reserved_0056_0063;
  double speed_scaling;
  double reserved_0072_0079;
  double v_main;
  double v_robot;
  double i_robot;
  double reserved_0104_0111;
  double reserved_0112_0119;
  double reserved_0120_0143[3];
  double reserved_0144_0167[3];
  double reserved_0168_0191[3];
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
  std::int8_t hand_type[4];
  std::uint8_t user_index;
  std::uint8_t tool_index;
  std::uint8_t run_queued_cmd;
  std::uint8_t pause_cmd_flag;
  std::uint8_t velocity_ratio;
  std::uint8_t acceleration_ratio;
  std::uint8_t jerk_ratio;
  std::uint8_t xyz_velocity_ratio;
  std::uint8_t r_velocity_ratio;
  std::uint8_t xyz_acceleration_ratio;
  std::uint8_t r_acceleration_ratio;
  std::uint8_t xyz_jerk_ratio;
  std::uint8_t r_jerk_ratio;
  std::uint8_t brake_status;
  std::uint8_t enable_status;
  std::uint8_t drag_status;
  std::uint8_t running_status;
  std::uint8_t error_status;
  std::uint8_t jog_status;
  std::uint8_t robot_type;
  std::uint8_t drag_button_signal;
  std::uint8_t enable_button_signal;
  std::uint8_t record_button_signal;
  std::uint8_t reappear_button_signal;
  std::uint8_t jaw_button_signal;
  std::uint8_t six_force_online;
  std::uint8_t reserved_1038_1119[82];
  double m_actual[6];
  double load;
  double center_x;
  double center_y;
  double center_z;
  double user[6];
  double tool[6];
  double trace_index;
  double six_force_value[6];
  double target_quaternion[4];
  double actual_quaternion[4];
  std::uint8_t reserved_1416_1439[24];

  bool isValid() const noexcept
  {
    return message_size == EXPECTED_MESSAGE_SIZE && test_value == EXPECTED_TEST_VALUE;
  }
};
#pragma pack(pop)

static_assert(std::is_standard_layout<RealTimeData>::value, "RealTimeData must be standard-layout");
static_assert(sizeof(RealTimeData) == 1440, "RealTimeData must match the wire packet size");
static_assert(offsetof(RealTimeData, digital_inputs) == 8, "Unexpected DigitalInputs offset");
static_assert(offsetof(RealTimeData, robot_mode) == 24, "Unexpected RobotMode offset");
static_assert(
  offsetof(RealTimeData, controller_timestamp) == 32, "Unexpected TimeStamp offset");
static_assert(offsetof(RealTimeData, test_value) == 48, "Unexpected TestValue offset");
static_assert(offsetof(RealTimeData, speed_scaling) == 64, "Unexpected SpeedScaling offset");
static_assert(offsetof(RealTimeData, v_main) == 80, "Unexpected VMain offset");
static_assert(offsetof(RealTimeData, q_target) == 192, "Unexpected QTarget offset");
static_assert(offsetof(RealTimeData, q_actual) == 432, "Unexpected QActual offset");
static_assert(
  offsetof(RealTimeData, tool_vector_actual) == 624, "Unexpected ToolVectorActual offset");
static_assert(
  offsetof(RealTimeData, motor_temperatures) == 864, "Unexpected MotorTemperatures offset");
static_assert(offsetof(RealTimeData, hand_type) == 1008, "Unexpected HandType offset");
static_assert(offsetof(RealTimeData, brake_status) == 1025, "Unexpected BrakeStatus offset");
static_assert(offsetof(RealTimeData, robot_type) == 1031, "Unexpected RobotType offset");
static_assert(offsetof(RealTimeData, m_actual) == 1120, "Unexpected MActual offset");
static_assert(offsetof(RealTimeData, load) == 1168, "Unexpected Load offset");
static_assert(offsetof(RealTimeData, user) == 1200, "Unexpected User offset");
static_assert(offsetof(RealTimeData, trace_index) == 1296, "Unexpected TraceIndex offset");
static_assert(
  offsetof(RealTimeData, target_quaternion) == 1352, "Unexpected TargetQuaternion offset");
static_assert(
  offsetof(RealTimeData, actual_quaternion) == 1384, "Unexpected ActualQuaternion offset");

}  // namespace mg400_interface
#endif
