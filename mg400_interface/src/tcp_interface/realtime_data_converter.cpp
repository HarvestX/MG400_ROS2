// Copyright 2026 HarvestX Inc.
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

#include "mg400_interface/tcp_interface/realtime_data_converter.hpp"

#include <algorithm>
#include <iterator>

namespace mg400_interface
{

constexpr std::uint16_t RealTimeData::EXPECTED_MESSAGE_SIZE;
constexpr std::uint64_t RealTimeData::EXPECTED_TEST_VALUE;

namespace
{

template<typename SourceT, typename DestinationT>
void copyArray(const SourceT & source, DestinationT & destination)
{
  std::copy(std::begin(source), std::end(source), destination.begin());
}

}  // namespace

mg400_msgs::msg::RealtimeFeedback toRealtimeFeedbackMessage(const RealTimeData & data)
{
  mg400_msgs::msg::RealtimeFeedback message;

  message.digital_inputs = data.digital_inputs;
  message.digital_outputs = data.digital_outputs;
  message.robot_mode = data.robot_mode;
  message.controller_timestamp = data.controller_timestamp;
  message.speed_scaling = data.speed_scaling;
  message.v_main = data.v_main;
  message.v_robot = data.v_robot;
  message.i_robot = data.i_robot;

  copyArray(data.q_target, message.q_target);
  copyArray(data.qd_target, message.qd_target);
  copyArray(data.qdd_target, message.qdd_target);
  copyArray(data.i_target, message.i_target);
  copyArray(data.m_target, message.m_target);
  copyArray(data.q_actual, message.q_actual);
  copyArray(data.qd_actual, message.qd_actual);
  copyArray(data.i_actual, message.i_actual);
  copyArray(data.actual_tcp_force, message.actual_tcp_force);
  copyArray(data.tool_vector_actual, message.tool_vector_actual);
  copyArray(data.tcp_speed_actual, message.tcp_speed_actual);
  copyArray(data.tcp_force, message.tcp_force);
  copyArray(data.tool_vector_target, message.tool_vector_target);
  copyArray(data.tcp_speed_target, message.tcp_speed_target);
  copyArray(data.motor_temperatures, message.motor_temperatures);
  copyArray(data.joint_modes, message.joint_modes);
  copyArray(data.v_actual, message.v_actual);

  copyArray(data.hand_type, message.hand_type);
  message.user_index = data.user_index;
  message.tool_index = data.tool_index;
  message.run_queued_cmd = data.run_queued_cmd;
  message.pause_cmd_flag = data.pause_cmd_flag;
  message.velocity_ratio = data.velocity_ratio;
  message.acceleration_ratio = data.acceleration_ratio;
  message.jerk_ratio = data.jerk_ratio;
  message.xyz_velocity_ratio = data.xyz_velocity_ratio;
  message.r_velocity_ratio = data.r_velocity_ratio;
  message.xyz_acceleration_ratio = data.xyz_acceleration_ratio;
  message.r_acceleration_ratio = data.r_acceleration_ratio;
  message.xyz_jerk_ratio = data.xyz_jerk_ratio;
  message.r_jerk_ratio = data.r_jerk_ratio;
  message.brake_status = data.brake_status;
  message.enable_status = data.enable_status;
  message.drag_status = data.drag_status;
  message.running_status = data.running_status;
  message.error_status = data.error_status;
  message.jog_status = data.jog_status;
  message.robot_type = data.robot_type;
  message.drag_button_signal = data.drag_button_signal;
  message.enable_button_signal = data.enable_button_signal;
  message.record_button_signal = data.record_button_signal;
  message.reappear_button_signal = data.reappear_button_signal;
  message.jaw_button_signal = data.jaw_button_signal;
  message.six_force_online = data.six_force_online;

  copyArray(data.m_actual, message.m_actual);
  message.load = data.load;
  message.center_x = data.center_x;
  message.center_y = data.center_y;
  message.center_z = data.center_z;
  copyArray(data.user, message.user);
  copyArray(data.tool, message.tool);
  message.trace_index = data.trace_index;
  copyArray(data.six_force_value, message.six_force_value);
  copyArray(data.target_quaternion, message.target_quaternion);
  copyArray(data.actual_quaternion, message.actual_quaternion);

  return message;
}

}  // namespace mg400_interface
