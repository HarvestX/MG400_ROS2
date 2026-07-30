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

#include <chrono>
#include <cstddef>

#include <gtest/gtest.h>

#include "mg400_interface/command_utils.hpp"
#include "mg400_interface/tcp_interface/realtime_data.hpp"
#include "mg400_interface/tcp_interface/realtime_data_converter.hpp"
#include "mg400_interface/tcp_interface/realtime_data_snapshot.hpp"

namespace
{

using mg400_interface::RealTimeData;
using mg400_interface::RealtimeDataSnapshot;

TEST(TestRealtimeData, ConvertsActualJointAnglesToRadians)
{
  RealtimeDataSnapshot snapshot;
  snapshot.data.q_actual[0] = 0.0;
  snapshot.data.q_actual[1] = 90.0;
  snapshot.data.q_actual[2] = -180.0;
  snapshot.data.q_actual[3] = 360.0;

  const auto joints = snapshot.jointAnglesRad();

  EXPECT_DOUBLE_EQ(0.0, joints[0]);
  EXPECT_DOUBLE_EQ(90.0 * mg400_interface::TO_RADIAN, joints[1]);
  EXPECT_DOUBLE_EQ(-180.0 * mg400_interface::TO_RADIAN, joints[2]);
  EXPECT_DOUBLE_EQ(360.0 * mg400_interface::TO_RADIAN, joints[3]);
}

TEST(TestRealtimeData, SnapshotFreshnessIncludesTimeoutBoundaryAndConnectionEpoch)
{
  using namespace std::chrono_literals;  // NOLINT
  RealtimeDataSnapshot snapshot;
  snapshot.has_data = true;
  snapshot.connection_epoch = 7;
  snapshot.received_at = RealtimeDataSnapshot::Clock::now();

  EXPECT_TRUE(snapshot.isFresh(7, 100ms, snapshot.received_at + 99ms));
  EXPECT_TRUE(snapshot.isFresh(7, 100ms, snapshot.received_at + 100ms));
  EXPECT_FALSE(snapshot.isFresh(7, 100ms, snapshot.received_at + 100ms + 1ns));
  EXPECT_FALSE(snapshot.isFresh(8, 100ms, snapshot.received_at));

  snapshot.has_data = false;
  EXPECT_FALSE(snapshot.isFresh(7, 100ms, snapshot.received_at));
}

TEST(TestRealtimeData, ValidatesTransportFields)
{
  RealTimeData data{};

  EXPECT_FALSE(data.isValid());
  data.message_size = RealTimeData::EXPECTED_MESSAGE_SIZE;
  EXPECT_FALSE(data.isValid());
  data.test_value = RealTimeData::EXPECTED_TEST_VALUE;
  EXPECT_TRUE(data.isValid());

  data.message_size = 0;
  EXPECT_FALSE(data.isValid());
  data.message_size = RealTimeData::EXPECTED_MESSAGE_SIZE;
  data.test_value = 0;
  EXPECT_FALSE(data.isValid());
}

TEST(TestRealtimeData, ConvertsEveryPublishedField)
{
  RealTimeData data{};
  data.digital_inputs = 1;
  data.digital_outputs = 2;
  data.robot_mode = 3;
  data.controller_timestamp = 4;
  data.speed_scaling = 5.0;
  data.v_main = 6.0;
  data.v_robot = 7.0;
  data.i_robot = 8.0;

  for (std::size_t i = 0; i < 6; ++i) {
    const auto index = static_cast<double>(i);
    data.q_target[i] = 10.0 + index;
    data.qd_target[i] = 20.0 + index;
    data.qdd_target[i] = 30.0 + index;
    data.i_target[i] = 40.0 + index;
    data.m_target[i] = 50.0 + index;
    data.q_actual[i] = 60.0 + index;
    data.qd_actual[i] = 70.0 + index;
    data.i_actual[i] = 80.0 + index;
    data.actual_tcp_force[i] = 90.0 + index;
    data.tool_vector_actual[i] = 100.0 + index;
    data.tcp_speed_actual[i] = 110.0 + index;
    data.tcp_force[i] = 120.0 + index;
    data.tool_vector_target[i] = 130.0 + index;
    data.tcp_speed_target[i] = 140.0 + index;
    data.motor_temperatures[i] = 150.0 + index;
    data.joint_modes[i] = 160.0 + index;
    data.v_actual[i] = 170.0 + index;
    data.m_actual[i] = 180.0 + index;
    data.user[i] = 190.0 + index;
    data.tool[i] = 200.0 + index;
    data.six_force_value[i] = 210.0 + index;
  }

  data.hand_type[0] = -1;
  data.hand_type[1] = 1;
  data.hand_type[2] = -1;
  data.hand_type[3] = 1;
  data.user_index = 11;
  data.tool_index = 12;
  data.run_queued_cmd = 13;
  data.pause_cmd_flag = 14;
  data.velocity_ratio = 15;
  data.acceleration_ratio = 16;
  data.jerk_ratio = 17;
  data.xyz_velocity_ratio = 18;
  data.r_velocity_ratio = 19;
  data.xyz_acceleration_ratio = 20;
  data.r_acceleration_ratio = 21;
  data.xyz_jerk_ratio = 22;
  data.r_jerk_ratio = 23;
  data.brake_status = 24;
  data.enable_status = 25;
  data.drag_status = 26;
  data.running_status = 27;
  data.error_status = 28;
  data.jog_status = 29;
  data.robot_type = 30;
  data.drag_button_signal = 31;
  data.enable_button_signal = 32;
  data.record_button_signal = 33;
  data.reappear_button_signal = 34;
  data.jaw_button_signal = 35;
  data.six_force_online = 36;
  data.load = 220.0;
  data.center_x = 221.0;
  data.center_y = 222.0;
  data.center_z = 223.0;
  data.trace_index = 224.0;
  for (std::size_t i = 0; i < 4; ++i) {
    const auto index = static_cast<double>(i);
    data.target_quaternion[i] = 230.0 + index;
    data.actual_quaternion[i] = 240.0 + index;
  }

  const auto message = mg400_interface::toRealtimeFeedbackMessage(data);

  EXPECT_EQ(data.digital_inputs, message.digital_inputs);
  EXPECT_EQ(data.digital_outputs, message.digital_outputs);
  EXPECT_EQ(data.robot_mode, message.robot_mode);
  EXPECT_EQ(data.controller_timestamp, message.controller_timestamp);
  EXPECT_DOUBLE_EQ(data.speed_scaling, message.speed_scaling);
  EXPECT_DOUBLE_EQ(data.v_main, message.v_main);
  EXPECT_DOUBLE_EQ(data.v_robot, message.v_robot);
  EXPECT_DOUBLE_EQ(data.i_robot, message.i_robot);

  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(data.q_target[i], message.q_target[i]);
    EXPECT_DOUBLE_EQ(data.qd_target[i], message.qd_target[i]);
    EXPECT_DOUBLE_EQ(data.qdd_target[i], message.qdd_target[i]);
    EXPECT_DOUBLE_EQ(data.i_target[i], message.i_target[i]);
    EXPECT_DOUBLE_EQ(data.m_target[i], message.m_target[i]);
    EXPECT_DOUBLE_EQ(data.q_actual[i], message.q_actual[i]);
    EXPECT_DOUBLE_EQ(data.qd_actual[i], message.qd_actual[i]);
    EXPECT_DOUBLE_EQ(data.i_actual[i], message.i_actual[i]);
    EXPECT_DOUBLE_EQ(data.actual_tcp_force[i], message.actual_tcp_force[i]);
    EXPECT_DOUBLE_EQ(data.tool_vector_actual[i], message.tool_vector_actual[i]);
    EXPECT_DOUBLE_EQ(data.tcp_speed_actual[i], message.tcp_speed_actual[i]);
    EXPECT_DOUBLE_EQ(data.tcp_force[i], message.tcp_force[i]);
    EXPECT_DOUBLE_EQ(data.tool_vector_target[i], message.tool_vector_target[i]);
    EXPECT_DOUBLE_EQ(data.tcp_speed_target[i], message.tcp_speed_target[i]);
    EXPECT_DOUBLE_EQ(data.motor_temperatures[i], message.motor_temperatures[i]);
    EXPECT_DOUBLE_EQ(data.joint_modes[i], message.joint_modes[i]);
    EXPECT_DOUBLE_EQ(data.v_actual[i], message.v_actual[i]);
    EXPECT_DOUBLE_EQ(data.m_actual[i], message.m_actual[i]);
    EXPECT_DOUBLE_EQ(data.user[i], message.user[i]);
    EXPECT_DOUBLE_EQ(data.tool[i], message.tool[i]);
    EXPECT_DOUBLE_EQ(data.six_force_value[i], message.six_force_value[i]);
  }

  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(data.hand_type[i], message.hand_type[i]);
    EXPECT_DOUBLE_EQ(data.target_quaternion[i], message.target_quaternion[i]);
    EXPECT_DOUBLE_EQ(data.actual_quaternion[i], message.actual_quaternion[i]);
  }

  EXPECT_EQ(data.user_index, message.user_index);
  EXPECT_EQ(data.tool_index, message.tool_index);
  EXPECT_EQ(data.run_queued_cmd, message.run_queued_cmd);
  EXPECT_EQ(data.pause_cmd_flag, message.pause_cmd_flag);
  EXPECT_EQ(data.velocity_ratio, message.velocity_ratio);
  EXPECT_EQ(data.acceleration_ratio, message.acceleration_ratio);
  EXPECT_EQ(data.jerk_ratio, message.jerk_ratio);
  EXPECT_EQ(data.xyz_velocity_ratio, message.xyz_velocity_ratio);
  EXPECT_EQ(data.r_velocity_ratio, message.r_velocity_ratio);
  EXPECT_EQ(data.xyz_acceleration_ratio, message.xyz_acceleration_ratio);
  EXPECT_EQ(data.r_acceleration_ratio, message.r_acceleration_ratio);
  EXPECT_EQ(data.xyz_jerk_ratio, message.xyz_jerk_ratio);
  EXPECT_EQ(data.r_jerk_ratio, message.r_jerk_ratio);
  EXPECT_EQ(data.brake_status, message.brake_status);
  EXPECT_EQ(data.enable_status, message.enable_status);
  EXPECT_EQ(data.drag_status, message.drag_status);
  EXPECT_EQ(data.running_status, message.running_status);
  EXPECT_EQ(data.error_status, message.error_status);
  EXPECT_EQ(data.jog_status, message.jog_status);
  EXPECT_EQ(data.robot_type, message.robot_type);
  EXPECT_EQ(data.drag_button_signal, message.drag_button_signal);
  EXPECT_EQ(data.enable_button_signal, message.enable_button_signal);
  EXPECT_EQ(data.record_button_signal, message.record_button_signal);
  EXPECT_EQ(data.reappear_button_signal, message.reappear_button_signal);
  EXPECT_EQ(data.jaw_button_signal, message.jaw_button_signal);
  EXPECT_EQ(data.six_force_online, message.six_force_online);
  EXPECT_DOUBLE_EQ(data.load, message.load);
  EXPECT_DOUBLE_EQ(data.center_x, message.center_x);
  EXPECT_DOUBLE_EQ(data.center_y, message.center_y);
  EXPECT_DOUBLE_EQ(data.center_z, message.center_z);
  EXPECT_DOUBLE_EQ(data.trace_index, message.trace_index);
}

}  // namespace
