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

#include <gtest/gtest.h>

#include <mg400_msgs/msg/robot_mode.hpp>
#include <mg400_msgs/msg/robot_state.hpp>

#include "mg400_interface/robot_state_machine.hpp"

namespace mg400_interface
{

TEST(TestRobotStateMachine, InitialStateIsUnknown)
{
  const RobotStateMachine state_machine;

  EXPECT_EQ(state_machine.getState(), RobotStateMachine::State::UNKNOWN);
  EXPECT_TRUE(state_machine.isState(RobotStateMachine::State::UNKNOWN));

  const auto snapshot = state_machine.getSnapshot();
  EXPECT_EQ(snapshot.raw_robot_mode, mg400_msgs::msg::RobotState::RAW_MODE_UNAVAILABLE);
  EXPECT_FALSE(snapshot.feedback_fresh);
}

TEST(TestRobotStateMachine, MapsRobotModes)
{
  using RobotMode = mg400_msgs::msg::RobotMode;
  using State = RobotStateMachine::State;

  EXPECT_EQ(RobotStateMachine::fromRobotMode(RobotMode::INIT), State::NOT_READY);
  EXPECT_EQ(RobotStateMachine::fromRobotMode(RobotMode::BRAKE_OPEN), State::MANUAL);
  EXPECT_EQ(RobotStateMachine::fromRobotMode(RobotMode::DISABLED), State::DISABLED);
  EXPECT_EQ(RobotStateMachine::fromRobotMode(RobotMode::ENABLE), State::ENABLED);
  EXPECT_EQ(RobotStateMachine::fromRobotMode(RobotMode::BACKDRIVE), State::MANUAL);
  EXPECT_EQ(RobotStateMachine::fromRobotMode(RobotMode::RUNNING), State::RUNNING);
  EXPECT_EQ(RobotStateMachine::fromRobotMode(RobotMode::RECORDING), State::NOT_READY);
  EXPECT_EQ(RobotStateMachine::fromRobotMode(RobotMode::ERROR), State::ERROR);
  EXPECT_EQ(RobotStateMachine::fromRobotMode(RobotMode::PAUSE), State::PAUSED_OR_JOG);
  EXPECT_EQ(RobotStateMachine::fromRobotMode(RobotMode::JOG), State::PAUSED_OR_JOG);
  EXPECT_EQ(RobotStateMachine::fromRobotMode(RobotMode::INVALID), State::NOT_READY);
  EXPECT_EQ(RobotStateMachine::fromRobotMode(999), State::NOT_READY);
}

TEST(TestRobotStateMachine, StateValuesMatchRosMessage)
{
  using Message = mg400_msgs::msg::RobotState;
  using State = RobotStateMachine::State;

  EXPECT_EQ(static_cast<uint8_t>(State::UNKNOWN), Message::UNKNOWN);
  EXPECT_EQ(static_cast<uint8_t>(State::NOT_READY), Message::NOT_READY);
  EXPECT_EQ(static_cast<uint8_t>(State::MANUAL), Message::MANUAL);
  EXPECT_EQ(static_cast<uint8_t>(State::DISABLED), Message::DISABLED);
  EXPECT_EQ(static_cast<uint8_t>(State::ENABLED), Message::ENABLED);
  EXPECT_EQ(static_cast<uint8_t>(State::RUNNING), Message::RUNNING);
  EXPECT_EQ(static_cast<uint8_t>(State::PAUSED_OR_JOG), Message::PAUSED_OR_JOG);
  EXPECT_EQ(static_cast<uint8_t>(State::ERROR), Message::ERROR);
}

}  // namespace mg400_interface
