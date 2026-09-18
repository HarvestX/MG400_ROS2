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

#include <array>

#include <mg400_msgs/msg/robot_mode.hpp>

#include "mg400_plugin_base/api_plugin_base.hpp"

namespace mg400_plugin_base
{

TEST(TestMotionApiPluginBase, StandardCommandRequiresFreshEnabledState)
{
  using State = mg400_interface::RobotStateMachine::State;
  using Snapshot = mg400_interface::RobotStateMachine::Snapshot;

  Snapshot snapshot{State::ENABLED, mg400_msgs::msg::RobotMode::ENABLE, true};
  EXPECT_TRUE(MotionApiPluginBase::isMotionStateReady(snapshot));

  snapshot.feedback_fresh = false;
  EXPECT_FALSE(MotionApiPluginBase::isMotionStateReady(snapshot));

  snapshot.feedback_fresh = true;
  const std::array disallowed_states = {
    State::UNKNOWN,
    State::NOT_READY,
    State::MANUAL,
    State::DISABLED,
    State::RUNNING,
    State::PAUSED_OR_JOG,
    State::ERROR,
  };
  for (const auto state : disallowed_states) {
    snapshot.state = state;
    EXPECT_FALSE(MotionApiPluginBase::isMotionStateReady(snapshot));
  }
}

TEST(TestMotionApiPluginBase, JogStopAllowsEnabledOrJogState)
{
  using Policy = MotionApiPluginBase::CommandPolicy;
  using State = mg400_interface::RobotStateMachine::State;
  using Snapshot = mg400_interface::RobotStateMachine::Snapshot;

  Snapshot snapshot{State::ENABLED, mg400_msgs::msg::RobotMode::ENABLE, true};
  EXPECT_TRUE(MotionApiPluginBase::isMotionStateReady(snapshot, Policy::JOG_STOP));

  snapshot = {State::PAUSED_OR_JOG, mg400_msgs::msg::RobotMode::JOG, true};
  EXPECT_TRUE(MotionApiPluginBase::isMotionStateReady(snapshot, Policy::JOG_STOP));

  snapshot.raw_robot_mode = mg400_msgs::msg::RobotMode::PAUSE;
  EXPECT_FALSE(MotionApiPluginBase::isMotionStateReady(snapshot, Policy::JOG_STOP));

  snapshot.raw_robot_mode = mg400_msgs::msg::RobotMode::JOG;
  snapshot.feedback_fresh = false;
  EXPECT_FALSE(MotionApiPluginBase::isMotionStateReady(snapshot, Policy::JOG_STOP));
}

}  // namespace mg400_plugin_base
