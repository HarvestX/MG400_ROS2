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

#include "mg400_interface/robot_state_machine.hpp"

#include <mg400_msgs/msg/robot_mode.hpp>

namespace mg400_interface
{

RobotStateMachine::RobotStateMachine() noexcept
: state_(State::UNKNOWN)
{
}

RobotStateMachine::State RobotStateMachine::getState() const noexcept
{
  return this->state_.load();
}

bool RobotStateMachine::isState(const State expected) const noexcept
{
  return this->getState() == expected;
}

RobotStateMachine::State RobotStateMachine::fromRobotMode(const uint64_t robot_mode) noexcept
{
  using RobotMode = mg400_msgs::msg::RobotMode;

  switch (robot_mode) {
    case RobotMode::BRAKE_OPEN:
    case RobotMode::BACKDRIVE:
      return State::MANUAL;
    case RobotMode::DISABLED:
      return State::DISABLED;
    case RobotMode::ENABLE:
      return State::ENABLED;
    case RobotMode::RUNNING:
      return State::RUNNING;
    case RobotMode::PAUSE:
    case RobotMode::JOG:
      return State::PAUSED_OR_JOG;
    case RobotMode::ERROR:
      return State::ERROR;
    case RobotMode::INIT:
    case RobotMode::RECORDING:
    case RobotMode::INVALID:
    default:
      return State::NOT_READY;
  }
}

void RobotStateMachine::update(const uint64_t robot_mode) noexcept
{
  this->state_.store(fromRobotMode(robot_mode));
}

void RobotStateMachine::reset() noexcept
{
  this->state_.store(State::UNKNOWN);
}

}  // namespace mg400_interface
