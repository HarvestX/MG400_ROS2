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
: snapshot_{}
{
}

RobotStateMachine::State RobotStateMachine::getState() const
{
  return this->getSnapshot().state;
}

bool RobotStateMachine::isState(const State expected) const
{
  return this->getState() == expected;
}

RobotStateMachine::Snapshot RobotStateMachine::getSnapshot() const
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->snapshot_;
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

bool RobotStateMachine::tryBeginServoSession()
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  if (this->servo_session_active_ || !this->snapshot_.feedback_fresh ||
    this->snapshot_.state != State::ENABLED)
  {
    return false;
  }
  this->servo_session_active_ = true;
  this->snapshot_.state = State::SERVO;
  return true;
}

void RobotStateMachine::endServoSession()
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->servo_session_active_ = false;
  this->snapshot_.state = this->snapshot_.feedback_fresh ?
    fromRobotMode(this->snapshot_.raw_robot_mode) : State::UNKNOWN;
}

bool RobotStateMachine::isServoSessionActive() const
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->servo_session_active_;
}

void RobotStateMachine::update(const uint64_t robot_mode)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->snapshot_.state = fromRobotMode(robot_mode);
  this->snapshot_.raw_robot_mode = robot_mode;
  this->snapshot_.feedback_fresh = true;
  if (this->servo_session_active_) {
    if (this->snapshot_.state == State::ENABLED || this->snapshot_.state == State::RUNNING) {
      this->snapshot_.state = State::SERVO;
    } else {
      this->servo_session_active_ = false;
    }
  }
}

void RobotStateMachine::reset()
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->snapshot_.state = State::UNKNOWN;
  this->snapshot_.raw_robot_mode = 0;
  this->snapshot_.feedback_fresh = false;
  this->servo_session_active_ = false;
}

}  // namespace mg400_interface
