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

#include "mg400_interface/control_state_manager.hpp"

#include <limits>

#include <mg400_msgs/msg/control_state.hpp>
#include <mg400_msgs/msg/robot_mode.hpp>

namespace mg400_interface
{

static_assert(
  static_cast<std::uint8_t>(ControlStateManager::State::UNAVAILABLE) ==
  mg400_msgs::msg::ControlState::UNAVAILABLE,
  "ControlStateManager and ControlState.msg must use the same values");
static_assert(
  static_cast<std::uint8_t>(ControlStateManager::State::IDLE) ==
  mg400_msgs::msg::ControlState::IDLE,
  "ControlStateManager and ControlState.msg must use the same values");
static_assert(
  static_cast<std::uint8_t>(ControlStateManager::State::SERVO_J) ==
  mg400_msgs::msg::ControlState::SERVO_J,
  "ControlStateManager and ControlState.msg must use the same values");
static_assert(
  static_cast<std::uint8_t>(ControlStateManager::State::REGULAR_MOTION) ==
  mg400_msgs::msg::ControlState::REGULAR_MOTION,
  "ControlStateManager and ControlState.msg must use the same values");
static_assert(
  mg400_msgs::msg::RobotMode::ENABLE == 5,
  "ControlStateManager expects the MG400 four-axis RobotMode values");
static_assert(
  mg400_msgs::msg::RobotMode::RUNNING == 7,
  "ControlStateManager expects the MG400 four-axis RobotMode values");
static_assert(
  mg400_msgs::msg::RobotMode::PAUSE == 10,
  "ControlStateManager expects the MG400 four-axis RobotMode values");
static_assert(
  mg400_msgs::msg::RobotMode::JOG == 11,
  "ControlStateManager expects the MG400 four-axis RobotMode values");
static_assert(
  mg400_msgs::msg::RobotMode::INVALID == 12,
  "ControlStateManager expects the ROS-side invalid RobotMode sentinel");

ControlStateManager::ControlStateManager()
: control_state_(State::UNAVAILABLE),
  lease_id_(NO_LEASE),
  next_lease_id_(NO_LEASE),
  connected_(false),
  robot_mode_(ROBOT_MODE_INVALID),
  accepting_servo_targets_(false)
{
}

ControlStateManager::Snapshot ControlStateManager::updateRobotStatus(
  const bool connected, const std::uint64_t robot_mode)
{
  std::lock_guard<std::mutex> lock(this->mutex_);

  this->connected_ = connected;
  this->robot_mode_ = connected ? robot_mode : ROBOT_MODE_INVALID;

  if (!this->connected_) {
    this->clearOwnershipLocked();
    this->control_state_ = State::UNAVAILABLE;
    return this->getSnapshotLocked();
  }

  if (this->control_state_ == State::SERVO_J ||
    this->control_state_ == State::REGULAR_MOTION)
  {
    if (!isAllowedRobotMode(this->control_state_, this->robot_mode_)) {
      this->clearOwnershipLocked();
      this->control_state_ = State::UNAVAILABLE;
    }
    return this->getSnapshotLocked();
  }

  this->setStateWithoutOwnerLocked();
  return this->getSnapshotLocked();
}

ControlStateManager::Result ControlStateManager::tryAcquire(const State target_state)
{
  std::lock_guard<std::mutex> lock(this->mutex_);

  if (!isAcquirableState(target_state)) {
    return this->makeResultLocked(false, "Requested control state cannot be acquired");
  }
  if (!this->connected_) {
    return this->makeResultLocked(false, "MG400 is not connected");
  }
  if (this->robot_mode_ != ROBOT_MODE_ENABLE) {
    return this->makeResultLocked(false, "RobotMode is not ENABLE");
  }
  if (this->control_state_ != State::IDLE) {
    return this->makeResultLocked(false, "Control state is not IDLE");
  }

  this->lease_id_ = this->allocateLeaseLocked();
  this->control_state_ = target_state;
  this->accepting_servo_targets_ = target_state == State::SERVO_J;
  return this->makeResultLocked(true, "Control state acquired");
}

ControlStateManager::Result ControlStateManager::beginServoStop(const LeaseId lease_id)
{
  std::lock_guard<std::mutex> lock(this->mutex_);

  if (this->control_state_ != State::SERVO_J) {
    return this->makeResultLocked(false, "Servo control is not active");
  }
  if (lease_id == NO_LEASE || lease_id != this->lease_id_) {
    return this->makeResultLocked(false, "Servo lease does not match the active owner");
  }
  if (!this->accepting_servo_targets_) {
    return this->makeResultLocked(false, "Servo stop is already in progress");
  }

  this->accepting_servo_targets_ = false;
  return this->makeResultLocked(true, "Servo target admission stopped");
}

ControlStateManager::Result ControlStateManager::release(
  const State owned_state, const LeaseId lease_id)
{
  std::lock_guard<std::mutex> lock(this->mutex_);

  if (!isAcquirableState(owned_state) || this->control_state_ != owned_state) {
    return this->makeResultLocked(false, "Requested control ownership is not active");
  }
  if (lease_id == NO_LEASE || lease_id != this->lease_id_) {
    return this->makeResultLocked(false, "Lease does not match the active owner");
  }
  if (owned_state == State::SERVO_J && this->accepting_servo_targets_) {
    return this->makeResultLocked(false, "Servo stop has not begun");
  }

  this->clearOwnershipLocked();
  this->setStateWithoutOwnerLocked();
  return this->makeResultLocked(true, "Control state released");
}

bool ControlStateManager::owns(
  const State owned_state, const LeaseId lease_id) const
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  return isAcquirableState(owned_state) && lease_id != NO_LEASE &&
         this->control_state_ == owned_state && this->lease_id_ == lease_id;
}

bool ControlStateManager::acceptsServoJTarget(const LeaseId lease_id) const
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  return lease_id != NO_LEASE && this->connected_ &&
         isAllowedRobotMode(State::SERVO_J, this->robot_mode_) &&
         this->accepting_servo_targets_ &&
         this->control_state_ == State::SERVO_J && this->lease_id_ == lease_id;
}

ControlStateManager::Snapshot ControlStateManager::getSnapshot() const
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->getSnapshotLocked();
}

ControlStateManager::State ControlStateManager::getState() const
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->control_state_;
}

const char * ControlStateManager::toString(const State state) noexcept
{
  switch (state) {
    case State::UNAVAILABLE:
      return "UNAVAILABLE";
    case State::IDLE:
      return "IDLE";
    case State::SERVO_J:
      return "SERVO_J";
    case State::REGULAR_MOTION:
      return "REGULAR_MOTION";
    default:
      return "UNKNOWN";
  }
}

ControlStateManager::LeaseId ControlStateManager::allocateLeaseLocked()
{
  if (this->next_lease_id_ == std::numeric_limits<LeaseId>::max()) {
    this->next_lease_id_ = NO_LEASE;
  }
  ++this->next_lease_id_;
  return this->next_lease_id_;
}

ControlStateManager::Snapshot ControlStateManager::getSnapshotLocked() const
{
  return Snapshot{
    this->control_state_, this->lease_id_, this->connected_,
    this->robot_mode_, this->accepting_servo_targets_};
}

ControlStateManager::Result ControlStateManager::makeResultLocked(
  const bool success, const std::string & message) const
{
  return Result{success, this->control_state_, this->lease_id_, message};
}

void ControlStateManager::clearOwnershipLocked()
{
  this->lease_id_ = NO_LEASE;
  this->accepting_servo_targets_ = false;
}

void ControlStateManager::setStateWithoutOwnerLocked()
{
  this->control_state_ =
    this->connected_ && this->robot_mode_ == ROBOT_MODE_ENABLE ?
    State::IDLE : State::UNAVAILABLE;
}

bool ControlStateManager::isAcquirableState(const State state) noexcept
{
  return state == State::REGULAR_MOTION || state == State::SERVO_J;
}

bool ControlStateManager::isAllowedRobotMode(
  const State state, const std::uint64_t robot_mode) noexcept
{
  if (state == State::SERVO_J) {
    return robot_mode == ROBOT_MODE_ENABLE || robot_mode == ROBOT_MODE_RUNNING;
  }
  if (state == State::REGULAR_MOTION) {
    return robot_mode == ROBOT_MODE_ENABLE || robot_mode == ROBOT_MODE_RUNNING ||
           robot_mode == ROBOT_MODE_PAUSE || robot_mode == ROBOT_MODE_JOG;
  }
  return false;
}

}  // namespace mg400_interface
