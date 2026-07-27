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
  static_cast<std::uint8_t>(ControlStateManager::State::SERVO_P) ==
  mg400_msgs::msg::ControlState::SERVO_P,
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
  motion_owner_(MotionOwner::NONE),
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

  if (isServoOwner(this->motion_owner_)) {
    if (isServoAllowedRobotMode(this->robot_mode_)) {
      this->control_state_ = this->motion_owner_ == MotionOwner::SERVO_J ?
        State::SERVO_J : State::SERVO_P;
    } else {
      this->clearOwnershipLocked();
      this->control_state_ = State::UNAVAILABLE;
    }
    return this->getSnapshotLocked();
  }

  if (this->motion_owner_ == MotionOwner::REGULAR_MOTION) {
    if (isRegularMotionRobotMode(this->robot_mode_)) {
      // IDLE means that Servo does not own control. The regular-motion lease
      // still prevents Servo admission while the embedded robot is active.
      this->control_state_ = State::IDLE;
    } else {
      this->clearOwnershipLocked();
      this->control_state_ = State::UNAVAILABLE;
    }
    return this->getSnapshotLocked();
  }

  this->setStateWithoutOwnerLocked();
  return this->getSnapshotLocked();
}

ControlStateManager::Result ControlStateManager::requestControlState(
  const State target_state, const LeaseId lease_id)
{
  std::lock_guard<std::mutex> lock(this->mutex_);

  if (target_state == State::UNAVAILABLE) {
    return this->makeResultLocked(false, "UNAVAILABLE cannot be requested");
  }

  if (target_state == State::IDLE) {
    if (!isServoOwner(this->motion_owner_)) {
      return this->makeResultLocked(false, "Servo control is not active");
    }
    if (lease_id == NO_LEASE || lease_id != this->lease_id_) {
      return this->makeResultLocked(false, "Servo lease does not match the active owner");
    }
    if (this->accepting_servo_targets_) {
      return this->makeResultLocked(false, "Servo stop has not begun");
    }

    this->clearOwnershipLocked();
    // A completed explicit stop is a Servo -> IDLE transition. A subsequent
    // status update can still move the state to UNAVAILABLE if necessary.
    this->control_state_ = State::IDLE;
    return this->makeResultLocked(true, "Servo control stopped");
  }

  if (!isServoState(target_state)) {
    return this->makeResultLocked(false, "Unknown control state");
  }
  if (lease_id != NO_LEASE) {
    return this->makeResultLocked(false, "A Servo start request must use lease ID zero");
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
  if (this->motion_owner_ != MotionOwner::NONE) {
    return this->makeResultLocked(false, "Motion ownership is already held");
  }

  this->lease_id_ = this->allocateLeaseLocked();
  this->motion_owner_ = target_state == State::SERVO_J ?
    MotionOwner::SERVO_J : MotionOwner::SERVO_P;
  this->control_state_ = target_state;
  this->accepting_servo_targets_ = true;
  return this->makeResultLocked(true, "Servo control started");
}

ControlStateManager::Result ControlStateManager::beginServoStop(const LeaseId lease_id)
{
  std::lock_guard<std::mutex> lock(this->mutex_);

  if (!isServoOwner(this->motion_owner_)) {
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

ControlStateManager::Result ControlStateManager::tryAcquireRegularMotion()
{
  std::lock_guard<std::mutex> lock(this->mutex_);

  if (!this->connected_) {
    return this->makeResultLocked(false, "MG400 is not connected");
  }
  if (this->robot_mode_ != ROBOT_MODE_ENABLE) {
    return this->makeResultLocked(false, "RobotMode is not ENABLE");
  }
  if (this->control_state_ != State::IDLE) {
    return this->makeResultLocked(false, "Control state is not IDLE");
  }
  if (this->motion_owner_ != MotionOwner::NONE) {
    return this->makeResultLocked(false, "Motion ownership is already held");
  }

  this->lease_id_ = this->allocateLeaseLocked();
  this->motion_owner_ = MotionOwner::REGULAR_MOTION;
  return this->makeResultLocked(true, "Regular-motion ownership acquired");
}

ControlStateManager::Result ControlStateManager::releaseRegularMotion(
  const LeaseId lease_id)
{
  std::lock_guard<std::mutex> lock(this->mutex_);

  if (this->motion_owner_ != MotionOwner::REGULAR_MOTION) {
    return this->makeResultLocked(false, "Regular-motion ownership is not active");
  }
  if (lease_id == NO_LEASE || lease_id != this->lease_id_) {
    return this->makeResultLocked(false, "Regular-motion lease does not match the active owner");
  }

  this->clearOwnershipLocked();
  this->setStateWithoutOwnerLocked();
  return this->makeResultLocked(true, "Regular-motion ownership released");
}

ControlStateManager::Result ControlStateManager::handleServoWatchdogTimeout(
  const LeaseId lease_id)
{
  std::lock_guard<std::mutex> lock(this->mutex_);

  if (!isServoOwner(this->motion_owner_)) {
    return this->makeResultLocked(false, "Servo control is not active");
  }
  if (lease_id == NO_LEASE || lease_id != this->lease_id_) {
    return this->makeResultLocked(false, "Servo lease does not match the active owner");
  }
  if (this->accepting_servo_targets_) {
    return this->makeResultLocked(false, "Servo stop has not begun");
  }

  this->clearOwnershipLocked();
  // The caller must perform the configured safe-stop operation before
  // reporting the timeout here.
  this->control_state_ = State::IDLE;
  return this->makeResultLocked(true, "Servo watchdog released motion ownership");
}

bool ControlStateManager::acceptsServoTarget(
  const State servo_state, const LeaseId lease_id) const
{
  std::lock_guard<std::mutex> lock(this->mutex_);

  if (!isServoState(servo_state) || lease_id == NO_LEASE) {
    return false;
  }

  const MotionOwner expected_owner = servo_state == State::SERVO_J ?
    MotionOwner::SERVO_J : MotionOwner::SERVO_P;
  return this->connected_ &&
         isServoAllowedRobotMode(this->robot_mode_) &&
         this->accepting_servo_targets_ &&
         this->control_state_ == servo_state &&
         this->motion_owner_ == expected_owner &&
         this->lease_id_ == lease_id;
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
    case State::SERVO_P:
      return "SERVO_P";
    default:
      return "UNKNOWN";
  }
}

const char * ControlStateManager::toString(const MotionOwner owner) noexcept
{
  switch (owner) {
    case MotionOwner::NONE:
      return "NONE";
    case MotionOwner::REGULAR_MOTION:
      return "REGULAR_MOTION";
    case MotionOwner::SERVO_J:
      return "SERVO_J";
    case MotionOwner::SERVO_P:
      return "SERVO_P";
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
    this->control_state_, this->motion_owner_, this->lease_id_,
    this->connected_, this->robot_mode_, this->accepting_servo_targets_};
}

ControlStateManager::Result ControlStateManager::makeResultLocked(
  const bool success, const std::string & message) const
{
  return Result{success, this->control_state_, this->lease_id_, message};
}

void ControlStateManager::clearOwnershipLocked()
{
  this->motion_owner_ = MotionOwner::NONE;
  this->lease_id_ = NO_LEASE;
  this->accepting_servo_targets_ = false;
}

void ControlStateManager::setStateWithoutOwnerLocked()
{
  this->control_state_ =
    this->connected_ && this->robot_mode_ == ROBOT_MODE_ENABLE ?
    State::IDLE : State::UNAVAILABLE;
}

bool ControlStateManager::isServoState(const State state) noexcept
{
  return state == State::SERVO_J || state == State::SERVO_P;
}

bool ControlStateManager::isServoOwner(const MotionOwner owner) noexcept
{
  return owner == MotionOwner::SERVO_J || owner == MotionOwner::SERVO_P;
}

bool ControlStateManager::isServoAllowedRobotMode(const std::uint64_t robot_mode) noexcept
{
  return robot_mode == ROBOT_MODE_ENABLE || robot_mode == ROBOT_MODE_RUNNING;
}

bool ControlStateManager::isRegularMotionRobotMode(const std::uint64_t robot_mode) noexcept
{
  return robot_mode == ROBOT_MODE_ENABLE ||
         robot_mode == ROBOT_MODE_RUNNING ||
         robot_mode == ROBOT_MODE_PAUSE ||
         robot_mode == ROBOT_MODE_JOG;
}

}  // namespace mg400_interface
