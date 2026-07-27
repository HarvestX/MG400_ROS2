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

#ifndef MG400_INTERFACE__CONTROL_STATE_MANAGER_HPP_
#define MG400_INTERFACE__CONTROL_STATE_MANAGER_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace mg400_interface
{

/// Thread-safe arbiter for ROS-side MG400 motion ownership.
class ControlStateManager
{
public:
  using SharedPtr = std::shared_ptr<ControlStateManager>;
  using LeaseId = std::uint64_t;

  static constexpr LeaseId NO_LEASE = 0;

  enum class State : std::uint8_t
  {
    UNAVAILABLE = 0,
    IDLE = 1,
    SERVO_J = 2,
    SERVO_P = 3,
  };

  enum class MotionOwner : std::uint8_t
  {
    NONE = 0,
    REGULAR_MOTION,
    SERVO_J,
    SERVO_P,
  };

  struct Snapshot
  {
    State control_state;
    MotionOwner motion_owner;
    LeaseId lease_id;
    bool connected;
    std::uint64_t robot_mode;
    bool accepting_servo_targets;
  };

  struct Result
  {
    bool success;
    State current_state;
    LeaseId lease_id;
    std::string message;
  };

  ControlStateManager();

  /// Apply the latest connection and embedded RobotMode observation.
  Snapshot updateRobotStatus(bool connected, std::uint64_t robot_mode);

  /// Request IDLE -> SERVO_J/P or complete SERVO_J/P -> IDLE.
  ///
  /// A successful Servo start returns a lease that must be supplied to later
  /// target checks and stop operations. Before requesting IDLE, call
  /// beginServoStop(), perform the safe-stop action, and then complete the
  /// transition with this method. UNAVAILABLE cannot be requested.
  Result requestControlState(State target_state, LeaseId lease_id = NO_LEASE);

  /// Stop target admission while retaining ownership during the safe stop.
  Result beginServoStop(LeaseId lease_id);

  /// Atomically acquire ownership for MovJ, MovL, Jog, CommandQueue, etc.
  Result tryAcquireRegularMotion();

  /// Release regular-motion ownership. Stale or foreign leases are rejected.
  Result releaseRegularMotion(LeaseId lease_id);

  /// Release Servo ownership after the watchdog has performed its stop action.
  Result handleServoWatchdogTimeout(LeaseId lease_id);

  /// Check that a target belongs to the currently active Servo session.
  bool acceptsServoTarget(State servo_state, LeaseId lease_id) const;

  Snapshot getSnapshot() const;
  State getState() const;

  static const char * toString(State state) noexcept;
  static const char * toString(MotionOwner owner) noexcept;

private:
  static constexpr std::uint64_t ROBOT_MODE_ENABLE = 5;
  static constexpr std::uint64_t ROBOT_MODE_RUNNING = 7;
  static constexpr std::uint64_t ROBOT_MODE_PAUSE = 10;
  static constexpr std::uint64_t ROBOT_MODE_JOG = 11;
  static constexpr std::uint64_t ROBOT_MODE_INVALID = 12;

  mutable std::mutex mutex_;
  State control_state_;
  MotionOwner motion_owner_;
  LeaseId lease_id_;
  LeaseId next_lease_id_;
  bool connected_;
  std::uint64_t robot_mode_;
  bool accepting_servo_targets_;

  LeaseId allocateLeaseLocked();
  Snapshot getSnapshotLocked() const;
  Result makeResultLocked(bool success, const std::string & message) const;
  void clearOwnershipLocked();
  void setStateWithoutOwnerLocked();

  static bool isServoState(State state) noexcept;
  static bool isServoOwner(MotionOwner owner) noexcept;
  static bool isServoAllowedRobotMode(std::uint64_t robot_mode) noexcept;
  static bool isRegularMotionRobotMode(std::uint64_t robot_mode) noexcept;
};

}  // namespace mg400_interface

#endif  // MG400_INTERFACE__CONTROL_STATE_MANAGER_HPP_
