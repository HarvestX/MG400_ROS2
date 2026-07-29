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

#ifndef MG400_INTERFACE__SERVO_CONTROL_SESSION_HPP_
#define MG400_INTERFACE__SERVO_CONTROL_SESSION_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "mg400_interface/commander/motion_commander.hpp"
#include "mg400_interface/control_state_manager.hpp"
#include "mg400_interface/servo_feedback_state.hpp"
#include "mg400_interface/servo_kinematics_validator.hpp"
#include "mg400_interface/servo_operational_error.hpp"
#include "mg400_interface/servo_safety_violation.hpp"
#include "mg400_interface/servo_stop_strategy.hpp"

namespace mg400_interface
{

/// ROS-independent owner and periodic runner for one ServoJ lease.
class ServoControlSession
{
public:
  using Clock = std::chrono::steady_clock;
  using LeaseId = ControlStateManager::LeaseId;
  using FeedbackState = ServoFeedbackState;
  using OperationalErrorState = ServoOperationalErrorState;
  using SafetyViolationState = ServoSafetyViolationState;

  enum class State
  {
    IDLE,
    STARTING,
    ACTIVE,
    STOPPING,
    FAULTED,
  };

  enum class StopCause
  {
    NONE,
    EXPLICIT,
    WATCHDOG,
    FAULT,
  };

  struct Options
  {
    std::chrono::nanoseconds send_period = std::chrono::milliseconds(30);
    std::chrono::nanoseconds target_watchdog_timeout = std::chrono::milliseconds(100);
    std::chrono::nanoseconds response_poll_period = std::chrono::milliseconds(5);
    std::chrono::nanoseconds stop_confirmation_timeout = std::chrono::seconds(2);
    std::chrono::nanoseconds feedback_timeout = std::chrono::milliseconds(100);
    double max_initial_joint_distance_rad = 0.0872665;
    double max_joint_step_rad = 0.0174533;
    std::function<void(const std::string &)> safety_log_callback;
    std::function<void(ServoOperationalErrorCode, const std::string &)>
    operational_log_callback;
  };

  struct Result
  {
    bool success;
    State state;
    LeaseId lease_id;
    std::string message;
  };

  struct Snapshot
  {
    State state;
    LeaseId lease_id;
    StopCause stop_cause;
    std::string diagnostic;
  };

  ServoControlSession(
    ControlStateManager::SharedPtr control_state_manager,
    MotionCommander::SharedPtr motion_commander,
    ServoStopStrategy::SharedPtr stop_strategy,
    SafetyViolationState::SharedPtr safety_violation_state,
    OperationalErrorState::SharedPtr operational_error_state,
    FeedbackState::SharedPtr feedback_state,
    const Options & options);
  ~ServoControlSession();

  ServoControlSession(const ServoControlSession &) = delete;
  ServoControlSession & operator=(const ServoControlSession &) = delete;

  /// Acquire a fresh ServoJ lease.
  Result start();

  /// Store a latest-only SI-unit target. No TCP operation occurs in these calls.
  bool updateServoJTarget(LeaseId lease_id, const std::array<double, 4> & joint_angles);

  /// Safely stop, or retry a failed stop, using this session's active lease.
  Result stop();

  Snapshot getSnapshot() const;

  SafetyViolationState::SharedPtr getSafetyViolationStateShared() const noexcept;
  OperationalErrorState::SharedPtr getOperationalErrorStateShared() const noexcept;

private:
  ControlStateManager::SharedPtr control_state_manager_;
  MotionCommander::SharedPtr motion_commander_;
  ServoStopStrategy::SharedPtr stop_strategy_;
  SafetyViolationState::SharedPtr safety_violation_state_;
  OperationalErrorState::SharedPtr operational_error_state_;
  FeedbackState::SharedPtr feedback_state_;
  FeedbackState::ConnectionEpoch connection_epoch_;
  ServoKinematicsValidator kinematics_validator_;
  Options options_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  State state_;
  LeaseId lease_id_;
  bool has_target_;
  std::array<double, 4> target_;
  Clock::time_point started_at_;
  Clock::time_point target_updated_at_;
  bool has_target_update_time_;
  Clock::time_point next_send_at_;
  Clock::time_point next_response_poll_at_;
  bool worker_should_exit_;
  StopCause stop_cause_;
  std::uint64_t response_drop_baseline_;
  std::string diagnostic_;
  bool has_previous_successful_command_;
  std::array<double, 4> previous_successful_target_;
  bool safety_fault_requested_;

  mutable std::mutex mutex_worker_;
  std::thread worker_;
  std::atomic<bool> stop_attempt_in_progress_;

  void workerLoop();
  void workerLoopImpl();
  bool monitorResponses(std::string & fault_message);
  bool sendLatestTarget(LeaseId lease_id);
  bool rejectUnsafeTarget(
    const ServoKinematicsValidator::Result & violation,
    const std::string & diagnostic,
    LeaseId lease_id);
  bool rejectUnsafeSend(
    ServoSafetyViolationCode code,
    const std::string & message,
    const std::string & detail);
  void reportSafetyViolation(
    ServoSafetyViolationCode code,
    const std::string & message,
    const std::string & detail);
  void reportOperationalError(
    ServoOperationalErrorCode code,
    const std::string & message,
    const std::string & detail);
  bool targetMatchesSession(LeaseId lease_id);
  Result performStop(StopCause cause, LeaseId lease_id, const std::string & detail);
  void joinWorker();
  void requestWorkerExit();
  void advanceSendDeadlineLocked(const Clock::time_point & now);
  void advanceResponseDeadlineLocked(const Clock::time_point & now);
  bool currentLeaseMatches(LeaseId lease_id) const;
  bool rejectTarget(const std::string & reason);
  Result localResult(bool success, LeaseId lease_id, const std::string & message) const;

  static bool isFinite(const std::array<double, 4> & values) noexcept;
  static const char * responseResultName(MotionResponseResult result) noexcept;
  static ServoOperationalErrorCode responseErrorCode(MotionResponseResult result) noexcept;
};

}  // namespace mg400_interface

#endif  // MG400_INTERFACE__SERVO_CONTROL_SESSION_HPP_
