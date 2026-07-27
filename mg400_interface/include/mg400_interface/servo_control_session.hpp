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
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "mg400_interface/commander/motion_commander.hpp"
#include "mg400_interface/control_state_manager.hpp"
#include "mg400_interface/servo_stop_strategy.hpp"

namespace mg400_interface
{

/// ROS-independent owner and periodic runner for one ServoJ or ServoP lease.
class ServoControlSession
{
public:
  using Clock = std::chrono::steady_clock;
  using LeaseId = ControlStateManager::LeaseId;
  using ServoType = ControlStateManager::State;

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
    ServoType servo_type;
    LeaseId lease_id;
    bool has_target;
    bool has_target_update_time;
    Clock::time_point target_updated_at;
    bool watchdog_triggered;
    StopCause stop_cause;
    std::uint64_t accepted_target_count;
    std::uint64_t rejected_target_count;
    std::uint64_t sent_command_count;
    bool has_last_send_time;
    Clock::time_point last_sent_at;
    bool has_latest_response;
    MotionResponse latest_response;
    std::uint64_t dropped_response_count;
    std::string diagnostic;
  };

  ServoControlSession(
    ControlStateManager::SharedPtr control_state_manager,
    MotionCommander::SharedPtr motion_commander,
    ServoStopStrategy::SharedPtr stop_strategy);
  ServoControlSession(
    ControlStateManager::SharedPtr control_state_manager,
    MotionCommander::SharedPtr motion_commander,
    ServoStopStrategy::SharedPtr stop_strategy,
    const Options & options);
  ~ServoControlSession();

  ServoControlSession(const ServoControlSession &) = delete;
  ServoControlSession & operator=(const ServoControlSession &) = delete;

  /// Acquire a fresh Servo lease. Only SERVO_J and SERVO_P are accepted.
  Result start(ServoType servo_type);

  /// Store a latest-only SI-unit target. No TCP operation occurs in these calls.
  bool updateServoJTarget(LeaseId lease_id, const std::array<double, 4> & joint_angles);
  bool updateServoPTarget(
    LeaseId lease_id, double x, double y, double z, double yaw);

  /// Safely stop, or retry a failed stop, using the active lease.
  Result stop(LeaseId lease_id);

  Snapshot getSnapshot() const;

private:
  ControlStateManager::SharedPtr control_state_manager_;
  MotionCommander::SharedPtr motion_commander_;
  ServoStopStrategy::SharedPtr stop_strategy_;
  Options options_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  State state_;
  ServoType servo_type_;
  LeaseId lease_id_;
  bool has_target_;
  std::array<double, 4> target_;
  Clock::time_point started_at_;
  Clock::time_point target_updated_at_;
  bool has_target_update_time_;
  Clock::time_point next_send_at_;
  Clock::time_point next_response_poll_at_;
  bool worker_should_exit_;
  bool watchdog_triggered_;
  StopCause stop_cause_;
  std::uint64_t accepted_target_count_;
  std::uint64_t rejected_target_count_;
  std::uint64_t sent_command_count_;
  bool has_last_send_time_;
  Clock::time_point last_sent_at_;
  bool has_latest_response_;
  MotionResponse latest_response_;
  std::uint64_t response_drop_baseline_;
  std::uint64_t dropped_response_count_;
  std::string diagnostic_;

  mutable std::mutex mutex_worker_;
  std::thread worker_;
  std::atomic<bool> stop_attempt_in_progress_;

  void workerLoop();
  void workerLoopImpl();
  bool monitorResponses(std::string & fault_message);
  bool sendLatestTarget(ServoType servo_type, LeaseId lease_id);
  Result performStop(StopCause cause, LeaseId lease_id, const std::string & detail);
  void joinWorker();
  void requestWorkerExit();
  void advanceSendDeadlineLocked(const Clock::time_point & now);
  void advanceResponseDeadlineLocked(const Clock::time_point & now);
  bool currentLeaseMatches(LeaseId lease_id, ServoType servo_type) const;
  bool rejectTarget(const std::string & reason);
  Result localResult(bool success, LeaseId lease_id, const std::string & message) const;

  static bool isServoType(ServoType servo_type) noexcept;
  static bool isFinite(const std::array<double, 4> & values) noexcept;
  static const char * responseResultName(MotionResponseResult result) noexcept;
};

}  // namespace mg400_interface

#endif  // MG400_INTERFACE__SERVO_CONTROL_SESSION_HPP_
