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

#include "mg400_interface/servo_control_session.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace mg400_interface
{
namespace
{

class AtomicFlagReset
{
public:
  explicit AtomicFlagReset(std::atomic<bool> & flag)
  : flag_(flag) {}

  ~AtomicFlagReset() {this->flag_.store(false);}

private:
  std::atomic<bool> & flag_;
};

bool isServoJCommand(const std::string & command)
{
  const char * prefix = "ServoJ(";
  return command.compare(0, std::char_traits<char>::length(prefix), prefix) == 0;
}

double shortestAngleDistance(const double lhs, const double rhs)
{
  return std::abs(std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs)));
}

std::string arrayToString(const std::array<double, 4> & values)
{
  std::ostringstream stream;
  stream << std::setprecision(12) << '[' << values[0] << ", " << values[1] << ", " <<
    values[2] << ", " << values[3] << ']';
  return stream.str();
}

}  // namespace

ServoControlSession::ServoControlSession(
  ControlStateManager::SharedPtr control_state_manager,
  MotionCommander::SharedPtr motion_commander,
  ServoStopStrategy::SharedPtr stop_strategy,
  SafetyViolationState::SharedPtr safety_violation_state,
  OperationalErrorState::SharedPtr operational_error_state,
  FeedbackState::SharedPtr feedback_state,
  const Options & options)
: control_state_manager_(std::move(control_state_manager)),
  motion_commander_(std::move(motion_commander)),
  stop_strategy_(std::move(stop_strategy)),
  safety_violation_state_(std::move(safety_violation_state)),
  operational_error_state_(std::move(operational_error_state)),
  feedback_state_(std::move(feedback_state)),
  connection_epoch_(this->feedback_state_ ? this->feedback_state_->connectionEpoch() : 0),
  options_(options),
  state_(State::IDLE),
  lease_id_(ControlStateManager::NO_LEASE),
  has_target_(false),
  target_{{0.0, 0.0, 0.0, 0.0}},
  has_target_update_time_(false),
  worker_should_exit_(true),
  stop_cause_(StopCause::NONE),
  response_drop_baseline_(0),
  diagnostic_("Servo session is idle"),
  has_previous_successful_command_(false),
  previous_successful_target_{{0.0, 0.0, 0.0, 0.0}},
  safety_fault_requested_(false),
  stop_attempt_in_progress_(false)
{
  if (!this->control_state_manager_) {
    throw std::invalid_argument("ServoControlSession requires a ControlStateManager");
  }
  if (!this->motion_commander_) {
    throw std::invalid_argument("ServoControlSession requires a MotionCommander");
  }
  if (!this->stop_strategy_) {
    throw std::invalid_argument("ServoControlSession requires a stop strategy");
  }
  if (!this->safety_violation_state_) {
    throw std::invalid_argument("ServoControlSession requires a safety violation state");
  }
  if (!this->operational_error_state_) {
    throw std::invalid_argument("ServoControlSession requires an operational error state");
  }
  if (!this->feedback_state_) {
    throw std::invalid_argument("ServoControlSession requires a feedback state");
  }
  if (this->options_.send_period <= std::chrono::nanoseconds::zero() ||
    this->options_.target_watchdog_timeout <= std::chrono::nanoseconds::zero() ||
    this->options_.response_poll_period <= std::chrono::nanoseconds::zero() ||
    this->options_.stop_confirmation_timeout <= std::chrono::nanoseconds::zero() ||
    this->options_.feedback_timeout <= std::chrono::nanoseconds::zero())
  {
    throw std::invalid_argument("ServoControlSession durations must be positive");
  }
  const std::array<double, 2> thresholds{{
    this->options_.max_initial_joint_distance_rad,
    this->options_.max_joint_step_rad}};
  if (!std::all_of(
      thresholds.begin(), thresholds.end(),
      [](const double value) {return std::isfinite(value) && value > 0.0;}))
  {
    throw std::invalid_argument("Servo safety thresholds must be positive and finite");
  }
}

ServoControlSession::~ServoControlSession()
{
  try {
    const auto snapshot = this->getSnapshot();
    if (snapshot.lease_id != ControlStateManager::NO_LEASE &&
      snapshot.state != State::IDLE)
    {
      static_cast<void>(this->performStop(
        StopCause::FAULT, snapshot.lease_id, "Servo session is being destroyed"));
    }
  } catch (...) {
    // Destruction must still terminate the periodic thread. Ownership remains
    // fail-closed in ControlStateManager if the stop could not be confirmed.
  }
  this->requestWorkerExit();
  this->joinWorker();
}

ServoControlSession::Result ServoControlSession::start()
{
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (this->state_ != State::IDLE) {
      return Result{
        false, this->state_, ControlStateManager::NO_LEASE,
        "Servo session is not IDLE"};
    }
    // Reserve the local transition while the manager performs the atomic
    // ownership acquisition.
    this->state_ = State::STARTING;
    this->diagnostic_ = "Acquiring Servo lease";
  }

  this->joinWorker();
  const auto acquisition = this->control_state_manager_->tryAcquire(
    ControlStateManager::State::SERVO_J);
  if (!acquisition.success) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->state_ = State::IDLE;
    this->diagnostic_ = acquisition.message;
    return Result{false, this->state_, ControlStateManager::NO_LEASE, acquisition.message};
  }

  const auto now = Clock::now();
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->state_ = State::STARTING;
    this->lease_id_ = acquisition.lease_id;
    this->has_target_ = false;
    this->started_at_ = now;
    this->has_target_update_time_ = false;
    this->next_send_at_ = now + this->options_.send_period;
    this->next_response_poll_at_ = now + this->options_.response_poll_period;
    this->worker_should_exit_ = false;
    this->stop_cause_ = StopCause::NONE;
    this->response_drop_baseline_ =
      this->motion_commander_->getDroppedCompletedResponseCount();
    this->has_previous_successful_command_ = false;
    this->safety_fault_requested_ = false;
    this->diagnostic_ = "Servo lease acquired; waiting for the first target";
  }

  try {
    std::lock_guard<std::mutex> worker_lock(this->mutex_worker_);
    this->worker_ = std::thread(&ServoControlSession::workerLoop, this);
  } catch (const std::exception & error) {
    const std::string message = std::string("Failed to start Servo worker: ") + error.what();
    this->reportOperationalError(
      ServoOperationalErrorCode::INTERNAL_ERROR,
      "Failed to start the Servo worker", message);
    const auto stopped = this->performStop(StopCause::FAULT, acquisition.lease_id, message);
    return Result{false, stopped.state, stopped.lease_id, message + "; " + stopped.message};
  }

  // A fully successful explicit start is the only Servo error rearm point.
  this->safety_violation_state_->clearForRearm();
  this->operational_error_state_->clearForRearm();
  return this->localResult(true, acquisition.lease_id, "Servo control started");
}

bool ServoControlSession::updateServoJTarget(
  const LeaseId lease_id, const std::array<double, 4> & joint_angles)
{
  if (!isFinite(joint_angles)) {
    return this->rejectTarget("ServoJ target contains a non-finite value");
  }
  if (!this->control_state_manager_->acceptsServoJTarget(lease_id)) {
    return this->rejectTarget("ServoJ target has a foreign, stale, or inactive lease");
  }
  if (!this->targetMatchesSession(lease_id)) {
    return this->rejectTarget("ServoJ target does not match this session");
  }

  const auto validation = this->kinematics_validator_.validateServoJ(joint_angles);
  if (!validation.success) {
    return this->rejectUnsafeTarget(
      validation, validation.message, lease_id);
  }

  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if ((this->state_ != State::STARTING && this->state_ != State::ACTIVE) ||
      this->lease_id_ != lease_id)
    {
      this->diagnostic_ = "ServoJ target does not match this session";
      return false;
    }
    const auto now = Clock::now();
    const auto watchdog_from = this->has_target_update_time_ ?
      this->target_updated_at_ : this->started_at_;
    if (now >= watchdog_from + this->options_.target_watchdog_timeout) {
      this->diagnostic_ = "ServoJ target arrived after the watchdog deadline";
      this->cv_.notify_all();
      return false;
    }
    this->target_ = joint_angles;
    this->has_target_ = true;
    this->target_updated_at_ = now;
    this->has_target_update_time_ = true;
    this->state_ = State::ACTIVE;
    this->diagnostic_ = "ServoJ target accepted";
  }
  this->cv_.notify_all();
  return true;
}

ServoControlSession::Result ServoControlSession::stop()
{
  LeaseId lease_id;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    lease_id = this->lease_id_;
  }
  auto result = this->performStop(StopCause::EXPLICIT, lease_id, "Explicit stop requested");
  bool worker_should_exit = false;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    worker_should_exit = this->worker_should_exit_;
  }
  // A stale Session is rejected before target admission is changed. Joining
  // in that case would block behind an active worker in this Session.
  if (worker_should_exit) {
    this->joinWorker();
  }
  return result;
}

ServoControlSession::Snapshot ServoControlSession::getSnapshot() const
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  return Snapshot{
    this->state_, this->lease_id_, this->stop_cause_, this->diagnostic_};
}

ServoControlSession::SafetyViolationState::SharedPtr
ServoControlSession::getSafetyViolationStateShared() const noexcept
{
  return this->safety_violation_state_;
}

ServoControlSession::OperationalErrorState::SharedPtr
ServoControlSession::getOperationalErrorStateShared() const noexcept
{
  return this->operational_error_state_;
}

void ServoControlSession::workerLoop()
{
  try {
    this->workerLoopImpl();
  } catch (const std::exception & error) {
    LeaseId lease_id;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      lease_id = this->lease_id_;
    }
    if (lease_id != ControlStateManager::NO_LEASE) {
      const std::string message = std::string("Unhandled Servo worker error: ") + error.what();
      this->reportOperationalError(
        ServoOperationalErrorCode::INTERNAL_ERROR,
        "Unhandled error in the Servo worker", message);
      static_cast<void>(this->performStop(
        StopCause::FAULT, lease_id, message));
    }
  } catch (...) {
    LeaseId lease_id;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      lease_id = this->lease_id_;
    }
    if (lease_id != ControlStateManager::NO_LEASE) {
      this->reportOperationalError(
        ServoOperationalErrorCode::INTERNAL_ERROR,
        "Unhandled unknown error in the Servo worker", "unknown exception");
      static_cast<void>(this->performStop(
        StopCause::FAULT, lease_id, "Unhandled unknown Servo worker error"));
    }
  }
}

void ServoControlSession::workerLoopImpl()
{
  while (true) {
    bool response_due = false;
    bool watchdog_due = false;
    bool send_due = false;
    bool safety_fault_due = false;
    LeaseId lease_id = ControlStateManager::NO_LEASE;

    {
      std::unique_lock<std::mutex> lock(this->mutex_);
      if (this->worker_should_exit_) {
        return;
      }

      safety_fault_due = this->safety_fault_requested_;
      if (!safety_fault_due) {
        const auto watchdog_from = this->has_target_update_time_ ?
          this->target_updated_at_ : this->started_at_;
        const auto watchdog_at = watchdog_from + this->options_.target_watchdog_timeout;
        const auto wake_at = std::min(
          std::min(this->next_send_at_, this->next_response_poll_at_), watchdog_at);
        this->cv_.wait_until(lock, wake_at);

        if (this->worker_should_exit_) {
          return;
        }

        safety_fault_due = this->safety_fault_requested_;
        const auto now = Clock::now();
        response_due = !safety_fault_due && now >= this->next_response_poll_at_;
        watchdog_due = !safety_fault_due && now >= watchdog_at;
        send_due = !safety_fault_due && now >= this->next_send_at_;
        if (response_due) {
          this->advanceResponseDeadlineLocked(now);
        }
      }
      lease_id = this->lease_id_;
    }

    if (safety_fault_due) {
      static_cast<void>(this->performStop(
        StopCause::FAULT, lease_id, "Servo safety violation requested a fault stop"));
      return;
    }

    if (response_due) {
      std::string fault_message;
      if (!this->monitorResponses(fault_message)) {
        static_cast<void>(this->performStop(
          StopCause::FAULT, lease_id, fault_message));
        return;
      }
      if (!this->control_state_manager_->acceptsServoJTarget(lease_id)) {
        const auto manager_snapshot = this->control_state_manager_->getSnapshot();
        if (manager_snapshot.connected) {
          this->reportOperationalError(
            ServoOperationalErrorCode::SERVO_LEASE_LOST,
            "The active Servo lease or allowed RobotMode was lost",
            "Periodic response monitoring found that the active lease no longer matches");
        }
        static_cast<void>(this->performStop(
          StopCause::FAULT, lease_id,
          "Servo lease or allowed RobotMode was lost during periodic control"));
        return;
      }
    }

    if (watchdog_due) {
      this->reportOperationalError(
        ServoOperationalErrorCode::WATCHDOG_TIMEOUT,
        "Servo target watchdog expired",
        "lease_id=" + std::to_string(lease_id));
      static_cast<void>(this->performStop(
        StopCause::WATCHDOG, lease_id, "Servo target watchdog expired"));
      return;
    }

    if (send_due && !this->sendLatestTarget(lease_id)) {
      std::string fault_message;
      {
        std::lock_guard<std::mutex> lock(this->mutex_);
        fault_message = this->diagnostic_;
      }
      static_cast<void>(this->performStop(StopCause::FAULT, lease_id, fault_message));
      return;
    }
  }
}

bool ServoControlSession::monitorResponses(std::string & fault_message)
{
  const auto dropped = this->motion_commander_->getDroppedCompletedResponseCount();
  const auto dropped_response_count = dropped >= this->response_drop_baseline_ ?
    dropped - this->response_drop_baseline_ : dropped;
  const bool queue_overflowed = dropped_response_count > 0;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (queue_overflowed) {
      fault_message = "Motion response queue overflowed during Servo control";
      this->diagnostic_ = fault_message;
    }
  }
  if (queue_overflowed) {
    this->reportOperationalError(
      ServoOperationalErrorCode::MOTION_RESPONSE_QUEUE_OVERFLOW,
      fault_message,
      "dropped_response_count=" + std::to_string(dropped_response_count));
    return false;
  }

  MotionResponse response;
  while (this->motion_commander_->tryTakeResponse(response)) {
    const bool relevant = response.command.empty() ?
      response.result != MotionResponseResult::SUCCESS :
      isServoJCommand(response.command);
    if (!relevant) {
      continue;
    }

    if (response.result != MotionResponseResult::SUCCESS) {
      fault_message = std::string("Servo motion response failure: ") +
        responseResultName(response.result);
      if (response.error_code != MotionResponse::ERROR_CODE_UNAVAILABLE) {
        fault_message += " (controller error " + std::to_string(response.error_code) + ")";
      }
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->diagnostic_ = fault_message;
    }
    if (response.result != MotionResponseResult::SUCCESS) {
      std::string detail = "result=" + std::string(responseResultName(response.result)) +
        "; sequence_id=" + std::to_string(response.sequence_id);
      if (response.error_code != MotionResponse::ERROR_CODE_UNAVAILABLE) {
        detail += "; controller_error_code=" + std::to_string(response.error_code);
      }
      this->reportOperationalError(
        responseErrorCode(response.result), fault_message, detail);
      return false;
    }
  }
  return true;
}

bool ServoControlSession::sendLatestTarget(const LeaseId lease_id)
{
  std::array<double, 4> target;
  std::array<double, 4> previous_target;
  bool has_target = false;
  bool has_previous_successful_command = false;
  Clock::time_point started_at;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (this->worker_should_exit_ ||
      (this->state_ != State::STARTING && this->state_ != State::ACTIVE))
    {
      return true;
    }
    has_target = this->has_target_;
    target = this->target_;
    has_previous_successful_command = this->has_previous_successful_command_;
    previous_target = this->previous_successful_target_;
    started_at = this->started_at_;
  }

  if (!has_target) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->advanceSendDeadlineLocked(Clock::now());
    return true;
  }

  const auto kinematics = this->kinematics_validator_.validateServoJ(target);
  if (!kinematics.success) {
    return this->rejectUnsafeSend(kinematics.code, kinematics.message, kinematics.detail);
  }

  constexpr double tolerance = ServoKinematicsValidator::FLOATING_POINT_TOLERANCE;
  if (!has_previous_successful_command) {
    const auto feedback = this->feedback_state_->getSnapshot();
    const auto now = Clock::now();
    const bool received_after_start = feedback.has_feedback && feedback.received_at >= started_at;
    if (!received_after_start ||
      !feedback.isFresh(this->connection_epoch_, this->options_.feedback_timeout, now))
    {
      std::ostringstream detail;
      detail << "Initial ServoJ target=" << arrayToString(target) << "; feedback_received=" <<
        feedback.has_feedback << "; expected_epoch=" << this->connection_epoch_ <<
        "; feedback_epoch=" << feedback.connection_epoch <<
        "; received_after_start=" << received_after_start <<
        "; timeout_ms=" <<
        std::chrono::duration<double, std::milli>(this->options_.feedback_timeout).count();
      if (feedback.has_feedback && now >= feedback.received_at) {
        detail << "; feedback_age_ms=" <<
          std::chrono::duration<double, std::milli>(now - feedback.received_at).count();
      }
      return this->rejectUnsafeSend(
        ServoSafetyViolationCode::REALTIME_FEEDBACK_UNAVAILABLE,
        "Realtime Feedback is unavailable or stale for the initial Servo command",
        detail.str());
    }

    const double feedback_age_ms =
      std::chrono::duration<double, std::milli>(now - feedback.received_at).count();
    std::array<double, 4> differences;
    for (std::size_t index = 0; index < 3; ++index) {
      differences[index] = std::abs(target[index] - feedback.joint_angles_rad[index]);
    }
    // The canonical MG400 constraint checker normalizes J4; J1-J3 remain
    // finite mechanical axes and therefore use an unwrapped difference.
    differences[3] = shortestAngleDistance(target[3], feedback.joint_angles_rad[3]);

    std::ostringstream violated_joints;
    bool discontinuous = false;
    for (std::size_t index = 0; index < differences.size(); ++index) {
      if (differences[index] > this->options_.max_initial_joint_distance_rad + tolerance) {
        if (discontinuous) {
          violated_joints << ',';
        }
        discontinuous = true;
        violated_joints << "J" << index + 1;
      }
    }
    if (discontinuous) {
      std::ostringstream detail;
      detail << "Initial ServoJ target_rad=" << arrayToString(target) <<
        "; current_rad=" << arrayToString(feedback.joint_angles_rad) <<
        "; difference_rad=" << arrayToString(differences) <<
        "; threshold_rad=" << this->options_.max_initial_joint_distance_rad <<
        "; violated_joints=" << violated_joints.str() <<
        "; feedback_age_ms=" << feedback_age_ms;
      return this->rejectUnsafeSend(
        ServoSafetyViolationCode::SERVO_J_COMMAND_DISCONTINUITY,
        "Initial ServoJ target is too far from the current joint position",
        detail.str());
    }
  } else {
    std::array<double, 4> differences;
    for (std::size_t index = 0; index < 3; ++index) {
      differences[index] = std::abs(target[index] - previous_target[index]);
    }
    differences[3] = shortestAngleDistance(target[3], previous_target[3]);

    std::ostringstream violated_joints;
    bool discontinuous = false;
    for (std::size_t index = 0; index < differences.size(); ++index) {
      if (differences[index] > this->options_.max_joint_step_rad + tolerance) {
        if (discontinuous) {
          violated_joints << ',';
        }
        discontinuous = true;
        violated_joints << "J" << index + 1;
      }
    }
    if (discontinuous) {
      std::ostringstream detail;
      detail << "ServoJ target_rad=" << arrayToString(target) <<
        "; previous_TCP_success_rad=" << arrayToString(previous_target) <<
        "; difference_rad=" << arrayToString(differences) <<
        "; threshold_rad=" << this->options_.max_joint_step_rad <<
        "; violated_joints=" << violated_joints.str();
      return this->rejectUnsafeSend(
        ServoSafetyViolationCode::SERVO_J_COMMAND_DISCONTINUITY,
        "ServoJ target exceeds the maximum change from the previous command",
        detail.str());
    }
  }

  // The lease is deliberately rechecked immediately before every TCP send;
  // an accepted callback value is not sufficient authority.
  if (!this->control_state_manager_->acceptsServoJTarget(lease_id)) {
    const std::string message = "Servo lease was rejected immediately before TCP send";
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->diagnostic_ = message;
    }
    const auto manager_snapshot = this->control_state_manager_->getSnapshot();
    if (manager_snapshot.connected) {
      this->reportOperationalError(
        ServoOperationalErrorCode::SERVO_LEASE_LOST, message,
        "lease_id=" + std::to_string(lease_id));
    }
    return false;
  }

  try {
    this->motion_commander_->servoJ(target[0], target[1], target[2], target[3]);
  } catch (const std::exception & error) {
    const std::string message = std::string("Servo TCP send failed: ") + error.what();
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->diagnostic_ = message;
    }
    this->reportOperationalError(
      ServoOperationalErrorCode::MOTION_TCP_SEND_FAILED,
      "Failed to send a Servo command on the Motion TCP connection",
      message + "; lease_id=" + std::to_string(lease_id));
    return false;
  } catch (...) {
    const std::string message = "Servo TCP send failed with an unknown exception";
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->diagnostic_ = message;
    }
    this->reportOperationalError(
      ServoOperationalErrorCode::MOTION_TCP_SEND_FAILED,
      "Failed to send a Servo command on the Motion TCP connection",
      message + "; lease_id=" + std::to_string(lease_id));
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->has_previous_successful_command_ = true;
    this->previous_successful_target_ = target;
    // Advance from the completion time so a delayed send cannot be followed by
    // a catch-up burst of overdue commands.
    this->advanceSendDeadlineLocked(Clock::now());
    this->diagnostic_ = "Servo target sent";
  }
  return true;
}

ServoControlSession::Result ServoControlSession::performStop(
  const StopCause cause, const LeaseId lease_id, const std::string & detail)
{
  bool expected = false;
  if (!this->stop_attempt_in_progress_.compare_exchange_strong(expected, true)) {
    return this->localResult(
      false, ControlStateManager::NO_LEASE, "Another Servo stop attempt is in progress");
  }
  AtomicFlagReset reset_stop_attempt(this->stop_attempt_in_progress_);

  StopCause effective_cause = cause;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (lease_id == ControlStateManager::NO_LEASE || lease_id != this->lease_id_ ||
      this->state_ == State::IDLE)
    {
      return Result{
        false, this->state_, ControlStateManager::NO_LEASE,
        "Servo stop lease does not match this session"};
    }
    if ((this->state_ == State::STOPPING || this->state_ == State::FAULTED) &&
      this->stop_cause_ != StopCause::NONE)
    {
      // A retry completes the original stop path.
      effective_cause = this->stop_cause_;
    }
  }

  auto manager_snapshot = this->control_state_manager_->getSnapshot();
  if (manager_snapshot.lease_id != lease_id ||
    manager_snapshot.control_state != ControlStateManager::State::SERVO_J)
  {
    const std::string message = "Servo ownership changed before the stop strategy";
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->state_ = State::FAULTED;
      this->worker_should_exit_ = true;
      this->stop_cause_ = effective_cause;
      this->diagnostic_ = message;
    }
    this->cv_.notify_all();
    if (manager_snapshot.connected) {
      this->reportOperationalError(
        ServoOperationalErrorCode::SERVO_LEASE_LOST, message,
        "lease_id=" + std::to_string(lease_id));
    }
    return this->localResult(
      false, ControlStateManager::NO_LEASE, message);
  }

  if (manager_snapshot.accepting_servo_targets) {
    const auto begin_stop = this->control_state_manager_->beginServoStop(lease_id);
    if (!begin_stop.success) {
      const auto failed_snapshot = this->control_state_manager_->getSnapshot();
      if (failed_snapshot.connected) {
        this->reportOperationalError(
          ServoOperationalErrorCode::SERVO_LEASE_LOST,
          "The active Servo lease was lost while beginning the stop",
          begin_stop.message + "; lease_id=" + std::to_string(lease_id));
      }
      return this->localResult(false, lease_id, begin_stop.message);
    }
  }

  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->state_ = State::STOPPING;
    this->has_target_ = false;
    this->worker_should_exit_ = true;
    this->stop_cause_ = effective_cause;
    this->diagnostic_ = detail;
  }
  this->cv_.notify_all();

  // Revalidate after target admission has been closed. No session or manager
  // mutex is held while the replaceable stop strategy calls ResetRobot().
  if (!this->currentLeaseMatches(lease_id)) {
    const std::string message = "Servo ownership changed before ResetRobot";
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->state_ = State::FAULTED;
      this->diagnostic_ = message;
    }
    const auto failed_snapshot = this->control_state_manager_->getSnapshot();
    if (failed_snapshot.connected) {
      this->reportOperationalError(
        ServoOperationalErrorCode::SERVO_LEASE_LOST, message,
        "lease_id=" + std::to_string(lease_id));
    }
    return this->localResult(false, ControlStateManager::NO_LEASE, message);
  }

  ServoStopStrategy::Result stop_result;
  try {
    stop_result = this->stop_strategy_->stop(
      Clock::now() + this->options_.stop_confirmation_timeout);
  } catch (const std::exception & error) {
    stop_result = ServoStopStrategy::Result{
      ServoStopStrategy::Status::RESET_FAILED,
      std::string("Servo stop strategy threw: ") + error.what()};
  } catch (...) {
    stop_result = ServoStopStrategy::Result{
      ServoStopStrategy::Status::RESET_FAILED,
      "Servo stop strategy threw an unknown exception"};
  }

  // A stale completion must never release a newer lease obtained after a
  // disconnect/reconnect or by another Session instance.
  if (!this->currentLeaseMatches(lease_id)) {
    const std::string message =
      "Servo ownership changed while the stop strategy was running";
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->state_ = State::FAULTED;
      this->diagnostic_ = message;
    }
    const auto failed_snapshot = this->control_state_manager_->getSnapshot();
    if (failed_snapshot.connected) {
      this->reportOperationalError(
        ServoOperationalErrorCode::SERVO_LEASE_LOST, message,
        "lease_id=" + std::to_string(lease_id));
    }
    return this->localResult(false, ControlStateManager::NO_LEASE, message);
  }

  if (!stop_result.success()) {
    const auto code = stop_result.status == ServoStopStrategy::Status::CONFIRMATION_TIMEOUT ?
      ServoOperationalErrorCode::STOP_CONFIRMATION_TIMEOUT :
      ServoOperationalErrorCode::STOP_RESET_FAILED;
    this->reportOperationalError(
      code, stop_result.message,
      std::string("stop_cause=") +
      (effective_cause == StopCause::WATCHDOG ? "WATCHDOG" :
      effective_cause == StopCause::FAULT ? "FAULT" : "EXPLICIT") +
      "; lease_id=" + std::to_string(lease_id));
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->state_ = State::FAULTED;
      this->diagnostic_ = stop_result.message;
    }
    return this->localResult(false, lease_id, stop_result.message);
  }

  const auto completed = this->control_state_manager_->release(
    ControlStateManager::State::SERVO_J, lease_id);
  if (!completed.success) {
    const auto failed_snapshot = this->control_state_manager_->getSnapshot();
    if (failed_snapshot.connected) {
      const bool ownership_changed = failed_snapshot.lease_id != lease_id ||
        failed_snapshot.control_state != ControlStateManager::State::SERVO_J;
      this->reportOperationalError(
        ownership_changed ? ServoOperationalErrorCode::SERVO_LEASE_LOST :
        ServoOperationalErrorCode::INTERNAL_ERROR,
        ownership_changed ? "Servo ownership changed before ownership release completed" :
        "The Servo stop completed but ownership release failed",
        completed.message + "; lease_id=" + std::to_string(lease_id));
    }
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->state_ = State::FAULTED;
      this->diagnostic_ = completed.message;
    }
    return this->localResult(false, lease_id, completed.message);
  }

  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->state_ = State::IDLE;
    this->lease_id_ = ControlStateManager::NO_LEASE;
    this->has_target_ = false;
    this->has_target_update_time_ = false;
    this->diagnostic_ = stop_result.message;
  }
  return this->localResult(
    true, ControlStateManager::NO_LEASE, stop_result.message);
}

void ServoControlSession::joinWorker()
{
  std::lock_guard<std::mutex> worker_lock(this->mutex_worker_);
  if (this->worker_.joinable() && this->worker_.get_id() != std::this_thread::get_id()) {
    this->worker_.join();
  }
}

void ServoControlSession::requestWorkerExit()
{
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->worker_should_exit_ = true;
  }
  this->cv_.notify_all();
}

void ServoControlSession::advanceSendDeadlineLocked(const Clock::time_point & now)
{
  do {
    this->next_send_at_ += this->options_.send_period;
  } while (this->next_send_at_ <= now);
}

void ServoControlSession::advanceResponseDeadlineLocked(const Clock::time_point & now)
{
  do {
    this->next_response_poll_at_ += this->options_.response_poll_period;
  } while (this->next_response_poll_at_ <= now);
}

bool ServoControlSession::currentLeaseMatches(const LeaseId lease_id) const
{
  return this->control_state_manager_->owns(
    ControlStateManager::State::SERVO_J, lease_id);
}

bool ServoControlSession::targetMatchesSession(const LeaseId lease_id)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  return (this->state_ == State::STARTING || this->state_ == State::ACTIVE) &&
         this->lease_id_ == lease_id && !this->safety_fault_requested_;
}

bool ServoControlSession::rejectUnsafeTarget(
  const ServoKinematicsValidator::Result & violation,
  const std::string & diagnostic,
  const LeaseId lease_id)
{
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if ((this->state_ != State::STARTING && this->state_ != State::ACTIVE) ||
      this->lease_id_ != lease_id || this->safety_fault_requested_)
    {
      this->diagnostic_ = "Unsafe target no longer matches the active Servo session";
      return false;
    }
    this->safety_fault_requested_ = true;
    this->diagnostic_ = diagnostic;
  }
  this->reportSafetyViolation(violation.code, violation.message, violation.detail);
  this->cv_.notify_all();
  return false;
}

bool ServoControlSession::rejectUnsafeSend(
  const ServoSafetyViolationCode code,
  const std::string & message,
  const std::string & detail)
{
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->diagnostic_ = message;
  }
  this->reportSafetyViolation(code, message, detail);
  return false;
}

void ServoControlSession::reportSafetyViolation(
  const ServoSafetyViolationCode code,
  const std::string & message,
  const std::string & detail)
{
  // reportViolation() can publish diagnostics through its change callback, so
  // no Session/feedback/safety mutex is held by this caller.
  const bool first_fault = this->safety_violation_state_->reportViolation(code, message);
  if (!first_fault || !this->options_.safety_log_callback) {
    return;
  }

  std::string log_message = std::string(toString(code)) + ": " + message;
  if (!detail.empty()) {
    log_message += "; " + detail;
  }
  try {
    this->options_.safety_log_callback(log_message);
  } catch (...) {
    // Logging must not prevent the already-latched violation from stopping Servo.
  }
}

void ServoControlSession::reportOperationalError(
  const ServoOperationalErrorCode code,
  const std::string & message,
  const std::string & detail)
{
  // reportError() can publish through its change callback, so no Session or
  // operational-state mutex is held by this caller.
  const bool first_fault = this->operational_error_state_->reportError(code, message);
  const bool important_followup =
    code == ServoOperationalErrorCode::STOP_RESET_FAILED ||
    code == ServoOperationalErrorCode::STOP_CONFIRMATION_TIMEOUT ||
    code == ServoOperationalErrorCode::SERVO_LEASE_LOST ||
    code == ServoOperationalErrorCode::INTERNAL_ERROR;
  if ((!first_fault && !important_followup) || !this->options_.operational_log_callback) {
    return;
  }

  std::string log_message = std::string(toString(code)) + ": " + message;
  if (!detail.empty()) {
    log_message += "; " + detail;
  }
  try {
    this->options_.operational_log_callback(code, log_message);
  } catch (...) {
    // Logging must not prevent an operational fault from following its
    // existing stop path.
  }
}

bool ServoControlSession::rejectTarget(const std::string & reason)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->diagnostic_ = reason;
  return false;
}

ServoControlSession::Result ServoControlSession::localResult(
  const bool success, const LeaseId lease_id, const std::string & message) const
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  return Result{success, this->state_, lease_id, message};
}

bool ServoControlSession::isFinite(const std::array<double, 4> & values) noexcept
{
  return std::all_of(
    values.begin(), values.end(), [](const double value) {return std::isfinite(value);});
}

const char * ServoControlSession::responseResultName(
  const MotionResponseResult result) noexcept
{
  switch (result) {
    case MotionResponseResult::SUCCESS:
      return "success";
    case MotionResponseResult::CONTROLLER_ERROR:
      return "controller error";
    case MotionResponseResult::TIMEOUT:
      return "timeout";
    case MotionResponseResult::DISCONNECTED:
      return "disconnect";
    case MotionResponseResult::PARSE_ERROR:
      return "parse error";
    default:
      return "unknown response result";
  }
}

ServoOperationalErrorCode ServoControlSession::responseErrorCode(
  const MotionResponseResult result) noexcept
{
  switch (result) {
    case MotionResponseResult::CONTROLLER_ERROR:
      return ServoOperationalErrorCode::MOTION_RESPONSE_CONTROLLER_ERROR;
    case MotionResponseResult::TIMEOUT:
      return ServoOperationalErrorCode::MOTION_RESPONSE_TIMEOUT;
    case MotionResponseResult::DISCONNECTED:
      return ServoOperationalErrorCode::MOTION_RESPONSE_DISCONNECTED;
    case MotionResponseResult::PARSE_ERROR:
      return ServoOperationalErrorCode::MOTION_RESPONSE_PARSE_ERROR;
    case MotionResponseResult::SUCCESS:
    default:
      return ServoOperationalErrorCode::INTERNAL_ERROR;
  }
}

}  // namespace mg400_interface
