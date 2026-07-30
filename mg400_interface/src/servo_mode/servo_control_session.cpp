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

#include "mg400_interface/servo_mode/servo_control_session.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <utility>

#include <mg400_common/kinematics.hpp>

namespace mg400_interface
{
namespace
{

constexpr double FLOATING_POINT_TOLERANCE = 1.0e-12;

ServoSafetyViolation validateServoJ(const std::array<double, 4> & joint_angles)
{
  const bool finite = std::all_of(
    joint_angles.begin(), joint_angles.end(),
    [](const double value) {return std::isfinite(value);});
  if (!finite) {
    return ServoSafetyViolation{
      ServoSafetyViolationCode::SERVO_J_JOINT_LIMIT,
      "ServoJ target contains a non-finite value"};
  }

  Eigen::Vector4d joints;
  joints << joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3];

  // Separate unconditional ranges from the conditional J2 and J3-J2
  // constraints so the reported code identifies the root cause.
  mg400_common::kinematics::ConstraintOptions individual_options;
  individual_options.j2_min_no_collision = individual_options.j2_min;
  individual_options.j3_1_min = -std::numeric_limits<double>::max();
  individual_options.j3_1_max = std::numeric_limits<double>::max();
  if (!mg400_common::kinematics::check_constraints(joints, individual_options).allValid()) {
    return ServoSafetyViolation{
      ServoSafetyViolationCode::SERVO_J_JOINT_LIMIT,
      "ServoJ target violates an MG400 joint limit"};
  }

  if (!mg400_common::kinematics::check_constraints(joints).allValid()) {
    return ServoSafetyViolation{
      ServoSafetyViolationCode::SERVO_J_COUPLED_LIMIT,
      "ServoJ target violates an MG400 coupled-joint constraint"};
  }

  return ServoSafetyViolation{};
}

class AtomicFlagReset
{
public:
  explicit AtomicFlagReset(std::atomic<bool> & flag)
  : flag_(flag) {}

  ~AtomicFlagReset() {this->flag_.store(false);}

private:
  std::atomic<bool> & flag_;
};

double shortestAngleDistance(const double lhs, const double rhs)
{
  return std::abs(std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs)));
}

}  // namespace

ServoControlSession::ServoControlSession(
  ControlStateManager::SharedPtr control_state_manager,
  MotionCommander::SharedPtr motion_commander,
  ServoStopStrategy::SharedPtr stop_strategy,
  SafetyViolationState::SharedPtr safety_violation_state,
  OperationalErrorState::SharedPtr operational_error_state,
  FeedbackReader feedback_reader,
  const Options & options)
: control_state_manager_(std::move(control_state_manager)),
  motion_commander_(std::move(motion_commander)),
  stop_strategy_(std::move(stop_strategy)),
  safety_violation_state_(std::move(safety_violation_state)),
  operational_error_state_(std::move(operational_error_state)),
  feedback_reader_(std::move(feedback_reader)),
  connection_epoch_(0),
  options_(options),
  state_(State::IDLE),
  lease_id_(ControlStateManager::NO_LEASE),
  has_target_(false),
  target_{{0.0, 0.0, 0.0, 0.0}},
  has_target_update_time_(false),
  worker_should_exit_(true),
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
  if (!this->feedback_reader_) {
    throw std::invalid_argument("ServoControlSession requires a Realtime Feedback reader");
  }
  this->connection_epoch_ = this->feedback_reader_().connection_epoch;
  if (this->connection_epoch_ == 0) {
    throw std::invalid_argument("ServoControlSession requires an active feedback connection epoch");
  }
  if (this->options_.send_period <= std::chrono::nanoseconds::zero() ||
    this->options_.target_watchdog_timeout <= std::chrono::nanoseconds::zero() ||
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
      static_cast<void>(this->performStop(snapshot.lease_id));
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
        false, ControlStateManager::NO_LEASE, "Servo session is not IDLE"};
    }
    // Reserve the local transition while the manager performs the atomic
    // ownership acquisition.
    this->state_ = State::ACTIVE;
  }

  this->joinWorker();
  const auto acquisition = this->control_state_manager_->tryAcquire(
    ControlStateManager::State::SERVO_J);
  if (!acquisition.success) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->state_ = State::IDLE;
    return Result{false, ControlStateManager::NO_LEASE, acquisition.message};
  }

  const auto now = Clock::now();
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->state_ = State::ACTIVE;
    this->lease_id_ = acquisition.lease_id;
    this->has_target_ = false;
    this->started_at_ = now;
    this->has_target_update_time_ = false;
    this->next_send_at_ = now + this->options_.send_period;
    this->worker_should_exit_ = false;
    this->has_previous_successful_command_ = false;
    this->safety_fault_requested_ = false;
  }

  try {
    std::lock_guard<std::mutex> worker_lock(this->mutex_worker_);
    this->worker_ = std::thread(&ServoControlSession::workerLoop, this);
  } catch (const std::exception & error) {
    const std::string message = std::string("Failed to start Servo worker: ") + error.what();
    this->reportOperationalError(
      ServoOperationalErrorCode::INTERNAL_ERROR, "Failed to start the Servo worker");
    const auto stopped = this->performStop(acquisition.lease_id);
    return Result{false, stopped.lease_id, message + "; " + stopped.message};
  }

  // A fully successful explicit start is the only Servo error rearm point.
  this->safety_violation_state_->clearForRearm();
  this->operational_error_state_->clearForRearm();
  return Result{true, acquisition.lease_id, "Servo control started"};
}

bool ServoControlSession::updateServoJTarget(
  const LeaseId lease_id, const std::array<double, 4> & joint_angles)
{
  if (!this->control_state_manager_->acceptsServoJTarget(lease_id)) {
    return false;
  }

  const auto violation = validateServoJ(joint_angles);
  if (violation.code != ServoSafetyViolationCode::NONE) {
    return this->rejectUnsafeTarget(violation, lease_id);
  }

  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (this->state_ != State::ACTIVE || this->lease_id_ != lease_id ||
      this->safety_fault_requested_)
    {
      return false;
    }
    const auto now = Clock::now();
    const auto watchdog_from = this->has_target_update_time_ ?
      this->target_updated_at_ : this->started_at_;
    if (now >= watchdog_from + this->options_.target_watchdog_timeout) {
      this->cv_.notify_all();
      return false;
    }
    this->target_ = joint_angles;
    this->has_target_ = true;
    this->target_updated_at_ = now;
    this->has_target_update_time_ = true;
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
  auto result = this->performStop(lease_id);
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
  return Snapshot{this->state_, this->lease_id_};
}

void ServoControlSession::workerLoop()
{
  try {
    this->workerLoopImpl();
  } catch (const std::exception &) {
    LeaseId lease_id;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      lease_id = this->lease_id_;
    }
    if (lease_id != ControlStateManager::NO_LEASE) {
      this->reportOperationalError(
        ServoOperationalErrorCode::INTERNAL_ERROR, "Unhandled error in the Servo worker");
      static_cast<void>(this->performStop(lease_id));
    }
  } catch (...) {
    LeaseId lease_id;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      lease_id = this->lease_id_;
    }
    if (lease_id != ControlStateManager::NO_LEASE) {
      this->reportOperationalError(
        ServoOperationalErrorCode::INTERNAL_ERROR, "Unhandled unknown error in the Servo worker");
      static_cast<void>(this->performStop(lease_id));
    }
  }
}

void ServoControlSession::workerLoopImpl()
{
  while (true) {
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
        const auto wake_at = std::min(this->next_send_at_, watchdog_at);
        this->cv_.wait_until(lock, wake_at);

        if (this->worker_should_exit_) {
          return;
        }

        safety_fault_due = this->safety_fault_requested_;
        const auto now = Clock::now();
        watchdog_due = !safety_fault_due && now >= watchdog_at;
        send_due = !safety_fault_due && now >= this->next_send_at_;
      }
      lease_id = this->lease_id_;
    }

    if (safety_fault_due) {
      static_cast<void>(this->performStop(lease_id));
      return;
    }

    if (watchdog_due) {
      this->reportOperationalError(
        ServoOperationalErrorCode::WATCHDOG_TIMEOUT, "Servo target watchdog expired");
      static_cast<void>(this->performStop(lease_id));
      return;
    }

    if (send_due) {
      if (!this->sendLatestTarget(lease_id)) {
        static_cast<void>(this->performStop(lease_id));
        return;
      }
    }
  }
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
      this->state_ != State::ACTIVE)
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
    if (!this->activeLeaseAcceptsTarget(lease_id)) {
      return false;
    }
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->advanceSendDeadlineLocked(Clock::now());
    return true;
  }

  if (!has_previous_successful_command) {
    const auto feedback = this->feedback_reader_();
    const auto now = Clock::now();
    const bool received_after_start = feedback.has_data && feedback.received_at >= started_at;
    if (!received_after_start ||
      !feedback.isFresh(this->connection_epoch_, this->options_.feedback_timeout, now))
    {
      return this->rejectUnsafeSend(
        ServoSafetyViolationCode::REALTIME_FEEDBACK_UNAVAILABLE,
        "Realtime Feedback is unavailable or stale for the initial Servo command");
    }

    const auto current_joints = feedback.jointAnglesRad();
    const bool finite_feedback = std::all_of(
      current_joints.begin(), current_joints.end(),
      [](const double value) {return std::isfinite(value);});
    if (!finite_feedback) {
      return this->rejectUnsafeSend(
        ServoSafetyViolationCode::REALTIME_FEEDBACK_UNAVAILABLE,
        "Realtime Feedback contains a non-finite joint position");
    }

    std::array<double, 4> differences;
    for (std::size_t index = 0; index < 3; ++index) {
      differences[index] = std::abs(target[index] - current_joints[index]);
    }
    // The canonical MG400 constraint checker normalizes J4; J1-J3 remain
    // finite mechanical axes and therefore use an unwrapped difference.
    differences[3] = shortestAngleDistance(target[3], current_joints[3]);

    bool discontinuous = false;
    for (double difference : differences) {
      if (difference >
        this->options_.max_initial_joint_distance_rad + FLOATING_POINT_TOLERANCE)
      {
        discontinuous = true;
      }
    }
    if (discontinuous) {
      return this->rejectUnsafeSend(
        ServoSafetyViolationCode::SERVO_J_COMMAND_DISCONTINUITY,
        "Initial ServoJ target is too far from the current joint position");
    }
  } else {
    std::array<double, 4> differences;
    for (std::size_t index = 0; index < 3; ++index) {
      differences[index] = std::abs(target[index] - previous_target[index]);
    }
    differences[3] = shortestAngleDistance(target[3], previous_target[3]);

    bool discontinuous = false;
    for (double difference : differences) {
      if (difference > this->options_.max_joint_step_rad + FLOATING_POINT_TOLERANCE) {
        discontinuous = true;
      }
    }
    if (discontinuous) {
      return this->rejectUnsafeSend(
        ServoSafetyViolationCode::SERVO_J_COMMAND_DISCONTINUITY,
        "ServoJ target exceeds the maximum change from the previous command");
    }
  }

  // An accepted callback value is not sufficient authority. Recheck after the
  // safety calculations and immediately before the TCP operation.
  if (!this->activeLeaseAcceptsTarget(lease_id)) {
    return false;
  }

  try {
    this->motion_commander_->servoJ(target[0], target[1], target[2], target[3]);
  } catch (const std::exception & error) {
    this->reportOperationalError(
      ServoOperationalErrorCode::MOTION_TCP_SEND_FAILED,
      std::string("Failed to send a Servo command on the Motion TCP connection: ") + error.what());
    return false;
  } catch (...) {
    this->reportOperationalError(
      ServoOperationalErrorCode::MOTION_TCP_SEND_FAILED,
      "Failed to send a Servo command on the Motion TCP connection");
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->has_previous_successful_command_ = true;
    this->previous_successful_target_ = target;
    // Advance from the completion time so a delayed send cannot be followed by
    // a catch-up burst of overdue commands.
    this->advanceSendDeadlineLocked(Clock::now());
  }
  return true;
}

bool ServoControlSession::activeLeaseAcceptsTarget(const LeaseId lease_id)
{
  if (this->control_state_manager_->acceptsServoJTarget(lease_id)) {
    return true;
  }
  const auto manager_snapshot = this->control_state_manager_->getSnapshot();
  if (manager_snapshot.connected) {
    this->reportOperationalError(
      ServoOperationalErrorCode::SERVO_LEASE_LOST,
      "The active Servo lease or allowed RobotMode was lost");
  }
  return false;
}

ServoControlSession::Result ServoControlSession::performStop(const LeaseId lease_id)
{
  bool expected = false;
  if (!this->stop_attempt_in_progress_.compare_exchange_strong(expected, true)) {
    return Result{
      false, ControlStateManager::NO_LEASE, "Another Servo stop attempt is in progress"};
  }
  AtomicFlagReset reset_stop_attempt(this->stop_attempt_in_progress_);

  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (lease_id == ControlStateManager::NO_LEASE || lease_id != this->lease_id_ ||
      this->state_ == State::IDLE)
    {
      return Result{
        false, ControlStateManager::NO_LEASE, "Servo stop lease does not match this session"};
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
    }
    this->cv_.notify_all();
    if (manager_snapshot.connected) {
      this->reportOperationalError(
        ServoOperationalErrorCode::SERVO_LEASE_LOST, message);
    }
    return Result{false, ControlStateManager::NO_LEASE, message};
  }

  if (manager_snapshot.accepting_servo_targets) {
    const auto begin_stop = this->control_state_manager_->beginServoStop(lease_id);
    if (!begin_stop.success) {
      const auto failed_snapshot = this->control_state_manager_->getSnapshot();
      if (failed_snapshot.connected) {
        this->reportOperationalError(
          ServoOperationalErrorCode::SERVO_LEASE_LOST,
          "The active Servo lease was lost while beginning the stop");
      }
      return Result{false, lease_id, begin_stop.message};
    }
  }

  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->state_ = State::STOPPING;
    this->has_target_ = false;
    this->worker_should_exit_ = true;
  }
  this->cv_.notify_all();

  // Revalidate after target admission has been closed. No session or manager
  // mutex is held while the replaceable stop strategy calls ResetRobot().
  if (!this->currentLeaseMatches(lease_id)) {
    const std::string message = "Servo ownership changed before ResetRobot";
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->state_ = State::FAULTED;
    }
    const auto failed_snapshot = this->control_state_manager_->getSnapshot();
    if (failed_snapshot.connected) {
      this->reportOperationalError(
        ServoOperationalErrorCode::SERVO_LEASE_LOST, message);
    }
    return Result{false, ControlStateManager::NO_LEASE, message};
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
    }
    const auto failed_snapshot = this->control_state_manager_->getSnapshot();
    if (failed_snapshot.connected) {
      this->reportOperationalError(
        ServoOperationalErrorCode::SERVO_LEASE_LOST, message);
    }
    return Result{false, ControlStateManager::NO_LEASE, message};
  }

  if (!stop_result.success()) {
    const auto code = stop_result.status == ServoStopStrategy::Status::CONFIRMATION_TIMEOUT ?
      ServoOperationalErrorCode::STOP_CONFIRMATION_TIMEOUT :
      ServoOperationalErrorCode::STOP_RESET_FAILED;
    this->reportOperationalError(code, stop_result.message);
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->state_ = State::FAULTED;
    }
    return Result{false, lease_id, stop_result.message};
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
        "The Servo stop completed but ownership release failed");
    }
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->state_ = State::FAULTED;
    }
    return Result{false, lease_id, completed.message};
  }

  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->state_ = State::IDLE;
    this->lease_id_ = ControlStateManager::NO_LEASE;
    this->has_target_ = false;
    this->has_target_update_time_ = false;
  }
  return Result{true, ControlStateManager::NO_LEASE, stop_result.message};
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

bool ServoControlSession::currentLeaseMatches(const LeaseId lease_id) const
{
  return this->control_state_manager_->owns(
    ControlStateManager::State::SERVO_J, lease_id);
}

bool ServoControlSession::rejectUnsafeTarget(
  const ServoSafetyViolation & violation,
  const LeaseId lease_id)
{
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (this->state_ != State::ACTIVE || this->lease_id_ != lease_id ||
      this->safety_fault_requested_)
    {
      return false;
    }
    this->safety_fault_requested_ = true;
  }
  this->safety_violation_state_->reportViolation(violation.code, violation.message);
  this->cv_.notify_all();
  return false;
}

bool ServoControlSession::rejectUnsafeSend(
  const ServoSafetyViolationCode code,
  const std::string & message)
{
  this->safety_violation_state_->reportViolation(code, message);
  return false;
}

void ServoControlSession::reportOperationalError(
  const ServoOperationalErrorCode code,
  const std::string & message)
{
  this->operational_error_state_->reportError(code, message);
}

}  // namespace mg400_interface
