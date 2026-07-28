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

ControlStateManager::MotionOwner ownerForServoType(
  const ControlStateManager::State servo_type)
{
  return servo_type == ControlStateManager::State::SERVO_J ?
         ControlStateManager::MotionOwner::SERVO_J :
         ControlStateManager::MotionOwner::SERVO_P;
}

bool isCommandForServoType(
  const std::string & command, const ControlStateManager::State servo_type)
{
  const char * prefix = servo_type == ControlStateManager::State::SERVO_J ?
    "ServoJ(" : "ServoP(";
  return command.compare(0, std::char_traits<char>::length(prefix), prefix) == 0;
}

}  // namespace

ServoControlSession::ServoControlSession(
  ControlStateManager::SharedPtr control_state_manager,
  MotionCommander::SharedPtr motion_commander,
  ServoStopStrategy::SharedPtr stop_strategy)
: ServoControlSession(
    std::move(control_state_manager), std::move(motion_commander),
    std::move(stop_strategy), Options{})
{
}

ServoControlSession::ServoControlSession(
  ControlStateManager::SharedPtr control_state_manager,
  MotionCommander::SharedPtr motion_commander,
  ServoStopStrategy::SharedPtr stop_strategy,
  const Options & options)
: control_state_manager_(std::move(control_state_manager)),
  motion_commander_(std::move(motion_commander)),
  stop_strategy_(std::move(stop_strategy)),
  options_(options),
  state_(State::IDLE),
  servo_type_(ServoType::IDLE),
  lease_id_(ControlStateManager::NO_LEASE),
  has_target_(false),
  target_{{0.0, 0.0, 0.0, 0.0}},
  has_target_update_time_(false),
  worker_should_exit_(true),
  watchdog_triggered_(false),
  stop_cause_(StopCause::NONE),
  accepted_target_count_(0),
  rejected_target_count_(0),
  sent_command_count_(0),
  has_last_send_time_(false),
  has_latest_response_(false),
  response_drop_baseline_(0),
  dropped_response_count_(0),
  diagnostic_("Servo session is idle"),
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
  if (this->options_.send_period <= std::chrono::nanoseconds::zero() ||
    this->options_.target_watchdog_timeout <= std::chrono::nanoseconds::zero() ||
    this->options_.response_poll_period <= std::chrono::nanoseconds::zero() ||
    this->options_.stop_confirmation_timeout <= std::chrono::nanoseconds::zero())
  {
    throw std::invalid_argument("ServoControlSession durations must be positive");
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

ServoControlSession::Result ServoControlSession::start(const ServoType servo_type)
{
  if (!isServoType(servo_type)) {
    return this->localResult(
      false, ControlStateManager::NO_LEASE, "Only SERVO_J or SERVO_P can be started");
  }

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
  const auto acquisition = this->control_state_manager_->requestControlState(
    servo_type, ControlStateManager::NO_LEASE);
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
    this->servo_type_ = servo_type;
    this->lease_id_ = acquisition.lease_id;
    this->has_target_ = false;
    this->started_at_ = now;
    this->has_target_update_time_ = false;
    this->next_send_at_ = now + this->options_.send_period;
    this->next_response_poll_at_ = now + this->options_.response_poll_period;
    this->worker_should_exit_ = false;
    this->watchdog_triggered_ = false;
    this->stop_cause_ = StopCause::NONE;
    this->accepted_target_count_ = 0;
    this->rejected_target_count_ = 0;
    this->sent_command_count_ = 0;
    this->has_last_send_time_ = false;
    this->has_latest_response_ = false;
    this->response_drop_baseline_ =
      this->motion_commander_->getDroppedCompletedResponseCount();
    this->dropped_response_count_ = 0;
    this->diagnostic_ = "Servo lease acquired; waiting for the first target";
  }

  try {
    std::lock_guard<std::mutex> worker_lock(this->mutex_worker_);
    this->worker_ = std::thread(&ServoControlSession::workerLoop, this);
  } catch (const std::exception & error) {
    const std::string message = std::string("Failed to start Servo worker: ") + error.what();
    const auto stopped = this->performStop(StopCause::FAULT, acquisition.lease_id, message);
    return Result{false, stopped.state, stopped.lease_id, message + "; " + stopped.message};
  }

  return this->localResult(true, acquisition.lease_id, "Servo control started");
}

bool ServoControlSession::updateServoJTarget(
  const LeaseId lease_id, const std::array<double, 4> & joint_angles)
{
  if (!isFinite(joint_angles)) {
    return this->rejectTarget("ServoJ target contains a non-finite value");
  }
  if (!this->control_state_manager_->acceptsServoTarget(ServoType::SERVO_J, lease_id)) {
    return this->rejectTarget("ServoJ target has a foreign, stale, or inactive lease");
  }

  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if ((this->state_ != State::STARTING && this->state_ != State::ACTIVE) ||
      this->servo_type_ != ServoType::SERVO_J || this->lease_id_ != lease_id)
    {
      ++this->rejected_target_count_;
      this->diagnostic_ = "ServoJ target does not match this session";
      return false;
    }
    const auto now = Clock::now();
    const auto watchdog_from = this->has_target_update_time_ ?
      this->target_updated_at_ : this->started_at_;
    if (now >= watchdog_from + this->options_.target_watchdog_timeout) {
      ++this->rejected_target_count_;
      this->watchdog_triggered_ = true;
      this->diagnostic_ = "ServoJ target arrived after the watchdog deadline";
      this->cv_.notify_all();
      return false;
    }
    this->target_ = joint_angles;
    this->has_target_ = true;
    this->target_updated_at_ = now;
    this->has_target_update_time_ = true;
    this->state_ = State::ACTIVE;
    ++this->accepted_target_count_;
    this->diagnostic_ = "ServoJ target accepted";
  }
  this->cv_.notify_all();
  return true;
}

bool ServoControlSession::updateServoPTarget(
  const LeaseId lease_id, const double x, const double y, const double z, const double yaw)
{
  const std::array<double, 4> target{{x, y, z, yaw}};
  if (!isFinite(target)) {
    return this->rejectTarget("ServoP target contains a non-finite value");
  }
  if (!this->control_state_manager_->acceptsServoTarget(ServoType::SERVO_P, lease_id)) {
    return this->rejectTarget("ServoP target has a foreign, stale, or inactive lease");
  }

  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if ((this->state_ != State::STARTING && this->state_ != State::ACTIVE) ||
      this->servo_type_ != ServoType::SERVO_P || this->lease_id_ != lease_id)
    {
      ++this->rejected_target_count_;
      this->diagnostic_ = "ServoP target does not match this session";
      return false;
    }
    const auto now = Clock::now();
    const auto watchdog_from = this->has_target_update_time_ ?
      this->target_updated_at_ : this->started_at_;
    if (now >= watchdog_from + this->options_.target_watchdog_timeout) {
      ++this->rejected_target_count_;
      this->watchdog_triggered_ = true;
      this->diagnostic_ = "ServoP target arrived after the watchdog deadline";
      this->cv_.notify_all();
      return false;
    }
    this->target_ = target;
    this->has_target_ = true;
    this->target_updated_at_ = now;
    this->has_target_update_time_ = true;
    this->state_ = State::ACTIVE;
    ++this->accepted_target_count_;
    this->diagnostic_ = "ServoP target accepted";
  }
  this->cv_.notify_all();
  return true;
}

ServoControlSession::Result ServoControlSession::stop(const LeaseId lease_id)
{
  auto result = this->performStop(StopCause::EXPLICIT, lease_id, "Explicit stop requested");
  bool worker_should_exit = false;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    worker_should_exit = this->worker_should_exit_;
  }
  // A foreign/stale lease is rejected before target admission is changed.
  // Joining in that case would block behind the still-active worker until its
  // watchdog expires and could indirectly stop the valid owner.
  if (worker_should_exit) {
    this->joinWorker();
  }
  return result;
}

ServoControlSession::Snapshot ServoControlSession::getSnapshot() const
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  return Snapshot{
    this->state_, this->servo_type_, this->lease_id_, this->has_target_,
    this->has_target_update_time_, this->target_updated_at_, this->watchdog_triggered_,
    this->stop_cause_, this->accepted_target_count_, this->rejected_target_count_,
    this->sent_command_count_, this->has_last_send_time_, this->last_sent_at_,
    this->has_latest_response_, this->latest_response_, this->dropped_response_count_,
    this->diagnostic_};
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
      static_cast<void>(this->performStop(
        StopCause::FAULT, lease_id,
        std::string("Unhandled Servo worker error: ") + error.what()));
    }
  } catch (...) {
    LeaseId lease_id;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      lease_id = this->lease_id_;
    }
    if (lease_id != ControlStateManager::NO_LEASE) {
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
    LeaseId lease_id = ControlStateManager::NO_LEASE;
    ServoType servo_type = ServoType::IDLE;

    {
      std::unique_lock<std::mutex> lock(this->mutex_);
      if (this->worker_should_exit_) {
        return;
      }

      const auto watchdog_from = this->has_target_update_time_ ?
        this->target_updated_at_ : this->started_at_;
      const auto watchdog_at = watchdog_from + this->options_.target_watchdog_timeout;
      const auto wake_at = std::min(
        std::min(this->next_send_at_, this->next_response_poll_at_), watchdog_at);
      this->cv_.wait_until(lock, wake_at);

      if (this->worker_should_exit_) {
        return;
      }

      const auto now = Clock::now();
      response_due = now >= this->next_response_poll_at_;
      watchdog_due = now >= watchdog_at;
      send_due = now >= this->next_send_at_;
      lease_id = this->lease_id_;
      servo_type = this->servo_type_;
      if (response_due) {
        this->advanceResponseDeadlineLocked(now);
      }
    }

    if (response_due) {
      std::string fault_message;
      if (!this->monitorResponses(fault_message)) {
        static_cast<void>(this->performStop(
          StopCause::FAULT, lease_id, fault_message));
        return;
      }
      if (!this->control_state_manager_->acceptsServoTarget(servo_type, lease_id)) {
        static_cast<void>(this->performStop(
          StopCause::FAULT, lease_id,
          "Servo lease or allowed RobotMode was lost during periodic control"));
        return;
      }
    }

    if (watchdog_due) {
      {
        std::lock_guard<std::mutex> lock(this->mutex_);
        this->watchdog_triggered_ = true;
      }
      static_cast<void>(this->performStop(
        StopCause::WATCHDOG, lease_id, "Servo target watchdog expired"));
      return;
    }

    if (send_due && !this->sendLatestTarget(servo_type, lease_id)) {
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
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->dropped_response_count_ = dropped >= this->response_drop_baseline_ ?
      dropped - this->response_drop_baseline_ : dropped;
    if (this->dropped_response_count_ > 0) {
      fault_message = "Motion response queue overflowed during Servo control";
      this->diagnostic_ = fault_message;
      return false;
    }
  }

  MotionResponse response;
  while (this->motion_commander_->tryTakeResponse(response)) {
    ServoType servo_type;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      servo_type = this->servo_type_;
    }

    const bool relevant = response.command.empty() ?
      response.result != MotionResponseResult::SUCCESS :
      isCommandForServoType(response.command, servo_type);
    if (!relevant) {
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->latest_response_ = response;
      this->has_latest_response_ = true;
    }
    if (response.result != MotionResponseResult::SUCCESS) {
      fault_message = std::string("Servo motion response failure: ") +
        responseResultName(response.result);
      if (response.error_code != MotionResponse::ERROR_CODE_UNAVAILABLE) {
        fault_message += " (controller error " + std::to_string(response.error_code) + ")";
      }
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->diagnostic_ = fault_message;
      return false;
    }
  }
  return true;
}

bool ServoControlSession::sendLatestTarget(
  const ServoType servo_type, const LeaseId lease_id)
{
  std::array<double, 4> target;
  bool has_target = false;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (this->worker_should_exit_ ||
      (this->state_ != State::STARTING && this->state_ != State::ACTIVE))
    {
      return true;
    }
    has_target = this->has_target_;
    target = this->target_;
  }

  if (!has_target) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->advanceSendDeadlineLocked(Clock::now());
    return true;
  }

  // The lease and Servo kind are deliberately rechecked immediately before
  // every TCP send; an accepted callback value is not sufficient authority.
  if (!this->control_state_manager_->acceptsServoTarget(servo_type, lease_id)) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->diagnostic_ = "Servo lease was rejected immediately before TCP send";
    return false;
  }

  try {
    if (servo_type == ServoType::SERVO_J) {
      this->motion_commander_->servoJ(target[0], target[1], target[2], target[3]);
    } else {
      this->motion_commander_->servoP(target[0], target[1], target[2], target[3]);
    }
  } catch (const std::exception & error) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->diagnostic_ = std::string("Servo TCP send failed: ") + error.what();
    return false;
  } catch (...) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->diagnostic_ = "Servo TCP send failed with an unknown exception";
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    ++this->sent_command_count_;
    this->has_last_send_time_ = true;
    this->last_sent_at_ = Clock::now();
    // Advance from the completion time so a delayed send cannot be followed by
    // a catch-up burst of overdue commands.
    this->advanceSendDeadlineLocked(this->last_sent_at_);
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

  ServoType servo_type;
  StopCause effective_cause = cause;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (lease_id == ControlStateManager::NO_LEASE || lease_id != this->lease_id_ ||
      !isServoType(this->servo_type_) || this->state_ == State::IDLE)
    {
      return Result{
        false, this->state_, ControlStateManager::NO_LEASE,
        "Servo stop lease does not match this session"};
    }
    servo_type = this->servo_type_;
    if ((this->state_ == State::STOPPING || this->state_ == State::FAULTED) &&
      this->stop_cause_ != StopCause::NONE)
    {
      // A retry completes the original stop path. In particular, a watchdog
      // retry must still release through handleServoWatchdogTimeout().
      effective_cause = this->stop_cause_;
    }
  }

  auto manager_snapshot = this->control_state_manager_->getSnapshot();
  if (manager_snapshot.lease_id != lease_id ||
    manager_snapshot.motion_owner != ownerForServoType(servo_type) ||
    manager_snapshot.control_state != servo_type)
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->state_ = State::FAULTED;
      this->worker_should_exit_ = true;
      this->stop_cause_ = effective_cause;
      this->diagnostic_ = "Servo ownership changed before the stop strategy";
    }
    this->cv_.notify_all();
    return this->localResult(
      false, ControlStateManager::NO_LEASE,
      "Servo ownership changed before the stop strategy");
  }

  if (manager_snapshot.accepting_servo_targets) {
    const auto begin_stop = this->control_state_manager_->beginServoStop(lease_id);
    if (!begin_stop.success) {
      return this->localResult(false, lease_id, begin_stop.message);
    }
  }

  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->state_ = State::STOPPING;
    this->has_target_ = false;
    this->worker_should_exit_ = true;
    this->stop_cause_ = effective_cause;
    if (effective_cause == StopCause::WATCHDOG) {
      this->watchdog_triggered_ = true;
    }
    this->diagnostic_ = detail;
  }
  this->cv_.notify_all();

  // Revalidate after target admission has been closed. No session or manager
  // mutex is held while the replaceable stop strategy calls ResetRobot().
  if (!this->currentLeaseMatches(lease_id, servo_type)) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->state_ = State::FAULTED;
    this->diagnostic_ = "Servo ownership changed before ResetRobot";
    return Result{
      false, this->state_, ControlStateManager::NO_LEASE, this->diagnostic_};
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
  if (!this->currentLeaseMatches(lease_id, servo_type)) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->state_ = State::FAULTED;
    this->diagnostic_ = "Servo ownership changed while the stop strategy was running";
    return Result{
      false, this->state_, ControlStateManager::NO_LEASE, this->diagnostic_};
  }

  if (!stop_result.success()) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->state_ = State::FAULTED;
    this->diagnostic_ = stop_result.message;
    return Result{false, this->state_, lease_id, this->diagnostic_};
  }

  const auto completed = effective_cause == StopCause::WATCHDOG ?
    this->control_state_manager_->handleServoWatchdogTimeout(lease_id) :
    this->control_state_manager_->requestControlState(ServoType::IDLE, lease_id);
  if (!completed.success) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->state_ = State::FAULTED;
    this->diagnostic_ = completed.message;
    return Result{false, this->state_, lease_id, this->diagnostic_};
  }

  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->state_ = State::IDLE;
    this->servo_type_ = ServoType::IDLE;
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

bool ServoControlSession::currentLeaseMatches(
  const LeaseId lease_id, const ServoType servo_type) const
{
  const auto snapshot = this->control_state_manager_->getSnapshot();
  return lease_id != ControlStateManager::NO_LEASE &&
         snapshot.lease_id == lease_id &&
         snapshot.control_state == servo_type &&
         snapshot.motion_owner == ownerForServoType(servo_type);
}

bool ServoControlSession::rejectTarget(const std::string & reason)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  ++this->rejected_target_count_;
  this->diagnostic_ = reason;
  return false;
}

ServoControlSession::Result ServoControlSession::localResult(
  const bool success, const LeaseId lease_id, const std::string & message) const
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  return Result{success, this->state_, lease_id, message};
}

bool ServoControlSession::isServoType(const ServoType servo_type) noexcept
{
  return servo_type == ServoType::SERVO_J || servo_type == ServoType::SERVO_P;
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

}  // namespace mg400_interface
