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

#include "mg400_node/servo_control_ros_interface.hpp"

#include <array>
#include <stdexcept>
#include <utility>

namespace mg400_node
{

ServoControlRosInterface::ServoControlRosInterface(
  rclcpp_lifecycle::LifecycleNode & node,
  Manager::SharedPtr manager,
  SafetyViolationState::SharedPtr safety_violation_state,
  OperationalErrorState::SharedPtr operational_error_state)
: node_(node),
  manager_(std::move(manager)),
  control_transition_in_progress_(false),
  lifecycle_active_(false),
  safety_violation_state_(std::move(safety_violation_state)),
  operational_error_state_(std::move(operational_error_state))
{
  if (!this->manager_) {
    throw std::invalid_argument("Servo ROS interface requires a ControlStateManager");
  }
  if (!this->safety_violation_state_) {
    throw std::invalid_argument("Servo ROS interface requires a safety violation state");
  }
  if (!this->operational_error_state_) {
    throw std::invalid_argument("Servo ROS interface requires an operational error state");
  }
  const auto control_state_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  this->control_state_publisher_ =
    rclcpp::create_publisher<mg400_msgs::msg::ControlState>(
    this->node_, "control_state", control_state_qos);
  const auto servo_error_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  this->servo_error_publisher_ =
    rclcpp::create_publisher<mg400_msgs::msg::ServoError>(
    this->node_, "servo_error", servo_error_qos);
  this->safety_violation_callback_ = this->safety_violation_state_->setChangeCallback(
    [this]() {
      this->publishServoError();
    });
  this->operational_error_callback_ = this->operational_error_state_->setChangeCallback(
    [this]() {
      this->publishServoError();
    });

  this->service_callback_group_ = this->node_.create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  this->enable_servo_j_service_ = this->node_.create_service<EnableServoJ>(
    "enable_servo_j",
    std::bind(
      &ServoControlRosInterface::handleEnableServoJ, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    this->service_callback_group_);

  rclcpp::SubscriptionOptions target_subscription_options;
  this->servo_j_subscriber_ = this->node_.create_subscription<mg400_msgs::msg::ServoJ>(
    "servo_j/target", rclcpp::QoS(rclcpp::KeepLast(1)),
    std::bind(
      &ServoControlRosInterface::handleServoJTarget, this,
      std::placeholders::_1),
    target_subscription_options);
  // This is a normal rclcpp publisher rather than a LifecyclePublisher, so the
  // configured UNAVAILABLE state is available to late subscribers immediately.
  this->publishControlState(true);
  this->publishServoError();
}

ServoControlRosInterface::~ServoControlRosInterface()
{
  // Stop and join state notifications before destroying ROS entities.
  this->operational_error_callback_.reset();
  this->safety_violation_callback_.reset();
  this->enable_servo_j_service_.reset();
  this->servo_j_subscriber_.reset();
}

void ServoControlRosInterface::installSession(std::shared_ptr<Session> session)
{
  if (!session) {
    throw std::invalid_argument("Cannot install an empty Servo Session");
  }
  if (session->getSafetyViolationStateShared() != this->safety_violation_state_) {
    throw std::invalid_argument(
            "Servo Session and ROS interface must share one safety violation state");
  }
  if (session->getOperationalErrorStateShared() != this->operational_error_state_) {
    throw std::invalid_argument(
            "Servo Session and ROS interface must share one operational error state");
  }
  {
    std::unique_lock<std::mutex> lock(this->operation_mutex_);
    this->operation_cv_.wait(
      lock, [this]() {return !this->control_transition_in_progress_;});
    if (this->session_) {
      throw std::logic_error("A Servo Session is already installed");
    }
    {
      std::lock_guard<std::mutex> pointer_lock(this->session_pointer_mutex_);
      this->session_ = std::move(session);
    }
    this->lifecycle_active_ = true;
  }
  this->publishServoError();
}

void ServoControlRosInterface::setLifecycleActive(const bool active)
{
  std::lock_guard<std::mutex> lock(this->operation_mutex_);
  this->lifecycle_active_ = active;
}

ServoControlRosInterface::StopResult ServoControlRosInterface::stopForLifecycle(
  const std::string & reason)
{
  std::shared_ptr<Session> stale_session;
  std::shared_ptr<Session> session_to_stop;
  Session::LeaseId lease_id = Manager::NO_LEASE;
  std::unique_lock<std::mutex> lock(this->operation_mutex_);
  this->lifecycle_active_ = false;
  this->operation_cv_.wait(
    lock, [this]() {return !this->control_transition_in_progress_;});
  if (!this->session_) {
    return StopResult{true, false, "No Servo Session is installed"};
  }

  const auto session_snapshot = this->session_->getSnapshot();
  if (session_snapshot.lease_id == Manager::NO_LEASE) {
    return StopResult{true, false, "Servo Session has no active lease"};
  }

  const auto manager_snapshot = this->manager_->getSnapshot();
  if (manager_snapshot.lease_id != session_snapshot.lease_id ||
    manager_snapshot.control_state != Manager::State::SERVO_J)
  {
    {
      std::lock_guard<std::mutex> pointer_lock(this->session_pointer_mutex_);
      stale_session = std::move(this->session_);
    }
    const auto retired_reason = reason + ": manager already revoked the old Servo lease";
    const bool report_lease_loss = manager_snapshot.connected;
    lock.unlock();
    if (report_lease_loss) {
      this->operational_error_state_->reportError(
        mg400_interface::ServoOperationalErrorCode::SERVO_LEASE_LOST,
        retired_reason);
    }
    // ServoControlSession revalidates the lease before invoking the strategy,
    // so destruction of this stale epoch cannot issue ResetRobot.
    stale_session.reset();
    return StopResult{true, true, retired_reason};
  }

  session_to_stop = this->session_;
  lease_id = session_snapshot.lease_id;
  this->control_transition_in_progress_ = true;
  lock.unlock();

  Session::Result stopped{
    false, session_snapshot.state, lease_id, "Servo lifecycle stop failed unexpectedly"};
  try {
    stopped = session_to_stop->stop();
  } catch (const std::exception & error) {
    stopped.message = std::string("Servo lifecycle stop threw: ") + error.what();
    this->operational_error_state_->reportError(
      mg400_interface::ServoOperationalErrorCode::INTERNAL_ERROR,
      "The Servo lifecycle stop raised an unexpected exception");
  } catch (...) {
    stopped.message = "Servo lifecycle stop threw an unknown exception";
    this->operational_error_state_->reportError(
      mg400_interface::ServoOperationalErrorCode::INTERNAL_ERROR,
      "The Servo lifecycle stop raised an unexpected exception");
  }
  {
    std::lock_guard<std::mutex> operation_lock(this->operation_mutex_);
    this->control_transition_in_progress_ = false;
  }
  this->operation_cv_.notify_all();
  this->publishControlState(true);
  return StopResult{stopped.success, false, stopped.message};
}

void ServoControlRosInterface::clearSession()
{
  std::shared_ptr<Session> old_session;
  {
    std::unique_lock<std::mutex> lock(this->operation_mutex_);
    this->lifecycle_active_ = false;
    this->operation_cv_.wait(
      lock, [this]() {return !this->control_transition_in_progress_;});
    if (!this->session_) {
      return;
    }
    std::lock_guard<std::mutex> pointer_lock(this->session_pointer_mutex_);
    old_session = std::move(this->session_);
  }
  old_session.reset();
  this->publishServoError();
}

bool ServoControlRosInterface::retireSessionIfLeaseLost(const std::string & reason)
{
  std::shared_ptr<Session> stale_session;
  std::string operational_message;
  bool report_lease_loss = false;
  {
    std::unique_lock<std::mutex> lock(this->operation_mutex_);
    if (!this->session_) {
      return false;
    }
    const auto session_snapshot = this->session_->getSnapshot();
    if (session_snapshot.lease_id == Manager::NO_LEASE) {
      return false;
    }
    const auto manager_snapshot = this->manager_->getSnapshot();
    if (manager_snapshot.lease_id == session_snapshot.lease_id &&
      manager_snapshot.control_state == Manager::State::SERVO_J)
    {
      return false;
    }

    this->lifecycle_active_ = false;
    this->operation_cv_.wait(
      lock, [this]() {return !this->control_transition_in_progress_;});
    if (!this->session_) {
      return false;
    }
    const auto current_session_snapshot = this->session_->getSnapshot();
    if (current_session_snapshot.lease_id == Manager::NO_LEASE) {
      return false;
    }
    const auto current_manager_snapshot = this->manager_->getSnapshot();
    if (current_manager_snapshot.lease_id == current_session_snapshot.lease_id &&
      current_manager_snapshot.control_state == Manager::State::SERVO_J)
    {
      return false;
    }
    std::lock_guard<std::mutex> pointer_lock(this->session_pointer_mutex_);
    operational_message = reason;
    report_lease_loss = current_manager_snapshot.connected;
    stale_session = std::move(this->session_);
  }
  if (report_lease_loss) {
    this->operational_error_state_->reportError(
      mg400_interface::ServoOperationalErrorCode::SERVO_LEASE_LOST,
      operational_message);
  }
  stale_session.reset();
  this->publishControlState(true);
  this->publishServoError();
  return true;
}

bool ServoControlRosInterface::hasSession() const
{
  std::lock_guard<std::mutex> lock(this->operation_mutex_);
  return static_cast<bool>(this->session_);
}

bool ServoControlRosInterface::hasActiveServoLease() const
{
  std::lock_guard<std::mutex> lock(this->session_pointer_mutex_);
  if (!this->session_) {
    return false;
  }
  const auto snapshot = this->session_->getSnapshot();
  return snapshot.lease_id != Manager::NO_LEASE &&
         snapshot.state != Session::State::IDLE;
}

ServoControlRosInterface::SafetyViolationState::SharedPtr
ServoControlRosInterface::getSafetyViolationStateShared() const noexcept
{
  return this->safety_violation_state_;
}

ServoControlRosInterface::OperationalErrorState::SharedPtr
ServoControlRosInterface::getOperationalErrorStateShared() const noexcept
{
  return this->operational_error_state_;
}

void ServoControlRosInterface::publishControlState(const bool force)
{
  this->publishControlStateLocked(force);
}

void ServoControlRosInterface::publishControlStateLocked(const bool force)
{
  const auto state = this->manager_->getState();
  {
    std::lock_guard<std::mutex> lock(this->publication_mutex_);
    if (!force && this->last_published_state_ && *this->last_published_state_ == state) {
      return;
    }
    this->last_published_state_ = state;
  }

  mg400_msgs::msg::ControlState message;
  message.control_state = static_cast<std::uint8_t>(state);
  this->control_state_publisher_->publish(message);
}

void ServoControlRosInterface::publishServoError()
{
  auto message = makeServoErrorMessage(
    this->safety_violation_state_->getSnapshot(),
    this->operational_error_state_->getSnapshot());
  message.header.stamp = this->node_.now();
  message.header.frame_id.clear();
  this->servo_error_publisher_->publish(message);
}

mg400_msgs::msg::ServoError ServoControlRosInterface::makeServoErrorMessage(
  const SafetyViolationState::Snapshot & safety,
  const OperationalErrorState::Snapshot & operational)
{
  mg400_msgs::msg::ServoError message;
  message.safety_violation_code = static_cast<std::uint8_t>(safety.code);
  if (message.safety_violation_code != mg400_msgs::msg::ServoError::SAFETY_NONE) {
    message.safety_violation_message = safety.message;
  }
  message.operational_error_code = static_cast<std::uint8_t>(operational.code);
  if (message.operational_error_code != mg400_msgs::msg::ServoError::OPERATIONAL_NONE) {
    message.operational_error_message = operational.message;
  }
  return message;
}

void ServoControlRosInterface::handleEnableServoJ(
  const std::shared_ptr<EnableServoJ::Request> request,
  std::shared_ptr<EnableServoJ::Response> response)
{
  response->success = false;
  response->enabled = false;
  response->lease_id = Manager::NO_LEASE;
  bool servo_rearmed = false;
  std::shared_ptr<Session> transition_session;
  bool transition_enable = false;

  {
    std::lock_guard<std::mutex> lock(this->operation_mutex_);
    if (!this->session_) {
      response->message = "No Servo Session is available for this connection";
    } else if (this->control_transition_in_progress_) {
      response->message = "Another ServoJ enable transition is in progress";
    } else if (request->enable && !this->lifecycle_active_) {
      response->message = "MG400Node is not active for ServoJ control";
    } else {
      transition_session = this->session_;
      transition_enable = request->enable;
      this->control_transition_in_progress_ = true;
    }
  }

  if (transition_session) {
    Session::Result result{
      false, Session::State::IDLE, Manager::NO_LEASE,
      "EnableServoJ transition failed unexpectedly"};
    try {
      result = transition_enable ?
        transition_session->start() : transition_session->stop();
    } catch (const std::exception & error) {
      result.message = std::string("EnableServoJ transition threw: ") + error.what();
      this->operational_error_state_->reportError(
        mg400_interface::ServoOperationalErrorCode::INTERNAL_ERROR,
        "An EnableServoJ transition raised an unexpected exception");
    } catch (...) {
      result.message = "EnableServoJ transition threw an unknown exception";
      this->operational_error_state_->reportError(
        mg400_interface::ServoOperationalErrorCode::INTERNAL_ERROR,
        "An EnableServoJ transition raised an unexpected exception");
    }
    response->success = result.success;
    response->message = result.message;
    if (result.success && transition_enable) {
      response->lease_id = result.lease_id;
      servo_rearmed = true;
    }
    {
      std::lock_guard<std::mutex> lock(this->operation_mutex_);
      this->control_transition_in_progress_ = false;
    }
    this->operation_cv_.notify_all();
  }

  response->enabled = this->manager_->getState() == Manager::State::SERVO_J;
  // On every failure, and after a successful stop, lease_id remains zero. A
  // failed stop can be retried with another enable=false request.
  this->publishControlState(true);
  if (servo_rearmed) {
    // Both latch callbacks may have published an intermediate clear state.
    // This explicit snapshot guarantees the final paired rearm state.
    this->publishServoError();
  }
}

void ServoControlRosInterface::handleServoJTarget(
  const mg400_msgs::msg::ServoJ::SharedPtr message)
{
  std::shared_ptr<Session> session;
  std::string admission_rejection;
  std::unique_lock<std::mutex> lock(this->operation_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    admission_rejection = "ServoJ target rejected during a Servo-state transition";
  } else if (this->control_transition_in_progress_) {
    admission_rejection = "ServoJ target rejected during a Servo-state transition";
  } else if (!this->lifecycle_active_) {
    admission_rejection = "ServoJ target rejected because MG400Node is not active";
  } else if (!this->session_) {
    admission_rejection = "ServoJ target rejected because no Session is installed";
  } else {
    session = this->session_;
  }
  if (lock.owns_lock()) {
    lock.unlock();
  }
  if (!admission_rejection.empty()) {
    this->rejectRosTarget(admission_rejection);
    return;
  }

  const std::array<double, 4> target{{
    message->joint_angles[0], message->joint_angles[1],
    message->joint_angles[2], message->joint_angles[3]}};
  static_cast<void>(session->updateServoJTarget(message->lease_id, target));
}

void ServoControlRosInterface::rejectRosTarget(const std::string & reason)
{
  RCLCPP_WARN(this->node_.get_logger(), "%s", reason.c_str());
}

}  // namespace mg400_node
