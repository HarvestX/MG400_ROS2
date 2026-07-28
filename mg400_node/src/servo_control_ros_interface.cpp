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

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mg400_node
{
namespace
{

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using KeyValue = diagnostic_msgs::msg::KeyValue;

std::string boolString(const bool value)
{
  return value ? "true" : "false";
}

KeyValue value(const std::string & key, const std::string & data)
{
  KeyValue output;
  output.key = key;
  output.value = data;
  return output;
}

template<typename IntegerT>
KeyValue integerValue(const std::string & key, const IntegerT data)
{
  return value(key, std::to_string(data));
}

std::int64_t nonnegativeAgeMilliseconds(
  const mg400_interface::ServoControlSession::Clock::time_point & now,
  const mg400_interface::ServoControlSession::Clock::time_point & then)
{
  if (then >= now) {
    return 0;
  }
  return std::chrono::duration_cast<std::chrono::milliseconds>(now - then).count();
}

mg400_interface::ControlStateManager::MotionOwner ownerForServoType(
  const mg400_interface::ControlStateManager::State servo_type)
{
  using Manager = mg400_interface::ControlStateManager;
  return servo_type == Manager::State::SERVO_J ?
         Manager::MotionOwner::SERVO_J : Manager::MotionOwner::SERVO_P;
}

}  // namespace

ServoControlRosInterface::ServoControlRosInterface(
  rclcpp_lifecycle::LifecycleNode & node,
  Manager::SharedPtr manager,
  const Options & options,
  TransformPoseFunction transform_pose)
: node_(node),
  manager_(std::move(manager)),
  options_(options),
  lifecycle_active_(false),
  ros_rejected_target_count_(0),
  transform_pose_(std::move(transform_pose))
{
  if (!this->manager_) {
    throw std::invalid_argument("Servo ROS interface requires a ControlStateManager");
  }
  if (this->options_.diagnostics_period <= std::chrono::nanoseconds::zero()) {
    throw std::invalid_argument("Servo diagnostics period must be positive");
  }
  if (this->options_.target_frame.empty()) {
    throw std::invalid_argument("ServoP target frame must not be empty");
  }

  if (!this->transform_pose_) {
    this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->node_.get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *this->tf_buffer_, this->node_.shared_from_this(), true);
    this->transform_pose_ =
      [this](
      const geometry_msgs::msg::PoseStamped & input,
      const std::string & target_frame,
      geometry_msgs::msg::PoseStamped & output,
      std::string & reason)
      {
        return this->defaultTransformPose(input, target_frame, output, reason);
      };
  }

  const auto control_state_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  this->control_state_publisher_ =
    rclcpp::create_publisher<mg400_msgs::msg::ControlState>(
    this->node_, "control_state", control_state_qos);
  this->diagnostics_publisher_ =
    rclcpp::create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    this->node_, "servo_diagnostics", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  this->service_callback_group_ = this->node_.create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  this->change_control_state_service_ = this->node_.create_service<ChangeControlState>(
    "change_control_state",
    std::bind(
      &ServoControlRosInterface::handleChangeControlState, this,
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
  this->servo_p_subscriber_ = this->node_.create_subscription<mg400_msgs::msg::ServoP>(
    "servo_p/target", rclcpp::QoS(rclcpp::KeepLast(1)),
    std::bind(
      &ServoControlRosInterface::handleServoPTarget, this,
      std::placeholders::_1),
    target_subscription_options);

  this->diagnostics_timer_ = this->node_.create_wall_timer(
    this->options_.diagnostics_period,
    [this]() {
      this->publishControlState();
      this->publishDiagnostics();
    });

  // This is a normal rclcpp publisher rather than a LifecyclePublisher, so the
  // configured UNAVAILABLE state is available to late subscribers immediately.
  this->publishControlState(true);
  this->publishDiagnostics();
}

ServoControlRosInterface::~ServoControlRosInterface()
{
  this->diagnostics_timer_.reset();
  this->change_control_state_service_.reset();
  this->servo_j_subscriber_.reset();
  this->servo_p_subscriber_.reset();
  this->tf_listener_.reset();
  this->tf_buffer_.reset();
}

void ServoControlRosInterface::installSession(std::shared_ptr<Session> session)
{
  if (!session) {
    throw std::invalid_argument("Cannot install an empty Servo Session");
  }
  std::lock_guard<std::mutex> lock(this->operation_mutex_);
  if (this->session_) {
    throw std::logic_error("A Servo Session is already installed");
  }
  {
    std::lock_guard<std::mutex> pointer_lock(this->session_pointer_mutex_);
    this->session_ = std::move(session);
  }
  this->lifecycle_active_ = true;
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
  std::unique_lock<std::mutex> lock(this->operation_mutex_);
  this->lifecycle_active_ = false;
  if (!this->session_) {
    return StopResult{true, false, "No Servo Session is installed"};
  }

  const auto session_snapshot = this->session_->getSnapshot();
  if (session_snapshot.lease_id == Manager::NO_LEASE) {
    return StopResult{true, false, "Servo Session has no active lease"};
  }

  const auto manager_snapshot = this->manager_->getSnapshot();
  if (manager_snapshot.lease_id != session_snapshot.lease_id ||
    manager_snapshot.control_state != session_snapshot.servo_type ||
    manager_snapshot.motion_owner != ownerForServoType(session_snapshot.servo_type))
  {
    {
      std::lock_guard<std::mutex> pointer_lock(this->session_pointer_mutex_);
      this->retired_snapshot_ = session_snapshot;
      this->retired_reason_ = reason + ": manager already revoked the old Servo lease";
      stale_session = std::move(this->session_);
    }
    const auto retired_reason = this->retired_reason_;
    lock.unlock();
    // ServoControlSession revalidates the lease before invoking the strategy,
    // so destruction of this stale epoch cannot issue ResetRobot.
    stale_session.reset();
    return StopResult{true, true, retired_reason};
  }

  const auto stopped = this->session_->stop(session_snapshot.lease_id);
  this->publishControlStateLocked(true);
  if (!stopped.success) {
    return StopResult{false, false, stopped.message};
  }
  return StopResult{true, false, stopped.message};
}

void ServoControlRosInterface::clearSession(const std::string & reason)
{
  std::shared_ptr<Session> old_session;
  {
    std::lock_guard<std::mutex> lock(this->operation_mutex_);
    this->lifecycle_active_ = false;
    if (!this->session_) {
      return;
    }
    std::lock_guard<std::mutex> pointer_lock(this->session_pointer_mutex_);
    this->retired_snapshot_ = this->session_->getSnapshot();
    this->retired_reason_ = reason;
    old_session = std::move(this->session_);
  }
  old_session.reset();
}

bool ServoControlRosInterface::retireSessionIfLeaseLost(const std::string & reason)
{
  std::shared_ptr<Session> stale_session;
  {
    std::lock_guard<std::mutex> lock(this->operation_mutex_);
    if (!this->session_) {
      return false;
    }
    const auto session_snapshot = this->session_->getSnapshot();
    if (session_snapshot.lease_id == Manager::NO_LEASE) {
      return false;
    }
    const auto manager_snapshot = this->manager_->getSnapshot();
    if (manager_snapshot.lease_id == session_snapshot.lease_id &&
      manager_snapshot.control_state == session_snapshot.servo_type &&
      manager_snapshot.motion_owner == ownerForServoType(session_snapshot.servo_type))
    {
      return false;
    }

    this->lifecycle_active_ = false;
    std::lock_guard<std::mutex> pointer_lock(this->session_pointer_mutex_);
    this->retired_snapshot_ = session_snapshot;
    this->retired_reason_ = reason;
    stale_session = std::move(this->session_);
  }
  stale_session.reset();
  this->publishControlState(true);
  this->publishDiagnostics();
  return true;
}

bool ServoControlRosInterface::hasSession() const
{
  std::lock_guard<std::mutex> lock(this->operation_mutex_);
  return static_cast<bool>(this->session_);
}

void ServoControlRosInterface::publishControlState(const bool force)
{
  this->publishControlStateLocked(force);
}

void ServoControlRosInterface::publishControlStateLocked(const bool force)
{
  const auto state = this->manager_->getState();
  {
    std::lock_guard<std::mutex> lock(this->diagnostic_mutex_);
    if (!force && this->last_published_state_ && *this->last_published_state_ == state) {
      return;
    }
    this->last_published_state_ = state;
  }

  mg400_msgs::msg::ControlState message;
  message.control_state = static_cast<std::uint8_t>(state);
  this->control_state_publisher_->publish(message);
}

void ServoControlRosInterface::publishDiagnostics()
{
  const auto manager_snapshot = this->manager_->getSnapshot();
  std::optional<Session::Snapshot> session_snapshot;
  std::string retired_reason;
  {
    std::lock_guard<std::mutex> lock(this->session_pointer_mutex_);
    if (this->session_) {
      session_snapshot = this->session_->getSnapshot();
    } else if (this->retired_snapshot_) {
      session_snapshot = this->retired_snapshot_;
    }
    retired_reason = this->retired_reason_;
  }

  std::string last_ros_rejection;
  {
    std::lock_guard<std::mutex> lock(this->diagnostic_mutex_);
    last_ros_rejection = this->last_ros_rejection_;
  }

  DiagnosticStatus status;
  status.name = "mg400/servo_control";
  status.hardware_id = this->options_.hardware_id;
  status.level = DiagnosticStatus::OK;
  status.message = "Servo control is ready";

  status.values.push_back(
    value(
      "control_state", Manager::toString(manager_snapshot.control_state)));
  status.values.push_back(
    value(
      "motion_owner", Manager::toString(manager_snapshot.motion_owner)));
  status.values.push_back(
    value(
      "has_current_lease", boolString(manager_snapshot.lease_id != Manager::NO_LEASE)));
  status.values.push_back(integerValue("lease_id", manager_snapshot.lease_id));
  status.values.push_back(value("connected", boolString(manager_snapshot.connected)));
  status.values.push_back(integerValue("robot_mode", manager_snapshot.robot_mode));
  status.values.push_back(
    value(
      "accepting_servo_targets", boolString(manager_snapshot.accepting_servo_targets)));
  status.values.push_back(
    integerValue(
      "ros_rejected_target_count", this->ros_rejected_target_count_.load()));
  status.values.push_back(value("last_ros_rejection", last_ros_rejection));
  status.values.push_back(value("retired_session_reason", retired_reason));

  if (session_snapshot) {
    const auto & snapshot = *session_snapshot;
    const auto now = Session::Clock::now();
    status.values.push_back(value("session_state", sessionStateName(snapshot.state)));
    status.values.push_back(
      value(
        "servo_type", Manager::toString(snapshot.servo_type)));
    status.values.push_back(value("has_target", boolString(snapshot.has_target)));
    status.values.push_back(
      value(
        "target_age_ms",
        snapshot.has_target_update_time ?
        std::to_string(nonnegativeAgeMilliseconds(now, snapshot.target_updated_at)) : "n/a"));
    status.values.push_back(
      value(
        "last_send_age_ms",
        snapshot.has_last_send_time ?
        std::to_string(nonnegativeAgeMilliseconds(now, snapshot.last_sent_at)) : "n/a"));
    status.values.push_back(
      value(
        "watchdog_triggered", boolString(snapshot.watchdog_triggered)));
    status.values.push_back(value("stop_cause", stopCauseName(snapshot.stop_cause)));
    status.values.push_back(
      integerValue(
        "accepted_target_count", snapshot.accepted_target_count));
    status.values.push_back(
      integerValue(
        "rejected_target_count",
        snapshot.rejected_target_count + this->ros_rejected_target_count_.load()));
    status.values.push_back(integerValue("sent_command_count", snapshot.sent_command_count));
    status.values.push_back(
      value(
        "latest_motion_response_result",
        snapshot.has_latest_response ?
        responseResultName(snapshot.latest_response.result) : "unavailable"));
    status.values.push_back(
      value(
        "controller_error_code",
        snapshot.has_latest_response &&
        snapshot.latest_response.error_code !=
        mg400_interface::MotionResponse::ERROR_CODE_UNAVAILABLE ?
        std::to_string(snapshot.latest_response.error_code) : "unavailable"));
    status.values.push_back(
      integerValue(
        "response_drop_count", snapshot.dropped_response_count));
    status.values.push_back(value("session_reason", snapshot.diagnostic));

    if (snapshot.state == Session::State::FAULTED) {
      status.level = DiagnosticStatus::ERROR;
      status.message = snapshot.diagnostic;
    } else if (snapshot.state == Session::State::STOPPING || snapshot.watchdog_triggered) {
      status.level = DiagnosticStatus::WARN;
      status.message = snapshot.diagnostic;
    } else {
      status.message = snapshot.diagnostic;
    }
  } else {
    status.values.push_back(value("session_state", "NOT_INSTALLED"));
  }

  if (manager_snapshot.control_state == Manager::State::UNAVAILABLE &&
    status.level != DiagnosticStatus::ERROR)
  {
    status.level = DiagnosticStatus::WARN;
    status.message = retired_reason.empty() ?
      "MG400 is unavailable for Servo control" : retired_reason;
  }

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = this->node_.now();
  array.status.push_back(std::move(status));
  this->diagnostics_publisher_->publish(array);
}

void ServoControlRosInterface::handleChangeControlState(
  const std::shared_ptr<ChangeControlState::Request> request,
  std::shared_ptr<ChangeControlState::Response> response)
{
  response->success = false;
  response->lease_id = Manager::NO_LEASE;

  std::unique_lock<std::mutex> lock(this->operation_mutex_);
  const auto requested_value = request->target_state.control_state;
  if (requested_value > static_cast<std::uint8_t>(Manager::State::SERVO_P)) {
    response->message = "Unknown control state";
  } else if (requested_value == static_cast<std::uint8_t>(Manager::State::UNAVAILABLE)) {
    response->message = "UNAVAILABLE cannot be requested";
  } else if (!this->session_) {
    response->message = "No Servo Session is available for this connection";
  } else {
    const auto requested_state = static_cast<Manager::State>(requested_value);
    if (requested_state == Manager::State::IDLE) {
      if (request->lease_id == Manager::NO_LEASE) {
        response->message = "A non-zero active Servo lease is required to stop";
      } else {
        const auto stopped = this->session_->stop(request->lease_id);
        response->success = stopped.success;
        response->message = stopped.message;
      }
    } else if (request->lease_id != Manager::NO_LEASE) {
      response->message = "A Servo start request must use lease ID zero";
    } else if (!this->lifecycle_active_) {
      response->message = "MG400Node is not active for Servo control";
    } else {
      const auto manager_snapshot = this->manager_->getSnapshot();
      const bool direct_switch =
        (manager_snapshot.control_state == Manager::State::SERVO_J ||
        manager_snapshot.control_state == Manager::State::SERVO_P) &&
        manager_snapshot.control_state != requested_state;
      if (direct_switch) {
        response->message = "Direct switching between SERVO_J and SERVO_P is not allowed";
      } else {
        const auto started = this->session_->start(requested_state);
        response->success = started.success;
        response->message = started.message;
        if (started.success) {
          response->lease_id = started.lease_id;
        }
      }
    }
  }

  const auto actual_state = this->manager_->getState();
  response->current_state.control_state = static_cast<std::uint8_t>(actual_state);
  // On every failure, and after a successful stop, lease_id remains zero. A
  // failed stop is retried with the lease retained by the caller and Manager.
  this->publishControlStateLocked(true);
  lock.unlock();
  this->publishDiagnostics();
}

void ServoControlRosInterface::handleServoJTarget(
  const mg400_msgs::msg::ServoJ::SharedPtr message)
{
  std::unique_lock<std::mutex> lock(this->operation_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    this->rejectRosTarget("ServoJ target rejected during a control-state transition");
    return;
  }
  if (!this->lifecycle_active_) {
    this->rejectRosTarget("ServoJ target rejected because MG400Node is not active");
    return;
  }
  if (!this->session_) {
    this->rejectRosTarget("ServoJ target rejected because no Session is installed");
    return;
  }

  const std::array<double, 4> target{{
    message->joint_angles[0], message->joint_angles[1],
    message->joint_angles[2], message->joint_angles[3]}};
  if (!this->session_->updateServoJTarget(message->lease_id, target)) {
    std::lock_guard<std::mutex> diagnostic_lock(this->diagnostic_mutex_);
    this->last_ros_rejection_ = this->session_->getSnapshot().diagnostic;
  }
}

void ServoControlRosInterface::handleServoPTarget(
  const mg400_msgs::msg::ServoP::SharedPtr message)
{
  std::unique_lock<std::mutex> lock(this->operation_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    this->rejectRosTarget("ServoP target rejected during a control-state transition");
    return;
  }
  if (!this->lifecycle_active_) {
    this->rejectRosTarget("ServoP target rejected because MG400Node is not active");
    return;
  }
  if (!this->session_) {
    this->rejectRosTarget("ServoP target rejected because no Session is installed");
    return;
  }
  if (message->pose.header.frame_id.empty()) {
    this->rejectRosTarget("ServoP target frame_id is empty");
    return;
  }

  std::string reason;
  if (!finitePose(message->pose.pose, reason)) {
    this->rejectRosTarget(reason);
    return;
  }
  double unused_yaw = 0.0;
  if (!quaternionToYaw(message->pose.pose.orientation, unused_yaw, reason)) {
    this->rejectRosTarget(reason);
    return;
  }

  geometry_msgs::msg::PoseStamped transformed;
  try {
    if (!this->transform_pose_(message->pose, this->options_.target_frame, transformed, reason)) {
      this->rejectRosTarget(
        reason.empty() ? "ServoP target TF transform failed" : reason);
      return;
    }
  } catch (const std::exception & error) {
    this->rejectRosTarget(std::string("ServoP target TF transform failed: ") + error.what());
    return;
  } catch (...) {
    this->rejectRosTarget("ServoP target TF transform failed with an unknown exception");
    return;
  }

  if (!finitePose(transformed.pose, reason)) {
    this->rejectRosTarget("Transformed " + reason);
    return;
  }
  double yaw = 0.0;
  if (!quaternionToYaw(transformed.pose.orientation, yaw, reason)) {
    this->rejectRosTarget("Transformed " + reason);
    return;
  }

  if (!this->session_->updateServoPTarget(
      message->lease_id,
      transformed.pose.position.x,
      transformed.pose.position.y,
      transformed.pose.position.z,
      yaw))
  {
    std::lock_guard<std::mutex> diagnostic_lock(this->diagnostic_mutex_);
    this->last_ros_rejection_ = this->session_->getSnapshot().diagnostic;
  }
}

std::uint64_t ServoControlRosInterface::getRosRejectedTargetCount() const noexcept
{
  return this->ros_rejected_target_count_.load();
}

std::string ServoControlRosInterface::getLastRosRejection() const
{
  std::lock_guard<std::mutex> lock(this->diagnostic_mutex_);
  return this->last_ros_rejection_;
}

bool ServoControlRosInterface::quaternionToYaw(
  const geometry_msgs::msg::Quaternion & quaternion,
  double & yaw,
  std::string & reason)
{
  const std::array<double, 4> values{{
    quaternion.x, quaternion.y, quaternion.z, quaternion.w}};
  if (!std::all_of(
      values.begin(), values.end(), [](const double component) {return std::isfinite(component);}))
  {
    reason = "ServoP quaternion contains a non-finite value";
    return false;
  }

  const double norm_squared =
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  if (!std::isfinite(norm_squared) || norm_squared <= std::numeric_limits<double>::epsilon()) {
    reason = "ServoP quaternion has zero norm";
    return false;
  }
  if (std::abs(norm_squared - 1.0) > 1.0e-3) {
    reason = "ServoP quaternion is not normalized";
    return false;
  }

  yaw = tf2::getYaw(quaternion);
  if (!std::isfinite(yaw)) {
    reason = "ServoP quaternion produced a non-finite yaw";
    return false;
  }
  reason.clear();
  return true;
}

void ServoControlRosInterface::rejectRosTarget(const std::string & reason)
{
  ++this->ros_rejected_target_count_;
  std::lock_guard<std::mutex> lock(this->diagnostic_mutex_);
  this->last_ros_rejection_ = reason;
}

bool ServoControlRosInterface::defaultTransformPose(
  const geometry_msgs::msg::PoseStamped & input,
  const std::string & target_frame,
  geometry_msgs::msg::PoseStamped & output,
  std::string & reason)
{
  try {
    this->tf_buffer_->transform(input, output, target_frame, tf2::durationFromSec(0.0));
    reason.clear();
    return true;
  } catch (const tf2::TransformException & error) {
    reason = std::string("ServoP target TF transform failed: ") + error.what();
    return false;
  }
}

bool ServoControlRosInterface::finitePose(
  const geometry_msgs::msg::Pose & pose,
  std::string & reason)
{
  const std::array<double, 7> values{{
    pose.position.x, pose.position.y, pose.position.z,
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w}};
  if (!std::all_of(
      values.begin(), values.end(), [](const double component) {return std::isfinite(component);}))
  {
    reason = "ServoP pose contains a non-finite value";
    return false;
  }
  reason.clear();
  return true;
}

const char * ServoControlRosInterface::sessionStateName(const Session::State state) noexcept
{
  switch (state) {
    case Session::State::IDLE:
      return "IDLE";
    case Session::State::STARTING:
      return "STARTING";
    case Session::State::ACTIVE:
      return "ACTIVE";
    case Session::State::STOPPING:
      return "STOPPING";
    case Session::State::FAULTED:
      return "FAULTED";
    default:
      return "UNKNOWN";
  }
}

const char * ServoControlRosInterface::stopCauseName(const Session::StopCause cause) noexcept
{
  switch (cause) {
    case Session::StopCause::NONE:
      return "NONE";
    case Session::StopCause::EXPLICIT:
      return "EXPLICIT";
    case Session::StopCause::WATCHDOG:
      return "WATCHDOG";
    case Session::StopCause::FAULT:
      return "FAULT";
    default:
      return "UNKNOWN";
  }
}

const char * ServoControlRosInterface::responseResultName(
  const mg400_interface::MotionResponseResult result) noexcept
{
  switch (result) {
    case mg400_interface::MotionResponseResult::SUCCESS:
      return "SUCCESS";
    case mg400_interface::MotionResponseResult::CONTROLLER_ERROR:
      return "CONTROLLER_ERROR";
    case mg400_interface::MotionResponseResult::TIMEOUT:
      return "TIMEOUT";
    case mg400_interface::MotionResponseResult::DISCONNECTED:
      return "DISCONNECTED";
    case mg400_interface::MotionResponseResult::PARSE_ERROR:
      return "PARSE_ERROR";
    default:
      return "UNKNOWN";
  }
}

}  // namespace mg400_node
