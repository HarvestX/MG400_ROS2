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

#include "mg400_node/servo_j_controller.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include <mg400_msgs/msg/robot_mode.hpp>

namespace mg400_node
{

using namespace std::chrono_literals;  // NOLINT

ServoJController::ServoJController(
  rclcpp_lifecycle::LifecycleNode & node,
  mg400_interface::MG400Interface::SharedPtr interface,
  const std::string & wire_format, const double default_t,
  const double aheadtime, const double gain)
: node_(node), interface_(std::move(interface)),
  wire_format_(mg400_interface::MotionCommander::ServoJFormat::FOUR_AXES_WITH_T),
  default_t_(default_t), aheadtime_(aheadtime), gain_(gain)
{
  using Format = mg400_interface::MotionCommander::ServoJFormat;
  if (wire_format == "four_axes_with_t") {
    wire_format_ = Format::FOUR_AXES_WITH_T;
  } else if (wire_format == "six_axes") {
    wire_format_ = Format::SIX_AXES;
  } else if (wire_format == "with_t") {
    wire_format_ = Format::WITH_T;
  } else if (wire_format == "full") {
    wire_format_ = Format::FULL;
  } else {
    throw std::invalid_argument(
            "servo_j_wire_format must be four_axes_with_t, six_axes, with_t, or full");
  }
  if (!std::isfinite(default_t_) || default_t_ < 0.004 || default_t_ > 3600.0 ||
    !std::isfinite(aheadtime_) || aheadtime_ < 20.0 || aheadtime_ > 100.0 ||
    !std::isfinite(gain_) || gain_ < 200.0 || gain_ > 1000.0)
  {
    throw std::invalid_argument("ServoJ t, aheadtime, or gain parameter is out of range");
  }

  next_session_id_ = static_cast<uint64_t>(Clock::now().time_since_epoch().count());
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  setpoint_sub_ = node_.create_subscription<ServoJ>(
    "servo_j", qos,
    [this](const ServoJ::SharedPtr msg) {onSetpoint(msg);});
  session_srv_ = node_.create_service<Session>(
    "servo_j_session",
    [this](const Session::Request::SharedPtr req, Session::Response::SharedPtr res) {
      onSession(req, res);
    });
  send_timer_ = node_.create_wall_timer(30ms, [this]() {onSendTimer();});
  send_timer_->cancel();
}

ServoJController::~ServoJController()
{
  deactivate();
  send_timer_.reset();
  setpoint_sub_.reset();
  session_srv_.reset();
}

void ServoJController::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  available_ = true;
  send_timer_->reset();
}

void ServoJController::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  available_ = false;
  if (send_timer_) {
    send_timer_->cancel();
  }
  clearSessionLocked();
}

void ServoJController::clearSessionLocked()
{
  session_id_ = 0;
  stopping_ = false;
  have_new_target_ = false;
  stale_warned_ = false;
  last_receive_ = Clock::time_point{};
  last_send_ = Clock::time_point{};
  interface_->endServoSession();
}

void ServoJController::onSession(
  const Session::Request::SharedPtr req, Session::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(mutex_);
  res->success = false;
  res->session_id = session_id_;
  if (session_id_ != 0 && !interface_->robot_state_machine->isServoSessionActive()) {
    clearSessionLocked();
  }

  if (req->start) {
    if (req->session_id != 0 || !available_ || !interface_->ok() || session_id_ != 0 ||
      !interface_->tryBeginServoSession())
    {
      res->message = "ServoJ start requires an idle, enabled robot and no active session";
      return;
    }
    session_id_ = ++next_session_id_;
    if (session_id_ == 0) {
      session_id_ = ++next_session_id_;
    }
    stopping_ = false;
    have_new_target_ = false;
    last_receive_ = Clock::time_point{};
    last_send_ = Clock::time_point{};
    res->success = true;
    res->session_id = session_id_;
    res->message = "ServoJ session started";
    RCLCPP_INFO(node_.get_logger(), "ServoJ session started");
    return;
  }

  if (req->session_id == 0 || req->session_id != session_id_) {
    res->message = "ServoJ session ID does not match";
    return;
  }
  // Stop accepting setpoints first; do not allow another motion while a final
  // ServoJ point may still be executing in the controller.
  stopping_ = true;
  have_new_target_ = false;
  if (last_send_ != Clock::time_point{} && Clock::now() - last_send_ < 300ms) {
    res->message = "Wait at least 300 ms after the last ServoJ point and retry stop";
    return;
  }
  const auto snapshot = interface_->robot_state_machine->getSnapshot();
  if (!interface_->ok() || !snapshot.feedback_fresh ||
    snapshot.raw_robot_mode != mg400_msgs::msg::RobotMode::ENABLE)
  {
    res->message = "Wait for fresh ENABLE feedback and retry stop";
    return;
  }
  clearSessionLocked();
  res->success = true;
  res->session_id = 0;
  res->message = "ServoJ session stopped";
  RCLCPP_INFO(node_.get_logger(), "ServoJ session stopped");
}

void ServoJController::onSetpoint(const ServoJ::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!available_ || stopping_ || session_id_ == 0 || msg->session_id != session_id_) {
    return;
  }
  if (!interface_->robot_state_machine->isServoSessionActive()) {
    clearSessionLocked();
    return;
  }
  for (const auto joint : msg->joint_positions) {
    if (!std::isfinite(joint)) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000, "Invalid ServoJ joint");
      return;
    }
  }
  const std::vector<double> joints(msg->joint_positions.begin(), msg->joint_positions.end());
  if (!ik_util_.InMG400Range(joints)) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 1000, "ServoJ target is outside MG400 joint limits");
    return;
  }
  const double t = msg->t == 0.0 ? default_t_ : msg->t;
  if (!std::isfinite(t) || t < 0.004 || t > 3600.0) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000, "Invalid ServoJ t");
    return;
  }
  target_ = msg->joint_positions;
  target_t_ = t;
  last_receive_ = Clock::now();
  have_new_target_ = true;
  stale_warned_ = false;
}

void ServoJController::onSendTimer()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!available_ || session_id_ == 0 || stopping_) {
    return;
  }
  const auto snapshot = interface_->robot_state_machine->getSnapshot();
  if (!interface_->ok() || !snapshot.feedback_fresh ||
    snapshot.state != mg400_interface::RobotStateMachine::State::SERVO)
  {
    RCLCPP_ERROR(node_.get_logger(), "ServoJ session invalidated by robot state or connection");
    clearSessionLocked();
    return;
  }
  if (last_receive_ != Clock::time_point{} && Clock::now() - last_receive_ > 150ms) {
    have_new_target_ = false;
    if (!stale_warned_) {
      RCLCPP_WARN(node_.get_logger(), "ServoJ stream stale; session retained until explicit stop");
      stale_warned_ = true;
    }
    return;
  }
  if (!have_new_target_) {
    return;
  }
  try {
    interface_->motion_commander->servoJ(
      target_, target_t_, wire_format_, aheadtime_, gain_);
    last_send_ = Clock::now();
    have_new_target_ = false;
  } catch (const std::exception & e) {
    stopping_ = true;
    have_new_target_ = false;
    RCLCPP_ERROR(node_.get_logger(), "ServoJ send failed; session retained: %s", e.what());
  }
}

}  // namespace mg400_node
