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

#include "mg400_plugin/motion_api/servo_j.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include <mg400_msgs/msg/robot_mode.hpp>
#include <rclcpp/create_service.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/create_timer.hpp>

namespace mg400_plugin
{

using namespace std::chrono_literals;  // NOLINT

void ServoJ::configure(
  const mg400_interface::MotionCommander::SharedPtr commander,
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_if,
  const rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_if,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_if,
  const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_if,
  const rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_if,
  const mg400_interface::MG400Interface::SharedPtr mg400_if)
{
  if (!this->configure_base(
      commander, node_base_if, node_clock_if, node_logging_if,
      node_services_if, node_waitables_if, mg400_if))
  {
    return;
  }
  if (!node_parameters_if_ || !node_topics_if_ || !node_timers_if_) {
    throw std::runtime_error("ServoJ plugin node resources are not configured");
  }

  const auto wire_format = node_parameters_if_->get_parameter("servo_j_wire_format").as_string();
  default_t_ = node_parameters_if_->get_parameter("servo_j_default_t").as_double();
  aheadtime_ = node_parameters_if_->get_parameter("servo_j_aheadtime").as_double();
  gain_ = node_parameters_if_->get_parameter("servo_j_gain").as_double();

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
  setpoint_sub_ = rclcpp::create_subscription<Setpoint>(
    node_parameters_if_, node_topics_if_, "servo_j", qos,
    [this](const Setpoint::SharedPtr msg) {onSetpoint(msg);});
  session_srv_ = rclcpp::create_service<Session>(
    node_base_if_, node_services_if_, "servo_j_session",
    [this](const Session::Request::SharedPtr req, Session::Response::SharedPtr res) {
      onSession(req, res);
    }, rclcpp::ServicesQoS().get_rmw_qos_profile(),
    node_base_if_->get_default_callback_group());
  send_timer_ = rclcpp::create_wall_timer(
    30ms, [this]() {onSendTimer();}, node_base_if_->get_default_callback_group(),
    node_base_if_.get(), node_timers_if_.get());
  send_timer_->cancel();
}

ServoJ::~ServoJ()
{
  deactivate();
  send_timer_.reset();
  setpoint_sub_.reset();
  session_srv_.reset();
}

void ServoJ::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  available_ = true;
  send_timer_->reset();
}

void ServoJ::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  available_ = false;
  if (send_timer_) {
    send_timer_->cancel();
  }
  clearSessionLocked();
}

void ServoJ::clearSessionLocked()
{
  session_id_ = 0;
  stopping_ = false;
  have_new_target_ = false;
  stale_warned_ = false;
  last_receive_ = Clock::time_point{};
  last_send_ = Clock::time_point{};
  if (mg400_interface_) {
    mg400_interface_->endServoSession();
  }
}

void ServoJ::onSession(
  const Session::Request::SharedPtr req, Session::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(mutex_);
  res->success = false;
  res->session_id = session_id_;
  if (session_id_ != 0 && !mg400_interface_->robot_state_machine->isServoSessionActive()) {
    clearSessionLocked();
  }

  if (req->start) {
    if (req->session_id != 0 || !available_ || !mg400_interface_->ok() || session_id_ != 0 ||
      !mg400_interface_->tryBeginServoSession())
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
    RCLCPP_INFO(node_logging_if_->get_logger(), "ServoJ session started");
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
  const auto snapshot = mg400_interface_->robot_state_machine->getSnapshot();
  if (!mg400_interface_->ok() || !snapshot.feedback_fresh ||
    snapshot.raw_robot_mode != mg400_msgs::msg::RobotMode::ENABLE)
  {
    res->message = "Wait for fresh ENABLE feedback and retry stop";
    return;
  }
  clearSessionLocked();
  res->success = true;
  res->session_id = 0;
  res->message = "ServoJ session stopped";
  RCLCPP_INFO(node_logging_if_->get_logger(), "ServoJ session stopped");
}

void ServoJ::onSetpoint(const Setpoint::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!available_ || stopping_ || session_id_ == 0 || msg->session_id != session_id_) {
    return;
  }
  if (!mg400_interface_->robot_state_machine->isServoSessionActive()) {
    clearSessionLocked();
    return;
  }
  for (const auto joint : msg->joint_positions) {
    if (!std::isfinite(joint)) {
      RCLCPP_WARN_THROTTLE(
        node_logging_if_->get_logger(), *node_clock_if_->get_clock(), 1000,
        "Invalid ServoJ joint");
      return;
    }
  }
  const std::vector<double> joints(msg->joint_positions.begin(), msg->joint_positions.end());
  if (!ik_util_.InMG400Range(joints)) {
    RCLCPP_WARN_THROTTLE(
      node_logging_if_->get_logger(), *node_clock_if_->get_clock(), 1000,
      "ServoJ target is outside MG400 joint limits");
    return;
  }
  const double t = msg->t == 0.0 ? default_t_ : msg->t;
  if (!std::isfinite(t) || t < 0.004 || t > 3600.0) {
    RCLCPP_WARN_THROTTLE(
      node_logging_if_->get_logger(), *node_clock_if_->get_clock(), 1000,
      "Invalid ServoJ t");
    return;
  }
  target_ = msg->joint_positions;
  target_t_ = t;
  last_receive_ = Clock::now();
  have_new_target_ = true;
  stale_warned_ = false;
}

void ServoJ::onSendTimer()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!available_ || session_id_ == 0 || stopping_) {
    return;
  }
  const auto snapshot = mg400_interface_->robot_state_machine->getSnapshot();
  if (!mg400_interface_->ok() || !snapshot.feedback_fresh ||
    snapshot.state != mg400_interface::RobotStateMachine::State::SERVO)
  {
    RCLCPP_ERROR(
      node_logging_if_->get_logger(),
      "ServoJ session invalidated by robot state or connection");
    clearSessionLocked();
    return;
  }
  if (last_receive_ != Clock::time_point{} && Clock::now() - last_receive_ > 150ms) {
    have_new_target_ = false;
    if (!stale_warned_) {
      RCLCPP_WARN(
        node_logging_if_->get_logger(),
        "ServoJ stream stale; session retained until explicit stop");
      stale_warned_ = true;
    }
    return;
  }
  if (!have_new_target_) {
    return;
  }
  try {
    commander_->servoJ(
      target_, target_t_, wire_format_, aheadtime_, gain_);
    last_send_ = Clock::now();
    have_new_target_ = false;
  } catch (const std::exception & e) {
    stopping_ = true;
    have_new_target_ = false;
    RCLCPP_ERROR(
      node_logging_if_->get_logger(),
      "ServoJ send failed; session retained: %s", e.what());
  }
}

}  // namespace mg400_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mg400_plugin::ServoJ, mg400_plugin_base::MotionApiPluginBase)
