// Copyright 2022 HarvestX Inc.
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

#include "mg400_plugin/motion_api/move_jog.hpp"

#include <thread>

#include <mg400_msgs/msg/robot_mode.hpp>

namespace mg400_plugin
{

MoveJog::~MoveJog()
{
  std::lock_guard<std::mutex> lock(this->jog_state_mutex_);
  if (this->jog_lease_ && this->jog_lease_->isCurrent()) {
    // Unloading this plugin is not proof that a jogging robot stopped. Leave
    // ControlStateManager owned so any remaining Servo/Motion API fails closed.
    this->jog_lease_->retainManagerOwnership();
  }
}

void MoveJog::configure(
  const mg400_interface::MotionCommander::SharedPtr commander,
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_if,
  const rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_if,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_if,
  const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_if,
  const rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_if,
  const mg400_interface::MG400Interface::SharedPtr mg400_if)
{
  if (!this->configure_base(
      commander, node_base_if, node_clock_if,
      node_logging_if, node_services_if, node_waitables_if, mg400_if))
  {
    return;
  }

  using namespace std::placeholders;  // NOLINT
  this->srv_ = rclcpp::create_service<ServiceT, CallbackT>(
    this->node_base_if_,
    this->node_services_if_,
    "move_jog",
    std::bind(&MoveJog::onServiceCall, this, _1, _2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(),
    this->node_base_if_->get_default_callback_group());
}


void MoveJog::onServiceCall(
  const ServiceT::Request::SharedPtr req,
  ServiceT::Response::SharedPtr res)
{
  res->error_id = -1;
  if (req->jog.jog_mode.empty()) {
    this->handleStop(res);
  } else {
    this->handleStart(req->jog.jog_mode, res);
  }
}

void MoveJog::handleStart(
  const std::string & jog_mode, ServiceT::Response::SharedPtr res)
{
  if (!this->mg400_interface_->ok()) {
    RCLCPP_ERROR(this->node_logging_if_->get_logger(), "MG400 is not connected");
    return;
  }

  bool reuse_active_lease = false;
  mg400_interface::ControlStateManager::LeaseId active_lease_id =
    mg400_interface::ControlStateManager::NO_LEASE;
  {
    std::lock_guard<std::mutex> lock(this->jog_state_mutex_);
    if (this->jog_state_ == JogState::STARTING || this->jog_state_ == JogState::STOPPING) {
      RCLCPP_WARN(this->node_logging_if_->get_logger(), "MoveJog transition is already active");
      return;
    }
    if (this->jog_state_ == JogState::ACTIVE && this->jog_lease_) {
      reuse_active_lease = true;
      active_lease_id = this->jog_lease_->id();
      // Reserve the direction update so STOP cannot be sent concurrently and
      // then followed by a late direction command.
      this->jog_state_ = JogState::STARTING;
    } else {
      this->jog_state_ = JogState::STARTING;
    }
  }

  if (reuse_active_lease) {
    const auto snapshot =
      this->mg400_interface_->getControlStateManagerShared()->getSnapshot();
    const bool lease_is_current =
      snapshot.motion_owner ==
      mg400_interface::ControlStateManager::MotionOwner::REGULAR_MOTION &&
      snapshot.lease_id == active_lease_id;

    if (lease_is_current) {
      try {
        this->commander_->moveJog(jog_mode);
        res->error_id = 0;
      } catch (const std::exception & error) {
        // The previous Jog command may still be moving the robot. Retain the
        // existing lease when a direction-change send fails.
        RCLCPP_ERROR(this->node_logging_if_->get_logger(), "%s", error.what());
      } catch (...) {
        RCLCPP_ERROR(this->node_logging_if_->get_logger(), "Interface Error");
      }
      std::lock_guard<std::mutex> lock(this->jog_state_mutex_);
      this->jog_state_ = JogState::ACTIVE;
      return;
    }

    std::optional<mg400_plugin_base::RegularMotionLease> stale_lease;
    {
      std::lock_guard<std::mutex> lock(this->jog_state_mutex_);
      if (this->jog_state_ != JogState::STARTING || !this->jog_lease_ ||
        this->jog_lease_->id() != active_lease_id)
      {
        RCLCPP_WARN(this->node_logging_if_->get_logger(), "MoveJog ownership changed concurrently");
        return;
      }
      stale_lease = std::move(this->jog_lease_);
      this->jog_lease_.reset();
      this->jog_state_ = JogState::STARTING;
    }
    stale_lease.reset();
  }

  auto lease = this->tryAcquireRegularMotionLease();
  if (!lease) {
    std::lock_guard<std::mutex> lock(this->jog_state_mutex_);
    if (this->jog_state_ == JogState::STARTING) {
      this->jog_state_ = JogState::IDLE;
    }
    return;
  }

  try {
    this->commander_->moveJog(jog_mode);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(this->node_logging_if_->get_logger(), "%s", error.what());
    std::lock_guard<std::mutex> lock(this->jog_state_mutex_);
    this->jog_state_ = JogState::IDLE;
    return;
  } catch (...) {
    RCLCPP_ERROR(this->node_logging_if_->get_logger(), "Interface Error");
    std::lock_guard<std::mutex> lock(this->jog_state_mutex_);
    this->jog_state_ = JogState::IDLE;
    return;
  }

  {
    std::lock_guard<std::mutex> lock(this->jog_state_mutex_);
    this->jog_lease_ = std::move(*lease);
    this->jog_state_ = JogState::ACTIVE;
  }
  res->error_id = 0;
}

void MoveJog::handleStop(ServiceT::Response::SharedPtr res)
{
  bool had_lease = false;
  {
    std::lock_guard<std::mutex> lock(this->jog_state_mutex_);
    if (this->jog_state_ == JogState::STARTING || this->jog_state_ == JogState::STOPPING) {
      RCLCPP_WARN(this->node_logging_if_->get_logger(), "MoveJog transition is already active");
      return;
    }
    had_lease = this->jog_lease_.has_value();
    this->jog_state_ = JogState::STOPPING;
  }

  // Synchronize connection/RobotMode into ControlStateManager, but still try
  // the stop send: port 30003 may remain available if another channel failed.
  (void)this->mg400_interface_->ok();
  const auto deadline = std::chrono::steady_clock::now() + STOP_TIMEOUT;
  try {
    this->commander_->moveJog("");
  } catch (const std::exception & error) {
    RCLCPP_ERROR(this->node_logging_if_->get_logger(), "%s", error.what());
    std::lock_guard<std::mutex> lock(this->jog_state_mutex_);
    this->jog_state_ = had_lease ? JogState::ACTIVE : JogState::IDLE;
    return;
  } catch (...) {
    RCLCPP_ERROR(this->node_logging_if_->get_logger(), "Interface Error");
    std::lock_guard<std::mutex> lock(this->jog_state_mutex_);
    this->jog_state_ = had_lease ? JogState::ACTIVE : JogState::IDLE;
    return;
  }

  const auto confirmation = waitForMoveJogStop(
    deadline,
    []() {return std::chrono::steady_clock::now();},
    [this](const std::chrono::steady_clock::duration & remaining)
    -> std::optional<mg400_interface::MotionResponse> {
      mg400_interface::MotionResponse response;
      if (!this->commander_->waitForResponse(response, remaining)) {
        return std::nullopt;
      }
      return response;
    },
    [this]() {
      using RobotMode = mg400_msgs::msg::RobotMode;
      if (!this->mg400_interface_->ok()) {
        return MoveJogRobotStatus::DISCONNECTED;
      }
      if (this->mg400_interface_->realtime_tcp_interface->isRobotMode(RobotMode::ENABLE)) {
        return MoveJogRobotStatus::ENABLED;
      }
      return MoveJogRobotStatus::WAITING;
    },
    [](const std::chrono::steady_clock::duration & duration) {
      std::this_thread::sleep_for(duration);
    });
  if (confirmation.result != MoveJogStopResult::CONFIRMED) {
    RCLCPP_ERROR(
      this->node_logging_if_->get_logger(),
      "MoveJog stop was not confirmed; retaining regular-motion ownership");
    res->error_id = confirmation.controller_error_id;
    std::lock_guard<std::mutex> lock(this->jog_state_mutex_);
    this->jog_state_ = had_lease ? JogState::ACTIVE : JogState::IDLE;
    return;
  }

  std::optional<mg400_plugin_base::RegularMotionLease> completed_lease;
  {
    std::lock_guard<std::mutex> lock(this->jog_state_mutex_);
    completed_lease = std::move(this->jog_lease_);
    this->jog_lease_.reset();
    this->jog_state_ = JogState::IDLE;
  }
  completed_lease.reset();
  res->error_id = 0;
}
}  // namespace mg400_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  mg400_plugin::MoveJog,
  mg400_plugin_base::MotionApiPluginBase)
