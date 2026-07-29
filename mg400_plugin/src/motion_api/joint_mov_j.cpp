// Copyright 2025 HarvestX Inc.
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

#include "mg400_plugin/motion_api/joint_mov_j.hpp"

namespace mg400_plugin
{

void JointMovJ::configure(
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

  this->action_server_ =
    rclcpp_action::create_server<ActionT>(
    this->node_base_if_,
    this->node_clock_if_,
    this->node_logging_if_,
    this->node_waitable_if_,
    "joint_mov_j",
    std::bind(&JointMovJ::handle_goal, this, _1, _2),
    std::bind(&JointMovJ::handle_cancel, this, _1),
    std::bind(&JointMovJ::handle_accepted, this, _1),
    rcl_action_server_get_default_options(),
    this->node_base_if_->get_default_callback_group());
}

rclcpp_action::GoalResponse JointMovJ::handle_goal(
  const rclcpp_action::GoalUUID & uuid, ActionT::Goal::ConstSharedPtr goal)
{
  if (!this->mg400_interface_->ok()) {
    RCLCPP_ERROR(
      this->node_logging_if_->get_logger(), "MG400 is not connected");
    return rclcpp_action::GoalResponse::REJECT;
  }

  const std::array<double, 4> goal_angles = {
    goal->joint_angles[0], goal->joint_angles[1],
    goal->joint_angles[2], goal->joint_angles[3]};
  if (!this->mg400_ik_util_.InMG400Range(
      std::vector<double>(goal_angles.begin(), goal_angles.end())))
  {
    RCLCPP_ERROR(this->node_logging_if_->get_logger(), "The angles are outside of the range.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  auto lease = this->tryAcquireRegularMotionLease();
  if (!lease) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!this->goal_reservations_.reserve(
      mg400_plugin_base::makeActionGoalKey(uuid), std::move(*lease)))
  {
    RCLCPP_ERROR(this->node_logging_if_->get_logger(), "Duplicate Action goal UUID");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointMovJ::handle_cancel(
  const std::shared_ptr<GoalHandle>/*unused*/)
{
  RCLCPP_INFO(
    this->node_logging_if_->get_logger(), "Received request to cancel goal");
  // No verified safe-stop command exists here. Execution therefore keeps the
  // lease until the robot reaches its terminal state after cancellation.
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JointMovJ::handle_accepted(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  auto lease = this->goal_reservations_.take(
    mg400_plugin_base::makeActionGoalKey(goal_handle->get_goal_id()));
  if (!lease || !lease->isCurrent()) {
    RCLCPP_ERROR(this->node_logging_if_->get_logger(), "Accepted goal has no current motion lease");
    auto result = std::make_shared<ActionT::Result>();
    result->result = false;
    goal_handle->abort(result);
    return;
  }
  try {
    std::thread{
      [this, goal_handle, lease = std::move(*lease)]() mutable {
        try {
          this->execute(goal_handle, std::move(lease));
        } catch (const std::exception & error) {
          RCLCPP_ERROR(this->node_logging_if_->get_logger(), "%s", error.what());
          auto result = std::make_shared<ActionT::Result>();
          result->result = false;
          goal_handle->abort(result);
        } catch (...) {
          RCLCPP_ERROR(this->node_logging_if_->get_logger(), "Unhandled motion execution error");
          auto result = std::make_shared<ActionT::Result>();
          result->result = false;
          goal_handle->abort(result);
        }
      }}.detach();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      this->node_logging_if_->get_logger(), "Failed to start execution: %s",
      error.what());
    auto result = std::make_shared<ActionT::Result>();
    result->result = false;
    goal_handle->abort(result);
  }
}


void JointMovJ::execute(
  const std::shared_ptr<GoalHandle> goal_handle,
  GoalReservations::Lease /*lease*/)
{
  rclcpp::Rate control_freq(10);  // Hz

  const auto & goal = goal_handle->get_goal();

  auto feedback = std::make_shared<ActionT::Feedback>();
  auto result = std::make_shared<ActionT::Result>();
  result->result = false;

  const std::array<double, 4> goal_angles = {
    goal->joint_angles[0], goal->joint_angles[1],
    goal->joint_angles[2], goal->joint_angles[3]};

  // send JointMovJ command
  try {
    int8_t speed_j = -1;
    int8_t acc_j = -1;
    int8_t cp = -1;
    if (goal->set_speed_j) {
      speed_j = plugin_utils::clampWithWarning(
        goal->speed_j, plugin_utils::SPEED_J_MIN, plugin_utils::SPEED_J_MAX,
        this->node_logging_if_->get_logger(), "speed_j");
    }
    if (goal->set_acc_j) {
      acc_j = plugin_utils::clampWithWarning(
        goal->acc_j, plugin_utils::ACC_J_MIN, plugin_utils::ACC_J_MAX,
        this->node_logging_if_->get_logger(), "acc_j");
    }
    if (goal->set_cp) {
      cp = plugin_utils::clampWithWarning(
        goal->cp, plugin_utils::CP_MIN, plugin_utils::CP_MAX,
        this->node_logging_if_->get_logger(), "cp");
    }
    this->commander_->jointMovJ(
      goal_angles[0], goal_angles[1], goal_angles[2], goal_angles[3], speed_j, acc_j, cp);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->node_logging_if_->get_logger(), "%s", e.what());
    goal_handle->abort(result);
    return;
  }

  const auto is_goal_reached = [&](
    const std::array<double, 4> & angles,
    const std::array<double, 4> & goal) -> bool {
      const double tolerance_rad = 1. / 180. * M_PI;
      auto is_in_tolerance = [](
        const double val, const double tolerance) -> bool {
          return std::abs(val) < tolerance;
        };

      return is_in_tolerance(angles[0] - goal[0], tolerance_rad) &&
             is_in_tolerance(angles[1] - goal[1], tolerance_rad) &&
             is_in_tolerance(angles[2] - goal[2], tolerance_rad) &&
             is_in_tolerance(angles[3] - goal[3], tolerance_rad)
      ;
    };

  const auto update_angles =
    [&](std::array<double, 4> & angles) -> void
    {
      this->mg400_interface_->realtime_tcp_interface->getCurrentJointStates(angles);
    };


  using RobotMode = mg400_msgs::msg::RobotMode;
  using namespace std::chrono_literals;   // NOLINT
  // TODO(anyone): Should calculate timeout with expectation goal time
  const auto timeout = rclcpp::Duration(10s);
  const auto start = this->node_clock_if_->get_clock()->now();
  update_angles(feedback->current_angles);

  while (!this->mg400_interface_->realtime_tcp_interface->isRobotMode(RobotMode::RUNNING)) {
    if (this->node_clock_if_->get_clock()->now() - start > rclcpp::Duration(300ms)) {
      if (is_goal_reached(feedback->current_angles, goal_angles)) {
        RCLCPP_INFO(
          this->node_logging_if_->get_logger(),
          "Arm is already at the goal.");
        break;
      }

      RCLCPP_ERROR(
        this->node_logging_if_->get_logger(),
        "execution timeout: Robot mode did not become RUNNING.");
      try {
        const std::array<std::vector<int>,
          6> error_id = this->mg400_interface_->dashboard_commander->getErrorId();
        this->mg400_interface_->dashboard_commander->convertToErrorIdMsg(
          error_id,
          result->error_id);
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->node_logging_if_->get_logger(), "Failed to get Error ID: %s", e.what());
      }
      goal_handle->abort(result);
      return;
    }

    if (this->mg400_interface_->realtime_tcp_interface->isRobotMode(RobotMode::ERROR)) {
      RCLCPP_ERROR(
        this->node_logging_if_->get_logger(), "Robot Mode Error while checking becoming RUNNING");
      try {
        const std::array<std::vector<int>, 6> error_id =
          this->mg400_interface_->dashboard_commander->getErrorId();
        this->mg400_interface_->dashboard_commander->convertToErrorIdMsg(
          error_id, result->error_id);
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->node_logging_if_->get_logger(), "Failed to get Error ID: %s", e.what());
      }
      goal_handle->abort(result);
      return;
    }

    control_freq.sleep();
  }

  while (!is_goal_reached(feedback->current_angles, goal_angles) ||
    !this->mg400_interface_->realtime_tcp_interface->isRobotMode(RobotMode::ENABLE))
  {
    if (!this->mg400_interface_->ok()) {
      RCLCPP_ERROR(this->node_logging_if_->get_logger(), "MG400 Connection Error");
      try {
        const std::array<std::vector<int>, 6> error_id =
          this->mg400_interface_->dashboard_commander->getErrorId();
        this->mg400_interface_->dashboard_commander->convertToErrorIdMsg(
          error_id, result->error_id);
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->node_logging_if_->get_logger(), "Failed to get Error ID: %s", e.what());
      }
      goal_handle->abort(result);
      return;
    }

    if (this->mg400_interface_->realtime_tcp_interface->isRobotMode(RobotMode::ERROR)) {
      RCLCPP_ERROR(
        this->node_logging_if_->get_logger(), "Robot Mode Error while checking goal");
      try {
        const std::array<std::vector<int>, 6> error_id =
          this->mg400_interface_->dashboard_commander->getErrorId();
        this->mg400_interface_->dashboard_commander->convertToErrorIdMsg(
          error_id, result->error_id);
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->node_logging_if_->get_logger(), "Failed to get Error ID: %s", e.what());
      }
      goal_handle->abort(result);
      return;
    }

    if (this->node_clock_if_->get_clock()->now() - start > timeout) {
      RCLCPP_ERROR(this->node_logging_if_->get_logger(), "execution timeout");
      try {
        const std::array<std::vector<int>, 6> error_id =
          this->mg400_interface_->dashboard_commander->getErrorId();
        this->mg400_interface_->dashboard_commander->convertToErrorIdMsg(
          error_id, result->error_id);
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->node_logging_if_->get_logger(), "Failed to get Error ID: %s", e.what());
      }
      goal_handle->abort(result);
      return;
    }

    update_angles(feedback->current_angles);
    goal_handle->publish_feedback(feedback);
    control_freq.sleep();
  }

  RCLCPP_INFO(this->node_logging_if_->get_logger(), "Execution succeeded");
  result->result = true;
  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);
  } else {
    goal_handle->succeed(result);
  }
}
}  // namespace mg400_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  mg400_plugin::JointMovJ,
  mg400_plugin_base::MotionApiPluginBase)
