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

#include "mg400_plugin/motion_api/mov_l.hpp"

namespace mg400_plugin
{

void MovL::configure(
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

  // Initialize TF manager if not already initialized
  TFManager & tf_manager = TFManager::getInstance();
  if (!tf_manager.isInitialized()) {
    tf_manager.initialize(node_clock_if);
  }

  using namespace std::placeholders;  // NOLINT

  this->action_server_ =
    rclcpp_action::create_server<ActionT>(
    this->node_base_if_,
    this->node_clock_if_,
    this->node_logging_if_,
    this->node_waitable_if_,
    "mov_l",
    std::bind(&MovL::handle_goal, this, _1, _2),
    std::bind(&MovL::handle_cancel, this, _1),
    std::bind(&MovL::handle_accepted, this, _1),
    rcl_action_server_get_default_options(),
    this->node_base_if_->get_default_callback_group());
}

rclcpp_action::GoalResponse MovL::handle_goal(
  const rclcpp_action::GoalUUID & uuid, ActionT::Goal::ConstSharedPtr /*goal*/)
{
  if (!this->mg400_interface_->ok()) {
    RCLCPP_ERROR(
      this->node_logging_if_->get_logger(), "MG400 is not connected");
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

rclcpp_action::CancelResponse MovL::handle_cancel(
  const std::shared_ptr<GoalHandle>/*unused*/)
{
  RCLCPP_INFO(
    this->node_logging_if_->get_logger(), "Received request to cancel goal");
  // No verified safe-stop command exists here. Execution therefore keeps the
  // lease until the robot reaches its terminal state after cancellation.
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MovL::handle_accepted(
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
  std::shared_ptr<GoalReservations::Lease> active_lease;
  try {
    active_lease = std::make_shared<GoalReservations::Lease>(std::move(*lease));
    std::thread{
      [this, goal_handle, active_lease]() {
        try {
          this->execute(goal_handle);
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


void MovL::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  rclcpp::Rate control_freq(10);  // Hz

  const auto & goal = goal_handle->get_goal();

  auto feedback = std::make_shared<ActionT::Feedback>();
  auto result = std::make_shared<ActionT::Result>();
  result->result = false;

  geometry_msgs::msg::PoseStamped tf_goal;
  try {
    TFManager & tf_manager = TFManager::getInstance();
    auto tf_buffer = tf_manager.getBuffer();
    const auto transform = tf_buffer->lookupTransform(
      this->mg400_interface_->realtime_tcp_interface->frame_id_prefix + "mg400_origin_link",
      goal->pose.header.frame_id, rclcpp::Time(0));
    tf2::doTransform(goal->pose, tf_goal, transform);
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(this->node_logging_if_->get_logger(), "%s", e.what());
    goal_handle->abort(result);
    return;
  }

  // check if the requested goal is inside the mg400 range
  // by solving inverse kinematics and see the angles of each joints.
  std::vector<double> tool_vec;
  tool_vec.push_back(tf_goal.pose.position.x);  // px
  tool_vec.push_back(tf_goal.pose.position.y);  // py
  tool_vec.push_back(tf_goal.pose.position.z);  // pz
  tool_vec.push_back(tf2::getYaw(tf_goal.pose.orientation));  // r in radian
  std::vector<double> angles;
  try {
    angles = this->mg400_ik_util_.InverseKinematics(tool_vec);
    RCLCPP_INFO(
      this->node_logging_if_->get_logger(), "Joint angles = {%f, %f, %f, %f}",
      angles[0] * 180.0 / M_PI, angles[1] * 180.0 / M_PI, angles[2] * 180.0 / M_PI,
      angles[3] * 180.0 / M_PI);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->node_logging_if_->get_logger(), "%s", e.what());
    // ErrorID 18: Inverse kinematics error with result out of working area
    result->result = false;
    result->error_id.controller.ids.emplace_back(18);
    goal_handle->abort(result);
    return;
  }

  // send MovL command
  try {
    int8_t speed_l = -1;
    int8_t acc_l = -1;
    int8_t cp = -1;
    if (goal->set_speed_l) {
      speed_l = plugin_utils::clampWithWarning(
        goal->speed_l, plugin_utils::SPEED_L_MIN, plugin_utils::SPEED_L_MAX,
        this->node_logging_if_->get_logger(), "speed_l");
    }
    if (goal->set_acc_l) {
      acc_l = plugin_utils::clampWithWarning(
        goal->acc_l, plugin_utils::ACC_L_MIN, plugin_utils::ACC_L_MAX,
        this->node_logging_if_->get_logger(), "acc_l");
    }
    if (goal->set_cp) {
      cp =
        plugin_utils::clampWithWarning(
        goal->cp, plugin_utils::CP_MIN, plugin_utils::CP_MAX,
        this->node_logging_if_->get_logger(), "cp");
    }
    this->commander_->movL(
      tf_goal.pose.position.x, tf_goal.pose.position.y,
      tf_goal.pose.position.z,
      tf2::getYaw(tf_goal.pose.orientation),
      speed_l, acc_l, cp);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->node_logging_if_->get_logger(), "%s", e.what());
    goal_handle->abort(result);
    return;
  }

  const auto is_goal_reached = [&](
    const geometry_msgs::msg::Pose & pose,
    const geometry_msgs::msg::Pose & goal) -> bool {
      const double tolerance_m = 5e-3;   // 5 mm
      const double tolerance_rad = 1.74e-2;   // pi/2 *10e-2
      auto is_in_tolerance = [](
        const double val, const double tolerance) -> bool {
          return std::abs(val) < tolerance;
        };

      return is_in_tolerance(pose.position.x - goal.position.x, tolerance_m) &&
             is_in_tolerance(pose.position.y - goal.position.y, tolerance_m) &&
             is_in_tolerance(pose.position.z - goal.position.z, tolerance_m) &&
             is_in_tolerance(
        tf2::getYaw(pose.orientation) - tf2::getYaw(goal.orientation),
        tolerance_rad);
    };

  const auto update_pose =
    [&](geometry_msgs::msg::PoseStamped & msg) -> void
    {
      msg.header.stamp = this->node_clock_if_->get_clock()->now();
      msg.header.frame_id =
        this->mg400_interface_->realtime_tcp_interface->frame_id_prefix + "mg400_origin_link";
      this->mg400_interface_->realtime_tcp_interface->getCurrentEndPose(msg.pose);
    };


  using RobotMode = mg400_msgs::msg::RobotMode;
  using namespace std::chrono_literals;   // NOLINT
  // TODO(anyone): Should calculate timeout with expectation goal time
  const auto timeout = rclcpp::Duration(10s);
  const auto start = this->node_clock_if_->get_clock()->now();
  update_pose(feedback->current_pose);

  while (!this->mg400_interface_->realtime_tcp_interface->isRobotMode(RobotMode::RUNNING)) {
    if (this->node_clock_if_->get_clock()->now() - start > rclcpp::Duration(300ms)) {
      if (is_goal_reached(feedback->current_pose.pose, tf_goal.pose)) {
        RCLCPP_INFO(
          this->node_logging_if_->get_logger(),
          "Arm is already at the goal.");
        break;
      }

      RCLCPP_ERROR(
        this->node_logging_if_->get_logger(),
        "execution timeout: Robot mode did not become RUNNING.");
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

  while (!is_goal_reached(feedback->current_pose.pose, tf_goal.pose) ||
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
        this->node_logging_if_->get_logger(), "Robot Mode Error while checking reaching goal");
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

    update_pose(feedback->current_pose);
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
  mg400_plugin::MovL,
  mg400_plugin_base::MotionApiPluginBase)
