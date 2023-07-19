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

  // setup for using tf
  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_clock_if_->get_clock());
  this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

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
  const rclcpp_action::GoalUUID &, ActionT::Goal::ConstSharedPtr)
{
  if (!this->mg400_interface_->ok()) {
    RCLCPP_ERROR(
      this->node_logging_if_->get_logger(), "MG400 is not connected");
    return rclcpp_action::GoalResponse::REJECT;
  }

  using RobotMode = mg400_msgs::msg::RobotMode;
  if (!this->mg400_interface_->realtime_tcp_interface->isRobotMode(RobotMode::ENABLE)) {
    RCLCPP_ERROR(
      this->node_logging_if_->get_logger(), "Robot mode is not enabled");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MovL::handle_cancel(
  const std::shared_ptr<GoalHandle>)
{
  RCLCPP_INFO(
    this->node_logging_if_->get_logger(), "Received request to cancel goal");
  // TODO(anyone): Should stop movL
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MovL::handle_accepted(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  using namespace std::placeholders;  // NOLINT
  std::thread{std::bind(&MovL::execute, this, _1), goal_handle}.detach();
}


void MovL::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  rclcpp::Rate control_freq(10);  // Hz

  const auto & goal = goal_handle->get_goal();

  // tf (from goal->pose to tf_goal)
  geometry_msgs::msg::PoseStamped tf_goal;
  try{
    const auto transform = this->tf_buffer_->lookupTransform(
      this->mg400_interface_->realtime_tcp_interface->frame_id_prefix + "mg400_origin_link",
      goal->pose.header.frame_id, rclcpp::Time(0));
    tf2::doTransform(goal->pose, tf_goal, transform);
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(this->node_logging_if_->get_logger(), e.what());
    return;
  }

  auto feedback = std::make_shared<ActionT::Feedback>();
  auto result = std::make_shared<ActionT::Result>();
  result->result = false;

  this->commander_->movL(
    tf_goal.pose.position.x, tf_goal.pose.position.y, tf_goal.pose.position.z,
    tf2::getYaw(tf_goal.pose.orientation));

  const auto is_goal_reached = [&](
    const geometry_msgs::msg::Pose & pose,
    const geometry_msgs::msg::Pose & goal) -> bool {
      const double tolerance_mm = 5e-3;  // 5 mm
      const double tolerance_rad = 1.74e-2;  // 1 rad
      auto is_in_tolerance = [](
        const double val, const double tolerance) -> bool {
          return std::abs(val) < tolerance;
        };

      return is_in_tolerance(pose.position.x - goal.position.x, tolerance_mm) &&
             is_in_tolerance(pose.position.y - goal.position.y, tolerance_mm) &&
             is_in_tolerance(pose.position.z - goal.position.z, tolerance_mm) &&
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
  const auto timeout = rclcpp::Duration(5s);
  const auto start = this->node_clock_if_->get_clock()->now();
  update_pose(feedback->current_pose);

  while (!is_goal_reached(feedback->current_pose.pose, tf_goal.pose)) {
    if (!this->mg400_interface_->ok()) {
      RCLCPP_ERROR(this->node_logging_if_->get_logger(), "MG400 Connection Error");
      goal_handle->abort(result);
      return;
    }

    if (this->mg400_interface_->realtime_tcp_interface->isRobotMode(RobotMode::ERROR)) {
      RCLCPP_ERROR(this->node_logging_if_->get_logger(), "Robot Mode Error");
      goal_handle->abort(result);
      return;
    }

    if (this->node_clock_if_->get_clock()->now() - start > timeout) {
      RCLCPP_ERROR(this->node_logging_if_->get_logger(), "execution timeout");
      goal_handle->abort(result);
      return;
    }

    update_pose(feedback->current_pose);
    goal_handle->publish_feedback(feedback);
    control_freq.sleep();
  }

  result->result = true;
  goal_handle->succeed(result);
}
}  // namespace mg400_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  mg400_plugin::MovL,
  mg400_plugin_base::MotionApiPluginBase)
