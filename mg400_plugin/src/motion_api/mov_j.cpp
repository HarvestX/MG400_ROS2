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

#include "mg400_plugin/motion_api/mov_j.hpp"

namespace mg400_plugin
{

void MovJ::configure(
  const mg400_interface::MotionCommander::SharedPtr commander,
  const rclcpp::Node::SharedPtr node,
  const mg400_interface::RealtimeFeedbackTcpInterface::SharedPtr rt_if)
{
  if (!this->configure_base(commander, node, rt_if)) {
    return;
  }

  using namespace std::placeholders;  // NOLINT

  this->action_server_ =
    rclcpp_action::create_server<ActionT>(
    this->base_node_.get(), "mov_j",
    std::bind(&MovJ::handle_goal, this, _1, _2),
    std::bind(&MovJ::handle_cancel, this, _1),
    std::bind(&MovJ::handle_accepted, this, _1));
}

rclcpp_action::GoalResponse MovJ::handle_goal(
  const rclcpp_action::GoalUUID &, ActionT::Goal::ConstSharedPtr)
{
  using RobotMode = mg400_msgs::msg::RobotMode;
  if (!this->realtime_tcp_interface_->isRobotMode(RobotMode::ENABLE)) {
    RCLCPP_ERROR(
      this->base_node_->get_logger(), "Robot mode is not enabled");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MovJ::handle_cancel(
  const std::shared_ptr<GoalHandle>)
{
  RCLCPP_INFO(
    this->base_node_->get_logger(), "Received request to cancel goal");
  // TODO(anyone): Should stop movJ
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MovJ::handle_accepted(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  using namespace std::placeholders;  // NOLINT
  std::thread{std::bind(&MovJ::execute, this, _1), goal_handle}.detach();
}


void MovJ::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  rclcpp::Rate control_freq(10);  // Hz

  const auto & goal = goal_handle->get_goal();

  auto feedback = std::make_shared<mg400_msgs::action::MovJ::Feedback>();
  auto result = std::make_shared<mg400_msgs::action::MovJ::Result>();
  result->result = false;

  this->commander_->movJ(
    goal->pose.x, goal->pose.y, goal->pose.z, goal->pose.z);

  const auto is_goal_reached = [&](
    const mg400_msgs::msg::EndPose & pose,
    const mg400_msgs::msg::EndPose & goal) -> bool {
      const double tolerance_mm = 5e-3;  // 5 mm
      const double tolerance_rad = 1.74e-2;  // 1 rad
      auto is_in_tolerance = [](
        const double val, const double tolerance) -> bool {
          return std::abs(val) < tolerance;
        };

      return is_in_tolerance(pose.x - goal.x, tolerance_mm) &&
             is_in_tolerance(pose.y - goal.y, tolerance_mm) &&
             is_in_tolerance(pose.z - goal.z, tolerance_mm) &&
             is_in_tolerance(pose.r - goal.r, tolerance_rad);
    };

  const auto update_pose =
    [&](mg400_msgs::msg::EndPoseStamped & msg) -> void
    {
      msg.header.stamp = this->base_node_->get_clock()->now();
      msg.header.frame_id =
        this->realtime_tcp_interface_->frame_id_prefix + "mg400_origin_link";
      this->realtime_tcp_interface_->getCurrentEndPose(msg.pose);
    };


  using RobotMode = mg400_msgs::msg::RobotMode;
  using namespace std::chrono_literals;   // NOLINT
  // TODO(anyone): Should calculate timeout with expectation goal time
  const auto timeout = rclcpp::Duration(5s);
  const auto start = this->base_node_->get_clock()->now();
  update_pose(feedback->current_pose);

  while (!is_goal_reached(feedback->current_pose.pose, goal->pose)) {
    if (this->realtime_tcp_interface_->isRobotMode(RobotMode::ERROR)) {
      RCLCPP_ERROR(this->base_node_->get_logger(), "Robot Mode Error");
      goal_handle->abort(result);
      return;
    }

    if (this->base_node_->get_clock()->now() - start > timeout) {
      RCLCPP_ERROR(this->base_node_->get_logger(), "execution timeout");
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
  mg400_plugin::MovJ,
  mg400_plugin_base::MotionApiPluginBase)
