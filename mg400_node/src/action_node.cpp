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


#include "mg400_node/action_node.hpp"
#include "mg400_interface/joint_handler.hpp"


namespace mg400_node
{
ActionNode::ActionNode(const rclcpp::NodeOptions & options)
: Node("action_node", options),
  prefix_(this->declare_parameter("prefix", ""))
{
  // ROS Interfaces ---------------------------------------------
  using namespace std::placeholders;  // NOLINT
  this->mov_j_client_ =
    this->create_client<mg400_msgs::srv::MovJ>("mov_j");
  this->mov_j_action_ =
    rclcpp_action::create_server<mg400_msgs::action::MovJ>(
    this, "mov_j",
    std::bind(&ActionNode::handle_goal, this, _1, _2),
    std::bind(&ActionNode::handle_cancel, this, _1),
    std::bind(&ActionNode::handle_accepted, this, _1));
  // End ROS Interfaces------------------------------------------
}

ActionNode::~ActionNode()
{
}

void ActionNode::activateSub()
{
  using namespace std::placeholders;  // NOLINT
  if (!this->js_sub_) {
    this->js_sub_ =
      this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ActionNode::onJs, this, _1));
  }
  if (!this->rm_sub_) {
    this->rm_sub_ =
      this->create_subscription<mg400_msgs::msg::RobotMode>(
      "robot_mode", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ActionNode::onRm, this, _1));
  }
}

void ActionNode::deactivateSub()
{
  if (this->js_sub_) {
    this->js_sub_.reset();
  }
  if (this->rm_sub_) {
    this->rm_sub_.reset();
  }
}

void ActionNode::onJs(const sensor_msgs::msg::JointState & msg)
{
  this->current_end_pose_ = mg400_interface::getEndPose(msg);
}

void ActionNode::onRm(const mg400_msgs::msg::RobotMode & msg)
{
  this->current_robot_mode_ = msg;
}

rclcpp_action::GoalResponse ActionNode::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const mg400_msgs::action::MovJ::Goal> goal)
{
  using namespace std::chrono_literals;  // NOLINT
  if (!this->mov_j_client_->wait_for_service(3s)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "\"%s\" service client is not available",
      this->mov_j_client_->get_service_name());
    return rclcpp_action::GoalResponse::REJECT;
  }
  auto req = std::make_unique<mg400_msgs::srv::MovJ::Request>();
  req->pose = goal->pose;
  this->mov_j_client_->async_send_request(std::move(req));
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionNode::handle_cancel(
  const std::shared_ptr<GoalHandle>)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionNode::handle_accepted(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  using namespace std::placeholders;  // NOLINT
  std::thread{std::bind(&ActionNode::execute, this, _1), goal_handle}.detach();
}

void ActionNode::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  rclcpp::Rate loop_rate(10);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<mg400_msgs::action::MovJ::Feedback>();
  auto result = std::make_shared<mg400_msgs::action::MovJ::Result>();
  this->activateSub();

  while (
    this->current_robot_mode_.robot_mode == RobotMode::RUNNING ||
    !this->isGoalReached(goal->pose))
  {
    if (goal_handle->is_canceling() ||
      this->current_robot_mode_.robot_mode == RobotMode::ERROR)
    {
      result->result = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      this->deactivateSub();
      return;
    }

    // Publish feedback
    feedback->current_pose.header.frame_id =
      this->prefix_ + "mg400_origin_link";
    feedback->current_pose.header.stamp =
      this->get_clock()->now();
    feedback->current_pose.pose = this->current_end_pose_;

    goal_handle->publish_feedback(feedback);
    loop_rate.sleep();
  }

  // Check if goal is done
  result->result = true;
  goal_handle->succeed(result);
  this->deactivateSub();
  RCLCPP_INFO(this->get_logger(), "Goal succeeded");
}

bool ActionNode::isGoalReached(
  const mg400_msgs::msg::EndPose & goal,
  const double tolerance)
{
  auto is_in_tolerance = [tolerance](
    const double val) -> bool {
      return std::abs(val) < tolerance;
    };

  return is_in_tolerance(this->current_end_pose_.x - goal.x) &&
         is_in_tolerance(this->current_end_pose_.y - goal.y) &&
         is_in_tolerance(this->current_end_pose_.z - goal.z);
}
}
