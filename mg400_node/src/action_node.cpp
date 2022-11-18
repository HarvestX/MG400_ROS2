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
  this->callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  this->callback_group_executor_.add_callback_group(
    this->callback_group_,
    this->get_node_base_interface());

  using namespace std::placeholders;  // NOLINT
  this->mov_j_client_ =
    this->create_client<mg400_msgs::srv::MovJ>(
    "mov_j", rmw_qos_profile_default, this->callback_group_);
  this->mov_j_action_ =
    rclcpp_action::create_server<mg400_msgs::action::MovJ>(
    this, "mov_j",
    std::bind(&ActionNode::handle_goal, this, _1, _2),
    std::bind(&ActionNode::handle_cancel, this, _1),
    std::bind(&ActionNode::handle_accepted, this, _1));

  this->rm_sub_ =
    this->create_subscription<mg400_msgs::msg::RobotMode>(
    "robot_mode", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&ActionNode::onRm, this, _1));
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
}

void ActionNode::deactivateSub()
{
  if (this->js_sub_) {
    this->js_sub_.reset();
  }
  if (this->current_joint_state_) {
    this->current_joint_state_.reset();
  }
}

void ActionNode::onJs(
  const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  this->js_mutex_.lock();
  this->current_joint_state_ = msg;
  this->js_mutex_.unlock();
}

void ActionNode::onRm(
  const mg400_msgs::msg::RobotMode::ConstSharedPtr msg)
{
  this->current_robot_mode_ = msg;
}

bool ActionNode::updateEndPose(
  mg400_msgs::msg::EndPoseStamped & pose_stamped)
{
  this->js_mutex_.lock();
  const bool ret = mg400_interface::JointHandler::getEndPose(
    this->current_joint_state_, pose_stamped.pose);
  this->js_mutex_.unlock();
  if (ret) {
    pose_stamped.header.frame_id =
      this->prefix_ + "mg400_origin_link";
    pose_stamped.header.stamp =
      this->get_clock()->now();
  }
  return ret;
}

bool ActionNode::callMovJ(const mg400_msgs::msg::EndPose & pose)
{
  if (!this->mov_j_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "\"%s\" is not ready", this->mov_j_client_->get_service_name());
    return false;
  }

  auto req = std::make_shared<mg400_msgs::srv::MovJ::Request>();
  req->pose = pose;
  auto future_result = this->mov_j_client_->async_send_request(req);

  if (this->callback_group_executor_.spin_until_future_complete(
      future_result) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      this->get_logger(), "\"%s\" service client: async_send_request failed",
      this->mov_j_client_->get_service_name());
    return false;
  }

  return true;
}

rclcpp_action::GoalResponse ActionNode::handle_goal(
  const rclcpp_action::GoalUUID &,
  ActionT::Goal::ConstSharedPtr)
{
  using namespace std::chrono_literals;  // NOLINT
  if (!this->mov_j_client_->wait_for_service(3s)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "\"%s\" service client is not available",
      this->mov_j_client_->get_service_name());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (this->current_robot_mode_->robot_mode != RobotMode::ENABLE) {
    RCLCPP_ERROR(this->get_logger(), "Robot not enabled.");
    return rclcpp_action::GoalResponse::REJECT;
  }
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
  rclcpp::Rate control_freq(10);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<mg400_msgs::action::MovJ::Feedback>();
  auto result = std::make_shared<mg400_msgs::action::MovJ::Result>();

  if (!this->callMovJ(goal->pose)) {
    RCLCPP_ERROR(
      this->get_logger(), "\"%s\" service client: failed",
      this->mov_j_client_->get_service_name());
    result->result = false;
    goal_handle->abort(result);
    return;
  }

  const auto timeout = rclcpp::Duration(5s);
  const auto start = this->get_clock()->now();

  auto abort = [this, result, goal_handle]() -> void {
      result->result = false;
      goal_handle->abort(result);
      this->deactivateSub();
    };

  // Wait for first end_pose update
  this->activateSub();
  while (!this->updateEndPose(feedback->current_pose)) {}

  while (!this->isGoalReached(feedback->current_pose.pose, goal->pose)) {
    if (this->current_robot_mode_->robot_mode == RobotMode::ERROR) {
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      abort();
      return;
    }

    if (this->get_clock()->now() - start > timeout) {
      RCLCPP_ERROR(
        this->get_logger(), "\"%s\" execution timeout",
        this->mov_j_client_->get_service_name());
      abort();
      return;
    }

    goal_handle->publish_feedback(feedback);
    control_freq.sleep();
    this->updateEndPose(feedback->current_pose);
  }

  // Check if goal is done
  result->result = true;
  goal_handle->succeed(result);
  this->deactivateSub();
  RCLCPP_INFO(this->get_logger(), "Goal succeeded");
}

bool ActionNode::isGoalReached(
  const mg400_msgs::msg::EndPose & pose,
  const mg400_msgs::msg::EndPose & goal,
  const double tolerance_mm,
  const double tolerance_rad
)
{
  auto is_in_tolerance = [](
    const double val, const double tolerance) -> bool {
      return std::abs(val) < tolerance;
    };

  return is_in_tolerance(pose.x - goal.x, tolerance_mm) &&
         is_in_tolerance(pose.y - goal.y, tolerance_mm) &&
         is_in_tolerance(pose.z - goal.z, tolerance_mm) &&
         is_in_tolerance(pose.r - goal.r, tolerance_rad);
}
}
