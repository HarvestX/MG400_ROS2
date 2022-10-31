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
  const std::string ip_address =
    this->declare_parameter<std::string>("ip_address", "192.168.1.6");
  RCLCPP_INFO(
    this->get_logger(),
    "Connecting to %s ...",
    ip_address.c_str());

  this->interface_ =
    std::make_unique<mg400_interface::MG400Interface>(ip_address);
  this->interface_->configure();
  this->interface_->activate();

  // ROS Interfaces
  this->joint_state_sub_ =
    this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states",
    10, std::bind(&ActionNode::onJsTimer, this, std::placeholders::_1));
  this->robot_mode_sub_ =
    this->create_subscription<mg400_msgs::msg::RobotMode>(
    "robot_mode",
    100, std::bind(&ActionNode::onRmTimer, this, std::placeholders::_1));
  this->mov_j_client_ =
    this->create_client<mg400_msgs::srv::MovJ>("mov_j");
  this->mov_j_action_ =
    rclcpp_action::create_server<mg400_msgs::action::MovJ>(
    this,
    "mov_j",
    std::bind(&ActionNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ActionNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&ActionNode::handle_accepted, this, std::placeholders::_1));

  // END Ros Interfaces


  // Robot Initialization
  current_robot_state_ = ROBOT_STATE::ERROR;
}

ActionNode::~ActionNode()
{
  this->interface_->deactivate();
}

void ActionNode::onJsTimer(const sensor_msgs::msg::JointState & msg)
{
  geometry_msgs::msg::Pose position = mg400_interface::getEndPoint(msg);
  this->current_robot_position_[0] = position.position.x;
  this->current_robot_position_[1] = position.position.y;
  this->current_robot_position_[2] = position.position.z;

  if (fabs(position.position.x - this->goal_position_[0]) < 1e-3 &&
    fabs(position.position.y - this->goal_position_[1]) < 1e-3 &&
    fabs(position.position.z - this->goal_position_[2]) < 1e-3)
  {
    if (this->current_robot_state_ == ROBOT_STATE::ENABLE ||
      this->current_robot_state_ == ROBOT_STATE::OK)
    {
      this->current_robot_state_ = ROBOT_STATE::OK;
    } else {
      this->current_robot_state_ = ROBOT_STATE::GOAL;
    }
  } else {
    if (this->current_robot_state_ == ROBOT_STATE::ERROR ||
      this->current_robot_state_ == ROBOT_STATE::GOAL)
    {
      this->current_robot_state_ = ROBOT_STATE::ERROR;
    } else {
      this->current_robot_state_ = ROBOT_STATE::ENABLE;
    }
  }
}

void ActionNode::onRmTimer(const mg400_msgs::msg::RobotMode & msg)
{
  this->current_robot_mode_ = msg.robot_mode;
  if (this->current_robot_mode_ == 5) {
    if (this->current_robot_state_ == ROBOT_STATE::GOAL ||
      this->current_robot_state_ == ROBOT_STATE::OK)
    {
      this->current_robot_state_ = ROBOT_STATE::OK;
    } else {
      this->current_robot_state_ = ROBOT_STATE::ENABLE;
    }
  } else {
    if (this->current_robot_state_ == ROBOT_STATE::ERROR ||
      this->current_robot_state_ == ROBOT_STATE::ENABLE)
    {
      this->current_robot_state_ = ROBOT_STATE::ERROR;
    } else {
      this->current_robot_state_ = ROBOT_STATE::GOAL;
    }
  }
}

rclcpp_action::GoalResponse ActionNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const mg400_msgs::action::MovJ::Goal> goal)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Received goal request with (x,y,z,r) = (%f,%f,%f,%f)",
    goal->x, goal->y, goal->z, goal->r);
  this->goal_position_[0] = goal->x;
  this->goal_position_[1] = goal->y;
  this->goal_position_[2] = goal->z;

  callMovJ(goal->x, goal->y, goal->z, goal->r);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionNode::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<mg400_msgs::action::MovJ>> goal_handle)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionNode::handle_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<mg400_msgs::action::MovJ>> goal_handle)
{
  std::thread{std::bind(&ActionNode::execute, this, std::placeholders::_1),
    goal_handle}.detach();
}

void ActionNode::execute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<mg400_msgs::action::MovJ>> goal_handle)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Executing goal");
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<mg400_msgs::action::MovJ::Feedback>();
  auto result = std::make_shared<mg400_msgs::action::MovJ::Result>();

  while (this->current_robot_state_ != ROBOT_STATE::OK) {
    if (goal_handle->is_canceling() || this->current_robot_mode_ == 9) {
      result->result = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
    // Publish feedback
    feedback->current_pose.pose.position.x = this->current_robot_position_[0];
    feedback->current_pose.pose.position.y = this->current_robot_position_[1];
    feedback->current_pose.pose.position.z = this->current_robot_position_[2];

    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(
      this->get_logger(),
      "Publish feedback : x =%f, y=%lf, z=%lf",
      feedback->current_pose.pose.position.x,
      feedback->current_pose.pose.position.y,
      feedback->current_pose.pose.position.z);
    loop_rate.sleep();
  }

  //Check if goal is done
  result->result = true;
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Goal succeeded");

}

//ToDo: Add handling when service submission fails
void ActionNode::callMovJ(
  const double x, const double y, const double z, const double r)
{
  auto req = std::make_shared<mg400_msgs::srv::MovJ::Request>();
  req->x = x;
  req->y = y;
  req->z = z;
  req->r = r;

  this->mov_j_client_->async_send_request(req);
}
}
