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
    "joint_state",
    10, std::bind(&ActionNode::, this, std::placeholders::_1));
  this->robot_mode_sub_ =
    this->create_subscription<mg400_msgs::msg::RobotMode>(
    "robot_mode",
    100, std::bind(&ActionNode::, this, std::placeholders::_1));
  this->mov_j_action_ = rclcpp_action::create_server<mg400_msgs::actioin::MovJ>(
    this,
    "mov_j"
    std::bind(&ActionNode::handle_goal, this, _1, _2),
    std::bind(&ActionNode::handle_cancel, this, _1),
    std::bind(&ActionNode::handle_accepted, this, _1));

  // END Ros Interfaces


  // Robot Initialization
  this->interface_->dashboard_commander->clearError();
}

ActionNode::~ActionNode()
{
  this->interface_->deactivate();
}

void ActionNode::onJsTimer(const sensor_msgs::msg::JointState::SharedPtr msg)
{
}

void ActionNode::onRmTimer(const mg400_msgs::msg::RobotMode::SharedPtr msg)
{
}

rclcpp_action::GoalResponse handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const mg400_msgs::action::MovJ> goal)
{
}

rclcpp_action::CancelResponse handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<mg400_msgs::action::MovJ>> goal_handle)
{
}

void handle_accepted(const std::shared_ptr<> goal_handle)
{
}

void execute(const std::shared_ptr<> goal_handle)
{
}
}
