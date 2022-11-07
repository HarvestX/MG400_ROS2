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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <mg400_msgs/srv/joint_mov_j.hpp>
#include <mg400_msgs/srv/move_jog.hpp>
#include <mg400_msgs/srv/mov_j.hpp>
#include <mg400_msgs/srv/mov_l.hpp>

#include <mg400_msgs/action/mov_j.hpp>

#include <mg400_interface/mg400_interface.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <mg400_msgs/msg/robot_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>


namespace mg400_node
{
using ActionT = mg400_msgs::action::MovJ;
using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

using namespace std::chrono_literals;  // NOLINT

class ActionNode : public rclcpp::Node
{
public:
  using RobotMode = mg400_msgs::msg::RobotMode;

private:
  const std::string prefix_;
  std::mutex js_mutex_;

  mg400_msgs::msg::RobotMode::ConstSharedPtr current_robot_mode_;
  sensor_msgs::msg::JointState::ConstSharedPtr current_joint_state_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    js_sub_;
  rclcpp::Subscription<mg400_msgs::msg::RobotMode>::SharedPtr
    rm_sub_;

  rclcpp::Client<mg400_msgs::srv::MovJ>::SharedPtr mov_j_client_;
  rclcpp_action::Server<ActionT>::SharedPtr mov_j_action_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

public:
  explicit ActionNode(const rclcpp::NodeOptions &);
  ~ActionNode();

private:
  void activateSub();
  void deactivateSub();
  void onJs(const sensor_msgs::msg::JointState::ConstSharedPtr);
  void onRm(const mg400_msgs::msg::RobotMode::ConstSharedPtr);

  bool updateEndPose(mg400_msgs::msg::EndPoseStamped &);
  bool callMovJ(const mg400_msgs::msg::EndPose &);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, ActionT::Goal::ConstSharedPtr);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle>);
  void handle_accepted(const std::shared_ptr<GoalHandle>);
  void execute(const std::shared_ptr<GoalHandle>);

  bool isGoalReached(
    const mg400_msgs::msg::EndPose &,
    const mg400_msgs::msg::EndPose &,
    const double = 5e-3,  // 5 mm
    const double = 1.74e-2  // 1 rad
  );

};
}  // namespace mg400_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mg400_node::ActionNode)
