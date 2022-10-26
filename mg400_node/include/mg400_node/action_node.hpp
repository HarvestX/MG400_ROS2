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

#include <mg400_msgs/srv/acc_j.hpp>
#include <mg400_msgs/srv/acc_l.hpp>
#include <mg400_msgs/srv/clear_error.hpp>
#include <mg400_msgs/srv/disable_robot.hpp>
#include <mg400_msgs/srv/enable_robot.hpp>
#include <mg400_msgs/srv/reset_robot.hpp>
#include <mg400_msgs/srv/speed_factor.hpp>
#include <mg400_msgs/srv/speed_j.hpp>
#include <mg400_msgs/srv/speed_l.hpp>
#include <mg400_msgs/srv/tool.hpp>
#include <mg400_msgs/srv/tool_do_execute.hpp>

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
namespace mg400_action = mg400_msgs::action;
class ActionNode : public rclcpp::Node
{
private:
  std::unique_ptr<mg400_interface::MG400Interface> interface_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<mg400_msgs::msg::RobotMode>::SharedPtr robot_mode_sub_;

  rclcpp_action::Server<mg400_action::MovJ>::SharedPtr mov_j_action_;

public:
  explicit ActionNode(const rclcpp::NodeOptions &);
  ~ActionNode();

private:
  void onJsTimer(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onRmTimer(const mg400_msgs::msg::RobotMode::SharedPtr msg);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const mg400_msgs::action::MovJ>);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<mg400_msgs::action::MovJ>>);
  void handle_accepted(const std::shared_ptr<>);
  void execute(const std::shared_ptr<>);
};
}  // namespace mg400_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mg400_node::ActionNode)
