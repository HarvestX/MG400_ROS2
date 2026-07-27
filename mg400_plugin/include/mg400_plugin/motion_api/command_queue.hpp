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

#ifndef __MG400_PLUGIN_MOTION_API_JOINT_MOV_J_HPP__
#define __MG400_PLUGIN_MOTION_API_JOINT_MOV_J_HPP__

#include <memory>
#include <string>
#include <variant>
#include <vector>

#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include <mg400_common/mg400_ik_util.hpp>
#include <mg400_msgs/action/command_queue.hpp>
#include <mg400_msgs/msg/joint_mov_j.hpp>
#include <mg400_msgs/msg/mov_j.hpp>
#include <mg400_msgs/msg/mov_jio.hpp>
#include <mg400_msgs/msg/mov_l.hpp>
#include <mg400_msgs/msg/mov_lio.hpp>
#include <mg400_msgs/msg/robot_mode.hpp>
#include <mg400_plugin_base/api_plugin_base.hpp>
#include <mg400_plugin/plugin_utils.hpp>
#include <mg400_plugin/tf_manager.hpp>

namespace mg400_plugin
{
class CommandQueue final : public mg400_plugin_base::MotionApiPluginBase
{
public:
  using ActionT = mg400_msgs::action::CommandQueue;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  using GoalReservations = mg400_plugin_base::RegularMotionGoalReservations<
    std::string, std::monostate>;

private:
  rclcpp_action::Server<ActionT>::SharedPtr action_server_;

  mg400_common::MG400IKUtil mg400_ik_util_;
  GoalReservations goal_reservations_;

public:
  void configure(
    const mg400_interface::MotionCommander::SharedPtr,
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr,
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr,
    const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr,
    const rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr,
    const mg400_interface::MG400Interface::SharedPtr)
  override;

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, ActionT::Goal::ConstSharedPtr);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle>);
  void handle_accepted(const std::shared_ptr<GoalHandle>);
  void execute(const std::shared_ptr<GoalHandle>, const GoalReservations::Entry &);

  // functions to handle each command
  void sendMovJ(const mg400_msgs::msg::MovJ &);
  void sendMovL(const mg400_msgs::msg::MovL &);
  void sendJointMovJ(const mg400_msgs::msg::JointMovJ &);
  void sendMovJIO(const mg400_msgs::msg::MovJIO &);
  void sendMovLIO(const mg400_msgs::msg::MovLIO &);
  int sendCommand(
    const std::vector<mg400_msgs::msg::Command> &,
    size_t,
    size_t);

  // functions to validate each target
  bool validateIK(const geometry_msgs::msg::PoseStamped &);
  bool validateAngles(const std::array<double, 4> &);
  bool validateTarget(const std::vector<mg400_msgs::msg::Command> &);

  // utility functions
  bool transformPoseToOrigin(
    const geometry_msgs::msg::PoseStamped &, geometry_msgs::msg::PoseStamped &);
};
}  // namespace mg400_plugin
#endif
