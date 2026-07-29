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

#ifndef __MG400_PLUGIN_MOTION_API_MOV_L_HPP__
#define __MG400_PLUGIN_MOTION_API_MOV_L_HPP__

#include <memory>
#include <string>

#include <mg400_common/mg400_ik_util.hpp>
#include <mg400_msgs/action/mov_l.hpp>
#include <mg400_msgs/msg/robot_mode.hpp>
#include <mg400_plugin_base/api_plugin_base.hpp>
#include <mg400_plugin_base/regular_motion_action_reservations.hpp>
#include <mg400_plugin/plugin_utils.hpp>
#include <mg400_plugin/tf_manager.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace mg400_plugin
{
class MovL final : public mg400_plugin_base::MotionApiPluginBase
{
public:
  using ActionT = mg400_msgs::action::MovL;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  using GoalReservations =
    mg400_plugin_base::RegularMotionActionReservations<std::string>;

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
  void execute(const std::shared_ptr<GoalHandle>, GoalReservations::Lease);
};
}  // namespace mg400_plugin
#endif
