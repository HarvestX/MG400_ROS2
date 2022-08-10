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

#include <mg400_msgs/srv/enable_robot.hpp>
#include <mg400_msgs/srv/joint_mov_j.hpp>

#include "mg400_interface/command_utils.hpp"

#include <mg400_interface/tcp_interface/dashboard_tcp_interface.hpp>
#include <mg400_interface/tcp_interface/motion_tcp_interface.hpp>
#include <mg400_interface/tcp_interface/realtime_feedback_tcp_interface.hpp>

#include <mg400_interface/commander/dashboard_commander.hpp>
#include <mg400_interface/commander/motion_commander.hpp>

#include <mg400_interface/joint_handler.hpp>
#include <mg400_interface/error_msg_generator.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mg400_node
{
class JointHandlerNode : public rclcpp::Node
{
private:
  const std::string prefix_;

  std::unique_ptr<mg400_interface::ErrorMsgGenerator> error_msg_generator_;

  std::unique_ptr<mg400_interface::DashboardTcpInterface> db_tcp_if_;
  std::unique_ptr<mg400_interface::MotionTcpInterface> mt_tcp_if_;
  std::unique_ptr<mg400_interface::RealtimeFeedbackTcpInterface> rt_tcp_if_;

  std::unique_ptr<mg400_interface::DashboardCommander> db_commander_;
  std::unique_ptr<mg400_interface::MotionCommander> mt_commander_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr js_timer_;
  rclcpp::TimerBase::SharedPtr error_timer_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  rclcpp::Service<mg400_msgs::srv::JointMovJ>::SharedPtr joint_mov_j_srv_;

public:
  explicit JointHandlerNode(const rclcpp::NodeOptions &);
  ~JointHandlerNode();
  void onJoint(sensor_msgs::msg::JointState::ConstSharedPtr);

private:
  void initTcpIf();

  void onJsTimer();

  void onErrorTimer();

  void callJointMovJ(
    const double, const double, const double,
    const double, const double, const double);
};
}  // namespace mg400_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mg400_node::JointHandlerNode)