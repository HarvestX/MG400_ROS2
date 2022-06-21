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

#include <mg400_msgs/srv/clear_error.hpp>
#include <mg400_msgs/srv/disable_robot.hpp>
#include <mg400_msgs/srv/enable_robot.hpp>
#include <mg400_msgs/srv/move_jog.hpp>
#include <mg400_msgs/srv/mov_j.hpp>
#include <mg400_msgs/srv/mov_l.hpp>

#include <mg400_interface/tcp_interface/dashboard_tcp_interface.hpp>
#include <mg400_interface/tcp_interface/motion_tcp_interface.hpp>
#include <mg400_interface/tcp_interface/realtime_feedback_tcp_interface.hpp>

#include <mg400_interface/commander/dashboard_commander.hpp>
#include <mg400_interface/commander/motion_commander.hpp>

#include <mg400_interface/joint_handler.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mg400_node
{
class ServiceNode : public rclcpp::Node
{
private:
  const std::string prefix_;

  std::unique_ptr<mg400_interface::DashboardTcpInterface> db_tcp_if_;
  std::unique_ptr<mg400_interface::MotionTcpInterface> mt_tcp_if_;
  std::unique_ptr<mg400_interface::RealtimeFeedbackTcpInterface> rt_tcp_if_;

  std::unique_ptr<mg400_interface::DashboardCommander> db_commander_;
  std::unique_ptr<mg400_interface::MotionCommander> mt_commander_;

  rclcpp::TimerBase::SharedPtr js_timer_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  rclcpp::Service<mg400_msgs::srv::ClearError>::SharedPtr clear_error_srv_;
  rclcpp::Service<mg400_msgs::srv::DisableRobot>::SharedPtr disable_robot_srv_;
  rclcpp::Service<mg400_msgs::srv::EnableRobot>::SharedPtr enable_robot_srv_;

  rclcpp::Service<mg400_msgs::srv::MoveJog>::SharedPtr move_jog_srv_;
  rclcpp::Service<mg400_msgs::srv::MovJ>::SharedPtr mov_j_srv_;
  rclcpp::Service<mg400_msgs::srv::MovL>::SharedPtr mov_l_srv_;

public:
  explicit ServiceNode(const rclcpp::NodeOptions &);
  ~ServiceNode();

private:
  void initTcpIf();

  void onJsTimer();

  void clearError(
    const mg400_msgs::srv::ClearError::Request::SharedPtr,
    mg400_msgs::srv::ClearError::Response::SharedPtr);
  void disableRobot(
    const mg400_msgs::srv::DisableRobot::Request::SharedPtr,
    mg400_msgs::srv::DisableRobot::Response::SharedPtr);
  void enableRobot(
    const mg400_msgs::srv::EnableRobot::Request::SharedPtr,
    mg400_msgs::srv::EnableRobot::Response::SharedPtr);

  void moveJog(
    const mg400_msgs::srv::MoveJog::Request::SharedPtr,
    mg400_msgs::srv::MoveJog::Response::SharedPtr);
  void movJ(
    const mg400_msgs::srv::MovJ::Request::SharedPtr,
    mg400_msgs::srv::MovJ::Response::SharedPtr);
  void movL(
    const mg400_msgs::srv::MovL::Request::SharedPtr,
    mg400_msgs::srv::MovL::Response::SharedPtr);
};
}  // namespace mg400_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mg400_node::ServiceNode)
