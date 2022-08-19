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
#include <map>

#include <mg400_msgs/srv/clear_error.hpp>
#include <mg400_msgs/srv/reset_robot.hpp>
#include <mg400_msgs/srv/disable_robot.hpp>
#include <mg400_msgs/srv/enable_robot.hpp>
#include <mg400_msgs/srv/tool_do_execute.hpp>
#include <mg400_msgs/srv/speed_factor.hpp>
#include <mg400_msgs/srv/speed_j.hpp>
#include <mg400_msgs/srv/speed_l.hpp>
#include <mg400_msgs/srv/acc_j.hpp>
#include <mg400_msgs/srv/acc_l.hpp>

#include <mg400_msgs/srv/joint_mov_j.hpp>
#include <mg400_msgs/srv/move_jog.hpp>
#include <mg400_msgs/srv/mov_j.hpp>
#include <mg400_msgs/srv/mov_l.hpp>

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
namespace mg400_srv = mg400_msgs::srv;
class ServiceNode : public rclcpp::Node
{
private:
  std::map<std::string, uint8_t> service_level_map;
  std::string service_level;
  uint8_t get_service_level(std::string);

  const std::string prefix_;

  std::unique_ptr<mg400_interface::ErrorMsgGenerator> error_msg_generator_;

  std::unique_ptr<mg400_interface::DashboardTcpInterface> db_tcp_if_;
  std::unique_ptr<mg400_interface::MotionTcpInterface> mt_tcp_if_;
  std::unique_ptr<mg400_interface::RealtimeFeedbackTcpInterface> rt_tcp_if_;

  std::unique_ptr<mg400_interface::DashboardCommander> db_commander_;
  std::unique_ptr<mg400_interface::MotionCommander> mt_commander_;

  rclcpp::TimerBase::SharedPtr js_timer_;
  rclcpp::TimerBase::SharedPtr error_timer_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  rclcpp::Service<mg400_srv::ClearError>::SharedPtr clear_error_srv_;
  rclcpp::Service<mg400_srv::ResetRobot>::SharedPtr reset_robot_srv_;
  rclcpp::Service<mg400_srv::DisableRobot>::SharedPtr disable_robot_srv_;
  rclcpp::Service<mg400_srv::EnableRobot>::SharedPtr enable_robot_srv_;
  rclcpp::Service<mg400_srv::ToolDOExecute>::SharedPtr tool_do_execute_srv_;
  rclcpp::Service<mg400_msgs::srv::SpeedFactor>::SharedPtr speed_factor_srv_;
  //TODO: Uncomment after command implementation.
  // rclcpp::Service<mg400_srv::SpeedJ>::SharedPtr speed_j_srv_;
  // rclcpp::Service<mg400_srv::SpeedL>::SharedPtr speed_l_srv_;
  // rclcpp::Service<mg400_srv::AccJ>::SharedPtr acc_j_srv_;
  // rclcpp::Service<mg400_srv::AccL>::SharedPtr acc_l_srv_;
  // end (Uncomment after command implementation.)

  rclcpp::Service<mg400_srv::JointMovJ>::SharedPtr joint_mov_j_srv_;
  rclcpp::Service<mg400_srv::MoveJog>::SharedPtr move_jog_srv_;
  rclcpp::Service<mg400_srv::MovJ>::SharedPtr mov_j_srv_;
  rclcpp::Service<mg400_srv::MovL>::SharedPtr mov_l_srv_;

public:
  explicit ServiceNode(const rclcpp::NodeOptions &);
  ~ServiceNode();

private:
  void initTcpIf();

  void onJsTimer();

  void onErrorTimer();

  void clearError(
    const mg400_srv::ClearError::Request::SharedPtr,
    mg400_srv::ClearError::Response::SharedPtr);
  void resetRobot(
    const mg400_srv::ResetRobot::Request::SharedPtr,
    mg400_srv::ResetRobot::Response::SharedPtr);
  void disableRobot(
    const mg400_srv::DisableRobot::Request::SharedPtr,
    mg400_srv::DisableRobot::Response::SharedPtr);
  void enableRobot(
    const mg400_srv::EnableRobot::Request::SharedPtr,
    mg400_srv::EnableRobot::Response::SharedPtr);
  void toolDOExecute(
    const mg400_srv::ToolDOExecute::Request::SharedPtr,
    mg400_srv::ToolDOExecute::Response::SharedPtr);
  void speedFactor(
    const mg400_srv::SpeedFactor::Request::SharedPtr,
    mg400_srv::SpeedFactor::Response::SharedPtr);

  // void speedJ(
  //   const mg400_srv::SpeedJ::Request::SharedPtr,
  //   mg400_srv::SpeedJ::Response::SharedPtr);
  // void speedL(
  //   const mg400_srv::SpeedL::Request::SharedPtr,
  //   mg400_srv::SpeedL::Response::SharedPtr);
  // void accJ(
  //   const mg400_srv::AccJ::Request::SharedPtr,
  //   mg400_srv::AccJ::Response::SharedPtr);
  // void accL(
  //   const mg400_srv::AccL::Request::SharedPtr,
  //   mg400_srv::AccL::Response::SharedPtr);

  void jointMovJ(
    const mg400_srv::JointMovJ::Request::SharedPtr,
    mg400_srv::JointMovJ::Response::SharedPtr);
  void moveJog(
    const mg400_srv::MoveJog::Request::SharedPtr,
    mg400_srv::MoveJog::Response::SharedPtr);
  void movJ(
    const mg400_srv::MovJ::Request::SharedPtr,
    mg400_srv::MovJ::Response::SharedPtr);
  void movL(
    const mg400_srv::MovL::Request::SharedPtr,
    mg400_srv::MovL::Response::SharedPtr);
};
}  // namespace mg400_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mg400_node::ServiceNode)
