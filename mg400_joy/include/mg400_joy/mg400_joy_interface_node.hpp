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

#ifndef __MG400_JOY_MG400_JOY_INTERFACE_NODE_HPP__
#define __MG400_JOY_MG400_JOY_INTERFACE_NODE_HPP__

#include <functional>
#include <future>
#include <string>
#include <memory>
#include <type_traits>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <mg400_msgs/msg/robot_mode.hpp>
#include <mg400_msgs/srv/disable_robot.hpp>
#include <mg400_msgs/srv/enable_robot.hpp>
#include <mg400_msgs/srv/move_jog.hpp>
#include <mg400_msgs/srv/reset_robot.hpp>

#include <mg400_interface/commander/motion_commander.hpp>

#include <p9n_interface/p9n_interface.hpp>


namespace mg400_joy
{
using namespace std::chrono_literals; // NOLINT
class MG400JoyInterfaceNode : public rclcpp::Node
{
private:
  using RobotMode = mg400_msgs::msg::RobotMode;
  const double TILT_THRESHOLD_ = 5e-2;

  p9n_interface::HW_TYPE hw_type_;

  std::unique_ptr<p9n_interface::PlayStationInterface> p9n_if_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::Subscription<RobotMode>::SharedPtr rm_sub_;
  RobotMode::ConstSharedPtr current_robot_mode_;

  rclcpp::Client<mg400_msgs::srv::ResetRobot>::SharedPtr
    mg400_reset_robot_clnt_;
  rclcpp::Client<mg400_msgs::srv::EnableRobot>::SharedPtr
    mg400_enable_robot_clnt_;
  rclcpp::Client<mg400_msgs::srv::DisableRobot>::SharedPtr
    mg400_disable_robot_clnt_;
  rclcpp::Client<mg400_msgs::srv::MoveJog>::SharedPtr
    mg400_move_jog_clnt_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  enum class JogMode : int64_t
  {
    JOINT = 0,
    LINEAR,
  };
  JogMode current_jog_mode_;

public:
  explicit MG400JoyInterfaceNode(const rclcpp::NodeOptions &);

private:
  void onJoy(const sensor_msgs::msg::Joy::ConstSharedPtr);
  bool isEnabled();

  void callResetRobot();
  void callEnableRobot();
  void callDisableRobot();
  void callMoveJog(const std::string &);
  bool tiltedStick2JogAxis(std::string &) const;
};
}  // namespace mg400_joy

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mg400_joy::MG400JoyInterfaceNode)
#endif
