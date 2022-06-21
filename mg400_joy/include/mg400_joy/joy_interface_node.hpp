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

#include <string>
#include <memory>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mg400_msgs/srv/clear_error.hpp>
#include <mg400_msgs/srv/move_jog.hpp>
#include <mg400_msgs/srv/enable_robot.hpp>
#include <mg400_msgs/srv/disable_robot.hpp>

#include <mg400_interface/commander/motion_commander.hpp>

#include <p9n_interface/p9n_interface.hpp>


namespace mg400_joy
{


class JoyInterfaceNode : public rclcpp::Node
{
private:
  const double TILT_THRESHOLD_ = 5e-2;

  p9n_interface::HW_TYPE hw_type_;

  std::unique_ptr<p9n_interface::PlayStationInterface> p9n_if_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::Client<mg400_msgs::srv::ClearError>::SharedPtr
    mg400_clear_error_clnt_;
  rclcpp::Client<mg400_msgs::srv::EnableRobot>::SharedPtr
    mg400_enable_robot_clnt_;
  rclcpp::Client<mg400_msgs::srv::DisableRobot>::SharedPtr
    mg400_disable_robot_clnt_;
  rclcpp::Client<mg400_msgs::srv::MoveJog>::SharedPtr
    mg400_move_jog_clnt_;

  enum class ROBOT_STATE
  {
    DISABLED,
    ENABLED,
    MOVING
  };

  ROBOT_STATE current_robot_state_;

  enum class SERVICE_STATE
  {
    IN_PROGRESS,
    DONE
  };

  SERVICE_STATE current_service_state_;

public:
  explicit JoyInterfaceNode(const rclcpp::NodeOptions &);
  void onJoy(sensor_msgs::msg::Joy::ConstSharedPtr);

private:
  void callClearError();
  void callEnableRobot();
  void callDisableRobot();
  void callMoveJog(const std::string &);
  bool isStickTilted() const;
  std::string tiltedStick2JogAxis() const;
};
}  // namespace mg400_joy

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mg400_joy::JoyInterfaceNode)
