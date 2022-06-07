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
#include <chrono>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <mg400_msgs/srv/clear_error.hpp>
#include <mg400_msgs/srv/move_jog.hpp>
#include <mg400_msgs/srv/enable_robot.hpp>
#include <mg400_msgs/srv/disable_robot.hpp>


namespace mg400_joy
{

enum class JOY_TYPE
{
  DUALSHOCK3,
  DUALSHOCK4,
  DUALSENSE,
};


enum class BUTTONS_DUALSENSE
{
  CROSS = 0,
  CIRCLE,
  TRIANGLE,
  SQUARE,
  L1,
  R1,
  L2,
  R2,
  SELECT,
  START,
  PS
};

enum class AXES_DUALSENSE
{
  LSTICK_X = 0,
  LSTICK_Y,
  L2,
  RSTICK_X,
  RSTICK_Y,
  R2,
  DPAD_X,
  DPAD_Y,
};

typedef struct
{
  bool square;
  bool circle;
  bool triangle;
  bool cross;

  bool L1;
  bool R1;
  bool R2;
  bool L2;

  bool select;
  bool start;
  bool PS;

  float d_pad_x;
  float d_pad_y;
  float lstick_x;
  float lstick_y;
  float rstick_x;
  float rstick_y;

  float R2_analog;
  float L2_analog;
} JoyButton;

class JoyComponent : public rclcpp::Node
{
private:
  JOY_TYPE joy_type = JOY_TYPE::DUALSENSE;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  std::unique_ptr<JoyButton> button_;

  rclcpp::Client<mg400_msgs::srv::ClearError>::SharedPtr clear_error_clnt_;
  rclcpp::Client<mg400_msgs::srv::MoveJog>::SharedPtr move_jog_clnt_;
  rclcpp::Client<mg400_msgs::srv::EnableRobot>::SharedPtr enable_robot_clnt_;
  rclcpp::Client<mg400_msgs::srv::DisableRobot>::SharedPtr disable_robot_clnt_;

  uint64_t robot_state;

public:
  explicit JoyComponent(const rclcpp::NodeOptions &);

private:
  void joyClearError();
  void joyEnableDisable();
  void joyMoveJog();
  void joyCallback(const sensor_msgs::msg::Joy::UniquePtr);
  void displayInfo() const noexcept;
  void updateButton(
    std::reference_wrapper<const sensor_msgs::msg::Joy::UniquePtr>);
};
}  // namespace mg400_joy
