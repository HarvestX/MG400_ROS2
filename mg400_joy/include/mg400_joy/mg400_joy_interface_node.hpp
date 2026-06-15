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

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <mg400_msgs/msg/realtime_feedback.hpp>
#include <mg400_msgs/msg/robot_mode.hpp>
#include <mg400_msgs/msg/servo_j.hpp>
#include <mg400_msgs/msg/servo_p.hpp>
#include <mg400_msgs/srv/disable_robot.hpp>
#include <mg400_msgs/srv/enable_robot.hpp>
#include <mg400_msgs/srv/reset_robot.hpp>
#include <mg400_msgs/srv/servo_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>

namespace mg400_joy
{
class MG400JoyInterfaceNode : public rclcpp::Node
{
private:
  enum class ServoControlType
  {
    SERVO_J = 0,
    SERVO_P,
  };

  enum class ControllerCheckStage
  {
    DISABLED = 0,
    WAITING_CROSS,
    WAITING_RELEASE_AFTER_CROSS,
    WAITING_L1,
    WAITING_RELEASE_AFTER_L1,
    WAITING_R1,
    WAITING_RELEASE_AFTER_R1,
    WAITING_SELECT,
    WAITING_RELEASE_AFTER_SELECT,
    WAITING_START,
    WAITING_RELEASE_AFTER_START,
    WAITING_PS,
    WAITING_RELEASE_AFTER_PS,
    WAITING_LEFT_STICK_X,
    WAITING_RELEASE_AFTER_LEFT_STICK_X,
    WAITING_LEFT_STICK_Y,
    WAITING_RELEASE_AFTER_LEFT_STICK_Y,
    WAITING_RIGHT_STICK_X,
    WAITING_RELEASE_AFTER_RIGHT_STICK_X,
    WAITING_RIGHT_STICK_Y,
    WAITING_RELEASE_AFTER_RIGHT_STICK_Y,
    WAITING_DPAD_UP,
    PASSED,
    FAILED,
  };

  struct JoyMapping
  {
    struct AxisMapping
    {
      int index = -1;
      double positive_value = 1.0;
    };

    int button_cross = 0;
    int button_l1 = 4;
    int button_r1 = 5;
    int button_select = 8;
    int button_start = 9;
    int button_ps = 10;
    int button_dpad_up = -1;
    AxisMapping axis_stick_lx = {0, -1.0};
    AxisMapping axis_stick_ly = {1, -1.0};
    AxisMapping axis_stick_rx = {3, -1.0};
    AxisMapping axis_stick_ry = {4, -1.0};
    int axis_dpad_y = 7;
    double dpad_up_axis_value = 1.0;
  };

  using RealtimeFeedback = mg400_msgs::msg::RealtimeFeedback;
  using RobotMode = mg400_msgs::msg::RobotMode;
  using ServoJ = mg400_msgs::msg::ServoJ;
  using ServoP = mg400_msgs::msg::ServoP;
  using ServoModeService = mg400_msgs::srv::ServoMode;

  std::string joy_mapping_file_;
  JoyMapping joy_mapping_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<RobotMode>::SharedPtr robot_mode_sub_;
  rclcpp::Subscription<RealtimeFeedback>::SharedPtr realtime_feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr servo_mode_enabled_sub_;

  rclcpp::Publisher<ServoJ>::SharedPtr servo_j_pub_;
  rclcpp::Publisher<ServoP>::SharedPtr servo_p_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr joy_control_active_pub_;

  rclcpp::Client<mg400_msgs::srv::ResetRobot>::SharedPtr reset_robot_clnt_;
  rclcpp::Client<mg400_msgs::srv::EnableRobot>::SharedPtr enable_robot_clnt_;
  rclcpp::Client<mg400_msgs::srv::DisableRobot>::SharedPtr disable_robot_clnt_;
  rclcpp::Client<ServoModeService>::SharedPtr servo_mode_clnt_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::TimerBase::SharedPtr servo_publish_timer_;
  rclcpp::TimerBase::SharedPtr joy_control_active_timer_;

  mutable std::mutex state_mutex_;
  RobotMode::_robot_mode_type current_robot_mode_;
  bool servo_mode_enabled_;
  bool has_actual_state_;
  geometry_msgs::msg::Pose current_pose_;
  double current_pose_yaw_;
  std::array<double, 4> current_joint_state_;

  ServoControlType servo_control_type_;
  bool servo_pose_target_initialized_;
  bool servo_joint_target_initialized_;
  geometry_msgs::msg::Pose servo_target_pose_;
  double servo_target_yaw_;
  std::array<double, 4> servo_joint_target_;
  std::array<double, 4> servo_pose_axes_;
  std::array<double, 4> servo_joint_axes_;
  rclcpp::Time last_servo_target_update_time_;
  rclcpp::Time last_joy_time_;
  rclcpp::Time last_button_action_time_;

  ControllerCheckStage controller_check_stage_;
  rclcpp::Time controller_check_start_time_;
  rclcpp::Time last_controller_check_prompt_time_;
  sensor_msgs::msg::Joy controller_check_baseline_;
  bool controller_check_baseline_ready_;

  std::string servo_pose_frame_id_;
  double linear_speed_mps_;
  double angular_speed_radps_;
  double joint_speed_radps_;
  double stick_deadzone_;
  double controller_axis_threshold_;
  int service_timeout_ms_;
  int button_cooldown_ms_;
  int joy_timeout_ms_;
  double controller_check_timeout_sec_;

public:
  explicit MG400JoyInterfaceNode(const rclcpp::NodeOptions &);

private:
  void onJoy(sensor_msgs::msg::Joy::ConstSharedPtr);
  void onRealtimeFeedback(RealtimeFeedback::ConstSharedPtr);
  void onServoModeEnabled(std_msgs::msg::Bool::ConstSharedPtr);
  void onServoPublishTimer();
  void publishJoyControlActive();

  bool loadJoyMappingFile(const std::string &);
  bool validateJoyMappingNoOverlap(const JoyMapping &, std::string &) const;
  bool handleControllerCheck(sensor_msgs::msg::Joy::ConstSharedPtr);
  void ensureControllerCheckBaseline(const sensor_msgs::msg::Joy &);
  bool controllerCheckInputActive(const sensor_msgs::msg::Joy &) const;
  bool learnButtonInput(const sensor_msgs::msg::Joy &, int &, const std::string &);
  bool learnAxisInput(
    const sensor_msgs::msg::Joy &, JoyMapping::AxisMapping &, const std::string &);
  bool learnDpadUpInput(const sensor_msgs::msg::Joy &);
  int singleActiveButtonIndex(const sensor_msgs::msg::Joy &) const;
  int singleActiveAxisIndex(const sensor_msgs::msg::Joy &) const;
  bool anyAxisChangedFromBaseline(const sensor_msgs::msg::Joy &) const;
  bool buttonPressed(const sensor_msgs::msg::Joy &, int) const;
  bool anyButtonPressed(const sensor_msgs::msg::Joy &) const;
  double axisValue(const sensor_msgs::msg::Joy &, int) const;
  double baselineAxisValue(int) const;
  double normalizedAxisValue(const sensor_msgs::msg::Joy &, int) const;
  double orientedAxisValue(const sensor_msgs::msg::Joy &, const JoyMapping::AxisMapping &) const;
  double axisDeltaFromBaseline(const sensor_msgs::msg::Joy &, int) const;
  bool axisPressed(const sensor_msgs::msg::Joy &, int, double) const;
  bool dpadUpPressed(const sensor_msgs::msg::Joy &) const;
  void promptControllerCheck(const std::string &);
  void failControllerCheck(const std::string &);

  bool handleButtonActions(const sensor_msgs::msg::Joy &);
  bool buttonActionReady() const;
  void markButtonAction();

  void updateServoAxesFromJoy(const sensor_msgs::msg::Joy &);
  void zeroServoAxes();
  double applyDeadzone(double) const;

  bool requestServoMode(bool);
  void callResetRobot();
  bool callEnableRobot();
  bool callDisableRobot();

  bool initializeServoPoseTarget();
  bool initializeServoJointTarget();
  bool initializeSelectedServoTarget();
  void resetServoTargets();
  void setServoControlType(ServoControlType);
  void publishServoTarget();
  void publishServoPoseTarget();
  void publishServoJointTarget();

  double consumeTargetElapsedSeconds(const rclcpp::Time &);
  bool joyInputIsFresh(const rclcpp::Time &) const;
  void setPoseYaw(geometry_msgs::msg::Pose &, double) const;
  const char * servoControlTypeName(ServoControlType) const;
};
}  // namespace mg400_joy

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mg400_joy::MG400JoyInterfaceNode)
#endif
