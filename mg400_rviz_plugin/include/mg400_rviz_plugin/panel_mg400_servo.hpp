// Copyright 2026
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

#ifndef __MG400_RVIZ_PLUGIN_PANEL_MG400_SERVO_HPP__
#define __MG400_RVIZ_PLUGIN_PANEL_MG400_SERVO_HPP__

#include <array>
#include <chrono>
#include <mutex>
#include <vector>

#ifndef Q_MOC_RUN
#include <QtWidgets>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#endif

#include <mg400_msgs/msg/realtime_feedback.hpp>
#include <mg400_msgs/msg/robot_mode.hpp>
#include <mg400_msgs/msg/servo_j.hpp>
#include <mg400_msgs/msg/servo_p.hpp>
#include <mg400_msgs/srv/servo_mode.hpp>

#include "mg400_rviz_plugin/panel_base.hpp"

namespace mg400_rviz_plugin
{
class Mg400ServoPanel : public Mg400PanelBase
{
  Q_OBJECT

private:
  enum class ServoControlType
  {
    SERVO_P,
    SERVO_J
  };

  using RealtimeFeedback = mg400_msgs::msg::RealtimeFeedback;
  using RobotMode = mg400_msgs::msg::RobotMode;
  using ServoModeService = mg400_msgs::srv::ServoMode;
  using ServoJ = mg400_msgs::msg::ServoJ;
  using ServoP = mg400_msgs::msg::ServoP;

protected:
  QLabel * label_robot_mode_;
  QLabel * label_servo_status_;
  QLabel * label_servo_target_;
  QLabel * label_servo_joint_target_;
  QRadioButton * radio_servo_p_;
  QRadioButton * radio_servo_j_;
  QPushButton * button_servo_mode_;
  QDoubleSpinBox * spin_linear_speed_;
  QDoubleSpinBox * spin_angular_speed_;
  QDoubleSpinBox * spin_joint_speed_;
  QFrame * joy_control_overlay_;
  std::vector<QPushButton *> target_buttons_;
  std::vector<QPushButton *> servo_j_buttons_;

  rclcpp::Subscription<RobotMode>::SharedPtr robot_mode_sub_;
  rclcpp::Subscription<RealtimeFeedback>::SharedPtr realtime_feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr servo_mode_enabled_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr joy_control_active_sub_;
  rclcpp::Publisher<ServoJ>::SharedPtr servo_j_pub_;
  rclcpp::Publisher<ServoP>::SharedPtr servo_p_pub_;
  rclcpp::Client<ServoModeService>::SharedPtr servo_mode_clnt_;

  std::mutex robot_mode_mutex_;
  RobotMode::_robot_mode_type current_robot_mode_;

  std::mutex actual_state_mutex_;
  geometry_msgs::msg::Pose current_pose_;
  std::array<double, 4> current_joint_state_;
  bool has_actual_state_;

  std::mutex servo_state_mutex_;
  ServoControlType servo_control_type_;
  bool servo_mode_enabled_;
  bool servo_target_initialized_;
  geometry_msgs::msg::Pose servo_target_pose_;
  std::array<int, 4> servo_target_directions_;
  bool servo_joint_target_initialized_;
  std::array<double, 4> servo_joint_target_;
  std::array<int, 4> servo_joint_directions_;
  std::chrono::steady_clock::time_point last_servo_target_update_time_;

  std::mutex joy_control_mutex_;
  bool joy_control_active_;
  std::chrono::steady_clock::time_point last_joy_control_active_time_;

public:
  explicit Mg400ServoPanel(QWidget * parent = nullptr);

public Q_SLOTS:
  void tick();
  void callbackServoMode();
  void onServoControlTimer();

protected:
  void resizeEvent(QResizeEvent * event) override;
  void onRosNodeInitialized() override;
  void onNamespaceChanged() override;
  void setupRosInterfaces();
  QWidget * createStatusSection();
  QWidget * createTargetSection();
  QWidget * createJointTargetSection();

  void connectTargetButton(QPushButton * button, size_t axis_index, int direction);
  void connectJointTargetButton(QPushButton * button, size_t axis_index, int direction);
  void setServoControlType(ServoControlType servo_control_type);
  void requestServoMode(bool enable);
  bool initializeServoTargetFromCurrentPose();
  bool initializeServoJointTargetFromCurrentJointState();
  bool initializeSelectedServoTarget();
  void setServoTargetDirection(size_t axis_index, int direction);
  void setServoJointDirection(size_t axis_index, int direction);
  void updateServoTargetPose();
  void updateServoJointTarget();
  void publishServoTarget();
  void publishServoPoseTarget();
  void publishServoJointTarget();
  bool joyControlIsActive();
  void updateJoyControlOverlay(bool active);
  void updateServoStatus(const QString & text);
  void updateServoTargetLabel();
  void updateServoJointTargetLabel();
};
}  // namespace mg400_rviz_plugin

#endif
