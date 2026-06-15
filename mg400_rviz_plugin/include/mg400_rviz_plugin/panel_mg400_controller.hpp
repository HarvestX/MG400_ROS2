// Copyright 2023 HarvestX Inc.
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

#ifndef __MG400_RVIZ_PLUGIN_PANEL_MG400_CONTROLLER_HPP__
#define __MG400_RVIZ_PLUGIN_PANEL_MG400_CONTROLLER_HPP__

#include <array>
#include <mutex>
#include <string>
#include <vector>

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <QtWidgets>
#endif
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mg400_msgs/msg/move_jog.hpp>
#include <mg400_msgs/msg/realtime_feedback.hpp>
#include <mg400_msgs/msg/robot_mode.hpp>
#include <mg400_msgs/srv/acc_j.hpp>
#include <mg400_msgs/srv/acc_l.hpp>
#include <mg400_msgs/srv/clear_error.hpp>
#include <mg400_msgs/srv/cp.hpp>
#include <mg400_msgs/srv/enable_robot.hpp>
#include <mg400_msgs/srv/disable_robot.hpp>
#include <mg400_msgs/srv/move_jog.hpp>
#include <mg400_msgs/srv/speed_factor.hpp>
#include <mg400_msgs/srv/speed_j.hpp>
#include <mg400_msgs/srv/speed_l.hpp>
#include <mg400_msgs/action/joint_mov_j.hpp>
#include <mg400_msgs/action/mov_j.hpp>
#include <mg400_msgs/action/mov_l.hpp>

#include "mg400_rviz_plugin/panel_base.hpp"

namespace mg400_rviz_plugin
{
class MG400InputGroup : public QHBoxLayout
{
private:
  QLabel * prefix_, * suffix_;
  QLineEdit * l_edit_;

public:
  QLabel * joint_state_;
  explicit MG400InputGroup(const std::string &, const std::string &);
  void addJogButtons(QPushButton * positive_button, QPushButton * negative_button);
  void disableLine();
  void enableLine();
  double getValue();
};

class Mg400ControllerPanel : public Mg400PanelBase
{
  Q_OBJECT

public:
  using MovJAction = mg400_msgs::action::MovJ;
  using MovLAction = mg400_msgs::action::MovL;
  using JointMovJAction = mg400_msgs::action::JointMovJ;
  using MovJGoalHandle = rclcpp_action::ClientGoalHandle<MovJAction>;
  using MovLGoalHandle = rclcpp_action::ClientGoalHandle<MovLAction>;
  using JointMovJGoalHandle = rclcpp_action::ClientGoalHandle<JointMovJAction>;

private:
  using MoveJog = mg400_msgs::msg::MoveJog;
  using MoveJogService = mg400_msgs::srv::MoveJog;
  using RealtimeFeedback = mg400_msgs::msg::RealtimeFeedback;
  using RobotMode = mg400_msgs::msg::RobotMode;

protected:
  QLabel * label_mode_;
  QLabel * label_action_status_;
  QLabel * label_param_status_;
  QLabel * label_jog_status_;

  MG400InputGroup * input_x_, * input_y_, * input_z_, * input_r_;
  MG400InputGroup * input_j1_, * input_j2_, * input_j3_, * input_j4_;

  QCheckBox * checkbox_speed_factor_, * checkbox_speed_j_, * checkbox_speed_l_;
  QCheckBox * checkbox_acc_j_, * checkbox_acc_l_, * checkbox_cp_;
  QSpinBox * spin_speed_factor_, * spin_speed_j_, * spin_speed_l_;
  QSpinBox * spin_acc_j_, * spin_acc_l_, * spin_cp_;

  QPushButton * button_enable_, * button_disable_, * button_clear_error_;
  QPushButton * button_send_params_;
  QPushButton * button_send_movj_, * button_send_movl_, * button_send_joint_movj_;
  std::vector<QPushButton *> jog_buttons_;

  rclcpp::Subscription<RobotMode>::SharedPtr rm_sub_;
  RobotMode::_robot_mode_type current_robot_mode_;

  rclcpp::Subscription<RealtimeFeedback>::SharedPtr realtime_feedback_sub_;
  std::mutex actual_state_mutex_;
  geometry_msgs::msg::Pose current_pose_;
  std::array<double, 4> current_joint_state_;
  bool has_actual_state_;

  rclcpp::Client<mg400_msgs::srv::EnableRobot>::SharedPtr
    mg400_enable_robot_clnt_;
  rclcpp::Client<mg400_msgs::srv::DisableRobot>::SharedPtr
    mg400_disable_robot_clnt_;
  rclcpp::Client<mg400_msgs::srv::ClearError>::SharedPtr
    mg400_clear_error_clnt_;
  rclcpp::Client<mg400_msgs::srv::SpeedFactor>::SharedPtr
    mg400_speed_factor_clnt_;
  rclcpp::Client<mg400_msgs::srv::SpeedJ>::SharedPtr
    mg400_speed_j_clnt_;
  rclcpp::Client<mg400_msgs::srv::SpeedL>::SharedPtr
    mg400_speed_l_clnt_;
  rclcpp::Client<mg400_msgs::srv::AccJ>::SharedPtr
    mg400_acc_j_clnt_;
  rclcpp::Client<mg400_msgs::srv::AccL>::SharedPtr
    mg400_acc_l_clnt_;
  rclcpp::Client<mg400_msgs::srv::CP>::SharedPtr
    mg400_cp_clnt_;
  rclcpp::Client<MoveJogService>::SharedPtr
    mg400_move_jog_clnt_;
  rclcpp_action::Client<MovJAction>::SharedPtr mg400_movj_clnt_;
  rclcpp_action::Client<MovLAction>::SharedPtr mg400_movl_clnt_;
  rclcpp_action::Client<JointMovJAction>::SharedPtr mg400_joint_movj_clnt_;

public:
  explicit Mg400ControllerPanel(QWidget * parent = nullptr);

public Q_SLOTS:
  void tick();
  void callbackEnableRobot();
  void callbackDisableRobot();
  void callbackClearError();
  void callbackSendParams();
  void callbackSendMovJ();
  void callbackSendMovL();
  void callbackSendJointMovJ();

protected:
  void onRosNodeInitialized() override;
  void onNamespaceChanged() override;
  void setupRosInterfaces();
  QWidget * createRobotSection();
  QWidget * createMotionParamsSection();
  QWidget * createPoseSection();
  QWidget * createJointSection();

  geometry_msgs::msg::PoseStamped createGoalPose();
  void addJogButtons(
    MG400InputGroup * input, QWidget * parent,
    const std::string & negative_mode, const std::string & positive_mode);
  void connectJogButton(QPushButton * button, const std::string & jog_mode);
  void sendMoveJog(const std::string & jog_mode);
  void applyJointMotionParams(MovJAction::Goal & goal);
  void applyLinearMotionParams(MovLAction::Goal & goal);
  void applyJointMotionParams(JointMovJAction::Goal & goal);
  void updateActionStatus(const QString & text);
  void updateParamStatus(const QString & text);
  void updateJogStatus(const QString & text);
  void handleGoalResponse(const std::string & action_name, const bool accepted);
  void handlePoseFeedback(
    const std::string & action_name, const geometry_msgs::msg::PoseStamped & pose);
  void handleJointFeedback(
    const std::string & action_name, const JointMovJAction::Feedback::ConstSharedPtr feedback);
  void handleActionResult(
    const std::string & action_name, const rclcpp_action::ResultCode code, const bool result);

  void onMovJGoalResponse(const MovJGoalHandle::SharedPtr & goal_handle);
  void onMovJFeedback(MovJGoalHandle::SharedPtr, const MovJAction::Feedback::ConstSharedPtr);
  void onMovJResult(const MovJGoalHandle::WrappedResult &);
  void onMovLGoalResponse(const MovLGoalHandle::SharedPtr & goal_handle);
  void onMovLFeedback(MovLGoalHandle::SharedPtr, const MovLAction::Feedback::ConstSharedPtr);
  void onMovLResult(const MovLGoalHandle::WrappedResult &);
  void onJointMovJGoalResponse(const JointMovJGoalHandle::SharedPtr & goal_handle);
  void onJointMovJFeedback(
    JointMovJGoalHandle::SharedPtr, const JointMovJAction::Feedback::ConstSharedPtr);
  void onJointMovJResult(const JointMovJGoalHandle::WrappedResult &);
};
}  // namespace mg400_rviz_plugin
#endif
