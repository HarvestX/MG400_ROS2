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

#ifndef MG400_RVIZ_PLUGIN__PANEL_MG400_CONTROLLER_HPP_
#define MG400_RVIZ_PLUGIN__PANEL_MG400_CONTROLLER_HPP_

#include <array>
#include <chrono>
#include <functional>

#ifndef Q_MOC_RUN
#include <mg400_msgs/action/joint_mov_j.hpp>
#include <mg400_msgs/action/mov_j.hpp>
#include <mg400_msgs/msg/robot_mode.hpp>
#include <mg400_msgs/srv/enable_robot.hpp>
#include <mg400_msgs/srv/disable_robot.hpp>
#include <mg400_msgs/srv/clear_error.hpp>
#include <mg400_msgs/srv/set_collision_level.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <QtWidgets>
#endif

namespace mg400_rviz_plugin
{
class Mg400ControllerPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit Mg400ControllerPanel(QWidget * parent = nullptr);
  void onInitialize() override;

protected:
  void initializeRos(const rclcpp::Node::SharedPtr & node);

private:
  using ActionT = mg400_msgs::action::JointMovJ;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using MovJAction = mg400_msgs::action::MovJ;
  using MovJGoalHandle = rclcpp_action::ClientGoalHandle<MovJAction>;
  using RobotMode = mg400_msgs::msg::RobotMode;
  using JointState = sensor_msgs::msg::JointState;

  void tick();
  void sendGoal();
  void sendMovJ();
  void sendCollisionLevel();
  void copyCurrentAngles();
  void copyCurrentPose();
  void updateControls();
  bool readGoal(ActionT::Goal & goal) const;
  bool readMovJGoal(MovJAction::Goal & goal) const;
  bool readEnableRequest(mg400_msgs::srv::EnableRobot::Request & request) const;
  void onJointState(const JointState::ConstSharedPtr msg);
  void showCurrentAngles(const std::array<double, 4> & angles);
  void showCurrentPose(const geometry_msgs::msg::Pose & pose);
  void onGoalResponse(const QString & action, bool accepted);
  void onResult(
    const QString & action, rclcpp_action::ResultCode code, bool succeeded,
    const mg400_msgs::msg::ErrorID * errors);

  template<typename ServiceT>
  void sendRobotRequest(
    const typename rclcpp::Client<ServiceT>::SharedPtr & client, const QString & name,
    const typename ServiceT::Request & request = typename ServiceT::Request());

  QLabel * label_mode_;
  QLabel * label_status_;
  QLabel * label_service_status_;
  QPushButton * button_enable_;
  QPushButton * button_disable_;
  QPushButton * button_clear_error_;
  std::array<QLineEdit *, 4> payload_inputs_;
  std::array<QLabel *, 4> current_labels_;
  std::array<QLineEdit *, 4> goal_inputs_;
  QPushButton * button_send_;
  QPushButton * button_copy_;
  std::array<QLabel *, 4> pose_labels_;
  std::array<QLineEdit *, 4> pose_inputs_;
  QPushButton * button_send_movj_;
  QPushButton * button_copy_pose_;
  QComboBox * collision_level_;
  QPushButton * button_set_collision_;

  RobotMode::_robot_mode_type current_robot_mode_ = RobotMode::INIT;
  std::array<double, 4> current_angles_{};
  std::array<double, 4> current_pose_values_{};
  bool have_joint_state_ = false;
  bool have_pose_ = false;
  bool goal_pending_ = false;
  bool service_pending_ = false;
  std::chrono::steady_clock::time_point service_deadline_;
  std::function<void()> remove_service_request_;

  rclcpp::Node::SharedPtr nh_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<RobotMode>::SharedPtr rm_sub_;
  rclcpp::Subscription<JointState>::SharedPtr js_sub_;
  rclcpp_action::Client<ActionT>::SharedPtr joint_movj_client_;
  rclcpp_action::Client<MovJAction>::SharedPtr movj_client_;
  rclcpp::Client<mg400_msgs::srv::EnableRobot>::SharedPtr enable_client_;
  rclcpp::Client<mg400_msgs::srv::DisableRobot>::SharedPtr disable_client_;
  rclcpp::Client<mg400_msgs::srv::ClearError>::SharedPtr clear_error_client_;
  rclcpp::Client<mg400_msgs::srv::SetCollisionLevel>::SharedPtr collision_client_;
};
}  // namespace mg400_rviz_plugin

#endif  // MG400_RVIZ_PLUGIN__PANEL_MG400_CONTROLLER_HPP_
