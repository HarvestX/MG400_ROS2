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

#pragma once

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <QtWidgets>
#endif
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mg400_msgs/msg/robot_mode.hpp>
#include <mg400_msgs/srv/enable_robot.hpp>
#include <mg400_msgs/srv/disable_robot.hpp>
#include <mg400_msgs/action/mov_j.hpp>

namespace mg400_rviz_plugin
{
using namespace std::chrono_literals; // NOLINT

class Mg400ControllerPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  Mg400ControllerPanel(QWidget * parent = nullptr);

  using ActionT = mg400_msgs::action::MovJ;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

  virtual void onInitialize();
  virtual void load(const rviz_common::Config & config);
  virtual void save(rviz_common::Config config) const;

public Q_SLOTS:
  void tick();
  void callbackSendMovJ();

private:
  using RobotMode = mg400_msgs::msg::RobotMode;
  float goal_x;
  float goal_y;
  float goal_z;
  float goal_r;

protected:
  QRadioButton * radio_enable_;
  QRadioButton * radio_disable_;

  QLabel * label_x_;
  QLabel * label_y_;
  QLabel * label_z_;
  QLabel * label_r_;
  QLineEdit * edit_movj_x_;
  QLineEdit * edit_movj_y_;
  QLineEdit * edit_movj_z_;
  QLineEdit * edit_movj_r_;
  QPushButton * button_send_movj_;

  bool is_enabled_before = false;

  rclcpp::Node::SharedPtr nh_;

  rclcpp::Subscription<RobotMode>::SharedPtr rm_sub_;
  RobotMode::ConstSharedPtr current_robot_mode_;

  rclcpp::Client<mg400_msgs::srv::EnableRobot>::SharedPtr
    mg400_enable_robot_clnt_;
  rclcpp::Client<mg400_msgs::srv::DisableRobot>::SharedPtr
    mg400_disable_robot_clnt_;
  rclcpp_action::Client<ActionT>::SharedPtr mg400_movj_clnt_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  void callEnableRobot();
  void callDisableRobot();
  void onGoalResponse(const GoalHandle::SharedPtr & goal_handle);
  void onFeedback(GoalHandle::SharedPtr, const ActionT::Feedback::ConstSharedPtr);
  void onResult(const GoalHandle::WrappedResult &);
};

}
