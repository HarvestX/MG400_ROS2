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

#include "mg400_rviz_plugin/panel_mg400_controller.hpp"

namespace mg400_rviz_plugin
{

Mg400ControllerPanel::Mg400ControllerPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  QVBoxLayout * layout = new QVBoxLayout;

  QHBoxLayout * layout_enable = new QHBoxLayout;
  radio_enable_ = new QRadioButton("Enable");
  layout_enable->addWidget(radio_enable_);
  radio_disable_ = new QRadioButton("Disable");
  layout_enable->addWidget(radio_disable_);
  radio_disable_->setChecked(true);
  QButtonGroup * group_enable = new QButtonGroup();
  group_enable->addButton(radio_enable_);
  group_enable->addButton(radio_disable_);
  layout->addLayout(layout_enable);

  QHBoxLayout * layout_movj = new QHBoxLayout;
  QVBoxLayout * layout_movj_goal = new QVBoxLayout;

  QHBoxLayout * layout_movj_goal_x = new QHBoxLayout;
  label_x_ = new QLabel("x:");
  layout_movj_goal_x->addWidget(label_x_);
  edit_movj_x_ = new QLineEdit("");
  layout_movj_goal_x->addWidget(edit_movj_x_);
  layout_movj_goal->addLayout(layout_movj_goal_x);

  QHBoxLayout * layout_movj_goal_y = new QHBoxLayout;
  label_y_ = new QLabel("y:");
  layout_movj_goal_y->addWidget(label_y_);
  edit_movj_y_ = new QLineEdit("");
  layout_movj_goal_y->addWidget(edit_movj_y_);
  layout_movj_goal->addLayout(layout_movj_goal_y);

  QHBoxLayout * layout_movj_goal_z = new QHBoxLayout;
  label_z_ = new QLabel("z:");
  layout_movj_goal_z->addWidget(label_z_);
  edit_movj_z_ = new QLineEdit("");
  layout_movj_goal_z->addWidget(edit_movj_z_);
  layout_movj_goal->addLayout(layout_movj_goal_z);

  QHBoxLayout * layout_movj_goal_r = new QHBoxLayout;
  label_r_ = new QLabel("r:");
  layout_movj_goal_r->addWidget(label_r_);
  edit_movj_r_ = new QLineEdit("");
  layout_movj_goal_r->addWidget(edit_movj_r_);
  layout_movj_goal->addLayout(layout_movj_goal_r);

  layout_movj->addLayout(layout_movj_goal);
  button_send_movj_ = new QPushButton("Send");
  layout_movj->addWidget(button_send_movj_);

  layout->addLayout(layout_movj);

  setLayout(layout);

  QTimer * output_timer = new QTimer(this);
  connect(output_timer, SIGNAL(timeout()), this, SLOT(tick()));
  output_timer->start(100);
  connect(button_send_movj_, SIGNAL(clicked()), this, SLOT(callbackSendMovJ()));
}

void Mg400ControllerPanel::onInitialize()
{
  nh_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  this->callback_group_ = nh_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  this->callback_group_executor_.add_callback_group(
    this->callback_group_, nh_->get_node_base_interface());

  rm_sub_ = nh_->create_subscription<RobotMode>(
    "/mg400/robot_mode", rclcpp::SensorDataQoS().keep_last(1),
    [&](const mg400_msgs::msg::RobotMode::ConstSharedPtr msg) {
      this->current_robot_mode_ = msg;
    });
  this->current_robot_mode_ = std::make_shared<const RobotMode>();

  mg400_enable_robot_clnt_ =
    nh_->create_client<mg400_msgs::srv::EnableRobot>(
    "/mg400/enable_robot", rmw_qos_profile_default, this->callback_group_);
  mg400_disable_robot_clnt_ =
    nh_->create_client<mg400_msgs::srv::DisableRobot>(
    "/mg400/disable_robot", rmw_qos_profile_default, this->callback_group_);

  this->mg400_movj_clnt_ = rclcpp_action::create_client<ActionT>(
    nh_, "/mg400/mov_j", this->callback_group_);
}

void Mg400ControllerPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void Mg400ControllerPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void Mg400ControllerPanel::tick()
{
  if (radio_enable_->isChecked()) {
    label_x_->setStyleSheet("color: black;");
    label_y_->setStyleSheet("color: black;");
    label_z_->setStyleSheet("color: black;");
    label_r_->setStyleSheet("color: black;");
    edit_movj_x_->setEnabled(true);
    edit_movj_y_->setEnabled(true);
    edit_movj_z_->setEnabled(true);
    edit_movj_r_->setEnabled(true);
    button_send_movj_->setEnabled(true);
    goal_x = edit_movj_x_->text().toFloat();
    goal_y = edit_movj_y_->text().toFloat();
    goal_z = edit_movj_z_->text().toFloat();
    goal_r = edit_movj_r_->text().toFloat();

    if (!is_enabled_before) {
      Mg400ControllerPanel::callEnableRobot();
      is_enabled_before = true;
    }
  }

  if (radio_disable_->isChecked()) {
    label_x_->setStyleSheet("color: gray;");
    label_y_->setStyleSheet("color: gray;");
    label_z_->setStyleSheet("color: gray;");
    label_r_->setStyleSheet("color: gray;");
    edit_movj_x_->setEnabled(false);
    edit_movj_y_->setEnabled(false);
    edit_movj_z_->setEnabled(false);
    edit_movj_r_->setEnabled(false);
    button_send_movj_->setEnabled(false);
    Mg400ControllerPanel::callDisableRobot();
    is_enabled_before = false;
  }
}

void Mg400ControllerPanel::callbackSendMovJ()
{
  auto goal_msg = ActionT::Goal();
  goal_msg.pose.header.frame_id = "mg400_origin_link";
  goal_msg.pose.header.stamp.sec = 0;
  goal_msg.pose.header.stamp.nanosec = 0;
  goal_msg.pose.pose.position.x = goal_x;
  goal_msg.pose.pose.position.y = goal_y;
  goal_msg.pose.pose.position.z = goal_z;

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, goal_r * M_PI / 180);
  goal_msg.pose.pose.orientation.x = quat.getX();
  goal_msg.pose.pose.orientation.y = quat.getY();
  goal_msg.pose.pose.orientation.z = quat.getZ();
  goal_msg.pose.pose.orientation.w = quat.getW();


  auto send_goal_option = rclcpp_action::Client<ActionT>::SendGoalOptions();

  using namespace std::placeholders; // NOLINT
  send_goal_option.goal_response_callback =
    std::bind(&Mg400ControllerPanel::onGoalResponse, this, _1);
  send_goal_option.feedback_callback =
    std::bind(&Mg400ControllerPanel::onFeedback, this, _1, _2);
  send_goal_option.result_callback =
    std::bind(&Mg400ControllerPanel::onResult, this, _1);

  mg400_movj_clnt_->async_send_goal(goal_msg, send_goal_option);
}

void Mg400ControllerPanel::callEnableRobot()
{
  if (this->current_robot_mode_->robot_mode == RobotMode::ENABLE) {
    RCLCPP_WARN(
      nh_->get_logger(),
      "robot already enabled");
    return;
  }

  if (!mg400_enable_robot_clnt_->wait_for_service(1s)) {
    RCLCPP_ERROR(
      nh_->get_logger(), "\"%s\" is not ready",
      mg400_enable_robot_clnt_->get_service_name());
    return;
  }

  auto req = std::make_shared<mg400_msgs::srv::EnableRobot::Request>();

  auto future_result = mg400_enable_robot_clnt_->async_send_request(req);

  if (this->callback_group_executor_.spin_until_future_complete(future_result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      nh_->get_logger(),
      "\"%s\" service client: async_send_request failed",
      mg400_enable_robot_clnt_->get_service_name());
    return;
  }

  const auto wrapped_result = future_result.get();
  if (!wrapped_result->result) {
    RCLCPP_ERROR(
      nh_->get_logger(),
      "\"%s\" service client: failed",
      mg400_enable_robot_clnt_->get_service_name());
  }
}

void Mg400ControllerPanel::callDisableRobot()
{
  if (this->current_robot_mode_->robot_mode == RobotMode::DISABLED) {
    RCLCPP_WARN(
      nh_->get_logger(),
      "robot already disabled");
    return;
  }

  if (!mg400_disable_robot_clnt_->wait_for_service(1s)) {
    RCLCPP_ERROR(
      nh_->get_logger(), "\"%s\" is not ready",
      mg400_disable_robot_clnt_->get_service_name());
    return;
  }

  auto req = std::make_shared<mg400_msgs::srv::DisableRobot::Request>();

  auto future_result = mg400_disable_robot_clnt_->async_send_request(req);

  if (this->callback_group_executor_.spin_until_future_complete(future_result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      nh_->get_logger(),
      "\"%s\" service client: async_send_request failed",
      mg400_disable_robot_clnt_->get_service_name());
    return;
  }

  const auto wrapped_result = future_result.get();
  if (!wrapped_result->result) {
    RCLCPP_ERROR(
      nh_->get_logger(),
      "\"%s\" service client: failed",
      mg400_disable_robot_clnt_->get_service_name());
  }
}

void Mg400ControllerPanel::onGoalResponse(const GoalHandle::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(nh_->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(nh_->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void Mg400ControllerPanel::onFeedback(
  GoalHandle::SharedPtr, const ActionT::Feedback::ConstSharedPtr feedback)
{
  RCLCPP_INFO(
    nh_->get_logger(), "current pose: (%lf, %lf, %lf)",
    feedback->current_pose.pose.position.x,
    feedback->current_pose.pose.position.y,
    feedback->current_pose.pose.position.z);
}

void Mg400ControllerPanel::onResult(const GoalHandle::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(nh_->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(nh_->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(nh_->get_logger(), "Unknown result code");
      return;
  }

  if (result.result->result) {
    RCLCPP_INFO(nh_->get_logger(), "Succeeded");
  } else {
    RCLCPP_ERROR(nh_->get_logger(), "Failed");
  }
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mg400_rviz_plugin::Mg400ControllerPanel, rviz_common::Panel)
