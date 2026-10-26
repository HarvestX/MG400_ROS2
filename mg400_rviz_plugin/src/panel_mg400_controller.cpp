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

MG400InputGroup::MG400InputGroup(const std::string & prefix, const std::string & suffix)
{
  this->prefix_ = new QLabel(prefix.data());
  this->addWidget(this->prefix_);
  this->suffix_ = new QLabel(suffix.data());
  this->addWidget(this->suffix_);
  this->joint_state_ = new QLabel("");
  this->addWidget(this->joint_state_);
  this->l_edit_ = new QLineEdit("");
  this->l_edit_->setMaximumWidth(60);
  this->l_edit_->setPlaceholderText("0.0");
  this->addWidget(this->l_edit_);
}

void MG400InputGroup::disableLine()
{
  // this->prefix_->setStyleSheet("color: gray");
  // this->suffix_->setStyleSheet("color: gray");
  this->l_edit_->setEnabled(false);
}

void MG400InputGroup::enableLine()
{
  // this->prefix_->setStyleSheet("color: black;");
  // this->suffix_->setStyleSheet("color: black;");
  this->l_edit_->setEnabled(true);
}

double MG400InputGroup::getValue()
{
  return this->l_edit_->text().toDouble();
}

Mg400ControllerPanel::Mg400ControllerPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  QVBoxLayout * layout = new QVBoxLayout;

  QHBoxLayout * layout_mode = new QHBoxLayout;
  layout_mode->addWidget(new QLabel("Robot Mode: "));
  label_mode_ = new QLabel();
  layout_mode->addWidget(label_mode_);

  QVBoxLayout * layout_mode_buttons = new QVBoxLayout;
  this->button_enable_ = new QPushButton("Enable");
  layout_mode_buttons->addWidget(this->button_enable_);
  this->button_disable_ = new QPushButton("Disable");
  layout_mode_buttons->addWidget(this->button_disable_);
  layout_mode->addLayout(layout_mode_buttons);
  layout->addLayout(layout_mode);

  QHBoxLayout * layout_pose_index = new QHBoxLayout;
  layout_pose_index->addWidget(new QLabel("Robot Pose / MovJ Goal Pose"));
  layout->addLayout(layout_pose_index);

  QHBoxLayout * layout_movj = new QHBoxLayout;
  QVBoxLayout * layout_movj_goal = new QVBoxLayout;

  this->input_x_ = new MG400InputGroup("x", "[mm]");
  layout_movj_goal->addLayout(this->input_x_);

  this->input_y_ = new MG400InputGroup("y", "[mm]");
  layout_movj_goal->addLayout(this->input_y_);

  this->input_z_ = new MG400InputGroup("z", "[mm]");
  layout_movj_goal->addLayout(this->input_z_);

  this->input_r_ = new MG400InputGroup("r", "[degree]");
  layout_movj_goal->addLayout(this->input_r_);

  layout_movj->addLayout(layout_movj_goal);
  this->button_send_movj_ = new QPushButton("Send");
  layout_movj->addWidget(this->button_send_movj_);

  layout->addLayout(layout_movj);

  this->setLayout(layout);

  QTimer * output_timer = new QTimer(this);
  this->connect(output_timer, SIGNAL(timeout()), this, SLOT(tick()));
  output_timer->start(100);  // Timeout set to 100ms = 0.1s
  this->connect(this->button_enable_, SIGNAL(clicked()), this, SLOT(callbackEnableRobot()));
  this->connect(this->button_disable_, SIGNAL(clicked()), this, SLOT(callbackDisableRobot()));
  this->connect(this->button_send_movj_, SIGNAL(clicked()), this, SLOT(callbackSendMovJ()));
}

void Mg400ControllerPanel::onInitialize()
{
  nh_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  this->callback_group_ = nh_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  this->callback_group_executor_.add_callback_group(
    this->callback_group_, nh_->get_node_base_interface());

  // TODO(anyone): make namespace editable
  this->current_robot_mode_ = mg400_msgs::msg::RobotMode::INIT;
  rm_sub_ = nh_->create_subscription<RobotMode>(
    "/mg400/robot_mode", rclcpp::SensorDataQoS().keep_last(1),
    [&](const mg400_msgs::msg::RobotMode::ConstSharedPtr msg) {
      this->current_robot_mode_ = msg->robot_mode;
    });
  js_sub_ = nh_->create_subscription<JointState>(
    "/mg400/joint_states", rclcpp::SensorDataQoS().keep_last(1),
    [&](const JointState::ConstSharedPtr msg) {
      mg400_interface::JointHandler::getEndPose(msg, this->current_pose_);
    });

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
  if (this->current_robot_mode_ == RobotMode::ENABLE) {
    this->input_x_->enableLine();
    this->input_y_->enableLine();
    this->input_z_->enableLine();
    this->input_r_->enableLine();
    this->label_mode_->setStyleSheet("color: green;");
  } else if (this->current_robot_mode_ == RobotMode::DISABLED) {
    this->input_x_->disableLine();
    this->input_y_->disableLine();
    this->input_z_->disableLine();
    this->input_r_->disableLine();
    this->label_mode_->setStyleSheet("color: blue;");
  } else {
    this->input_x_->disableLine();
    this->input_y_->disableLine();
    this->input_z_->disableLine();
    this->input_r_->disableLine();
    this->label_mode_->setStyleSheet("");
  }

  auto robot_mode2str = [&](const RobotMode::_robot_mode_type mode) -> const char *
    {
      switch (mode) {
        case RobotMode::INIT:
          return "INIT";
        case RobotMode::BRAKE_OPEN:
          return "RAKE_OPEN";
        case RobotMode::DISABLED:
          return "DISABLED";
        case RobotMode::ENABLE:
          return "ENABLE";
        case RobotMode::BACKDRIVE:
          return "BACKDRIVE";
        case RobotMode::RUNNING:
          return "RUNNING";
        case RobotMode::RECORDING:
          return "RECORDING";
        case RobotMode::ERROR:
          return "ERROR";
        case RobotMode::PAUSE:
          return "PAUSE";
        case RobotMode::JOG:
          return "JOG";
        case RobotMode::INVALID:
          return "INVALID";
        default:
          return "INVALID";
      }
    };

  this->button_enable_->setEnabled(this->current_robot_mode_ == RobotMode::DISABLED);
  this->button_disable_->setEnabled(this->current_robot_mode_ == RobotMode::ENABLE);
  this->button_send_movj_->setEnabled(this->current_robot_mode_ == RobotMode::ENABLE);
  this->label_mode_->setText(robot_mode2str(this->current_robot_mode_));
  this->input_x_->joint_state_->setText(QString::number(this->current_pose_.position.x * 1e3));
  this->input_y_->joint_state_->setText(QString::number(this->current_pose_.position.y * 1e3));
  this->input_z_->joint_state_->setText(QString::number(this->current_pose_.position.z * 1e3));
  tf2::Quaternion q(
    this->current_pose_.orientation.x,
    this->current_pose_.orientation.y,
    this->current_pose_.orientation.z,
    this->current_pose_.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  this->input_r_->joint_state_->setText(QString::number(yaw * 180.0 / M_PI));
}

void Mg400ControllerPanel::callbackSendMovJ()
{
  auto goal_msg = ActionT::Goal();
  goal_msg.pose.header.frame_id = "mg400_origin_link";
  goal_msg.pose.header.stamp.sec = 0;
  goal_msg.pose.header.stamp.nanosec = 0;
  goal_msg.pose.pose.position.x = this->input_x_->getValue() * 1e-3;
  goal_msg.pose.pose.position.y = this->input_y_->getValue() * 1e-3;
  goal_msg.pose.pose.position.z = this->input_z_->getValue() * 1e-3;

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, this->input_r_->getValue() * M_PI / 180.0);
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

void Mg400ControllerPanel::callbackEnableRobot()
{
  using namespace std::chrono_literals;  // NOLINT
  if (!mg400_enable_robot_clnt_->wait_for_service(1s)) {
    RCLCPP_ERROR(
      nh_->get_logger(), "\"%s\" is not ready", mg400_enable_robot_clnt_->get_service_name());
    return;
  }

  auto req = std::make_shared<mg400_msgs::srv::EnableRobot::Request>();

  auto future_result = mg400_enable_robot_clnt_->async_send_request(req);

  if (this->callback_group_executor_.spin_until_future_complete(future_result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      nh_->get_logger(), "\"%s\" service client: async_send_request failed",
      mg400_enable_robot_clnt_->get_service_name());
    return;
  }

  const auto wrapped_result = future_result.get();
  if (!wrapped_result->result) {
    RCLCPP_ERROR(
      nh_->get_logger(), "\"%s\" service client: failed",
      mg400_enable_robot_clnt_->get_service_name());
  }
}

void Mg400ControllerPanel::callbackDisableRobot()
{
  using namespace std::chrono_literals;  // NOLINT
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
      nh_->get_logger(), "\"%s\" service client: async_send_request failed",
      mg400_disable_robot_clnt_->get_service_name());
    return;
  }

  const auto wrapped_result = future_result.get();
  if (!wrapped_result->result) {
    RCLCPP_ERROR(
      nh_->get_logger(), "\"%s\" service client: failed",
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
}  // namespace mg400_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mg400_rviz_plugin::Mg400ControllerPanel, rviz_common::Panel)
