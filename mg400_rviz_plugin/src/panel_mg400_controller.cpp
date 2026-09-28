// Copyright 2026 HarvestX Inc.
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

#include <algorithm>
#include <cmath>
#include <exception>
#include <vector>

#include <mg400_common/kinematics.hpp>
#include <mg400_common/mg400_ik_util.hpp>
#include <mg400_interface/joint_handler.hpp>
#include <rviz_common/display_context.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mg400_rviz_plugin
{
Mg400ControllerPanel::Mg400ControllerPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout;
  auto * robot_buttons = new QHBoxLayout;
  button_enable_ = new QPushButton("Enable");
  button_enable_->setObjectName("enable_robot");
  button_disable_ = new QPushButton("Disable");
  button_disable_->setObjectName("disable_robot");
  button_clear_error_ = new QPushButton("Clear Error");
  button_clear_error_->setObjectName("clear_error");
  robot_buttons->addWidget(button_enable_);
  robot_buttons->addWidget(button_disable_);
  robot_buttons->addWidget(button_clear_error_);
  layout->addLayout(robot_buttons);
  label_mode_ = new QLabel("Robot Mode: INIT");
  label_mode_->setObjectName("robot_mode");
  layout->addWidget(label_mode_);
  label_service_status_ = new QLabel;
  label_service_status_->setObjectName("service_status");
  label_service_status_->setWordWrap(true);
  layout->addWidget(label_service_status_);

  auto * tabs = new QTabWidget;
  tabs->setObjectName("motion_tabs");
  auto * movj_tab = new QWidget;
  auto * movj_layout = new QVBoxLayout(movj_tab);
  auto * pose = new QGridLayout;
  pose->addWidget(new QLabel("Pose"), 0, 0);
  pose->addWidget(new QLabel("Current"), 0, 1);
  pose->addWidget(new QLabel("MovJ Goal"), 0, 2);
  const std::array<const char *, 4> axes = {"x", "y", "z", "r"};
  for (size_t i = 0; i < pose_inputs_.size(); ++i) {
    const QString unit = i == 3 ? "degree" : "mm";
    pose->addWidget(new QLabel(QString("%1 [%2]").arg(axes[i]).arg(unit)), i + 1, 0);
    pose_labels_[i] = new QLabel("--");
    pose_labels_[i]->setObjectName(QString("current_%1").arg(axes[i]));
    pose->addWidget(pose_labels_[i], i + 1, 1);
    pose_inputs_[i] = new QLineEdit;
    pose_inputs_[i]->setObjectName(QString("goal_%1").arg(axes[i]));
    pose_inputs_[i]->setPlaceholderText(unit);
    auto * validator = new QDoubleValidator(pose_inputs_[i]);
    validator->setLocale(QLocale::c());
    pose_inputs_[i]->setValidator(validator);
    pose->addWidget(pose_inputs_[i], i + 1, 2);
    connect(pose_inputs_[i], &QLineEdit::textChanged, this, [this]() {updateControls();});
  }
  movj_layout->addLayout(pose);
  auto * pose_buttons = new QHBoxLayout;
  button_copy_pose_ = new QPushButton("Use Current");
  button_copy_pose_->setObjectName("use_current_pose");
  button_send_movj_ = new QPushButton("Send MovJ");
  button_send_movj_->setObjectName("send_mov_j");
  pose_buttons->addWidget(button_copy_pose_);
  pose_buttons->addWidget(button_send_movj_);
  movj_layout->addLayout(pose_buttons);
  tabs->addTab(movj_tab, "MovJ");

  auto * joint_tab = new QWidget;
  auto * joint_layout = new QVBoxLayout(joint_tab);

  auto * joints = new QGridLayout;
  joints->addWidget(new QLabel("Joint [degree]"), 0, 0);
  joints->addWidget(new QLabel("Current"), 0, 1);
  joints->addWidget(new QLabel("JointMovJ Goal"), 0, 2);
  namespace kinematics = mg400_common::kinematics;
  const std::array<double, 4> lower = {
    kinematics::J1_MIN, kinematics::J2_MIN, kinematics::J3_MIN, kinematics::J4_MIN};
  const std::array<double, 4> upper = {
    kinematics::J1_MAX, kinematics::J2_MAX, kinematics::J3_MAX, kinematics::J4_MAX};
  for (size_t i = 0; i < goal_inputs_.size(); ++i) {
    joints->addWidget(new QLabel(QString("J%1").arg(i + 1)), i + 1, 0);
    current_labels_[i] = new QLabel("--");
    current_labels_[i]->setObjectName(QString("current_j%1").arg(i + 1));
    joints->addWidget(current_labels_[i], i + 1, 1);
    goal_inputs_[i] = new QLineEdit;
    goal_inputs_[i]->setObjectName(QString("goal_j%1").arg(i + 1));
    goal_inputs_[i]->setPlaceholderText("degree");
    auto * validator = new QDoubleValidator(
      mg400_interface::rad2degree(lower[i]), mg400_interface::rad2degree(upper[i]),
      6, goal_inputs_[i]);
    validator->setLocale(QLocale::c());
    goal_inputs_[i]->setValidator(validator);
    joints->addWidget(goal_inputs_[i], i + 1, 2);
    connect(goal_inputs_[i], &QLineEdit::textChanged, this, [this]() {updateControls();});
  }
  joint_layout->addLayout(joints);

  auto * buttons = new QHBoxLayout;
  button_copy_ = new QPushButton("Use Current");
  button_copy_->setObjectName("use_current");
  buttons->addWidget(button_copy_);
  button_send_ = new QPushButton("Send JointMovJ");
  button_send_->setObjectName("send_joint_mov_j");
  buttons->addWidget(button_send_);
  joint_layout->addLayout(buttons);
  tabs->addTab(joint_tab, "JointMovJ");
  layout->addWidget(tabs);
  label_status_ = new QLabel("Enter a target or use the current values.");
  label_status_->setObjectName("status");
  label_status_->setWordWrap(true);
  layout->addWidget(label_status_);
  setLayout(layout);

  connect(button_copy_, &QPushButton::clicked, this, &Mg400ControllerPanel::copyCurrentAngles);
  connect(button_send_, &QPushButton::clicked, this, &Mg400ControllerPanel::sendGoal);
  connect(button_copy_pose_, &QPushButton::clicked, this, &Mg400ControllerPanel::copyCurrentPose);
  connect(button_send_movj_, &QPushButton::clicked, this, &Mg400ControllerPanel::sendMovJ);
  connect(
    button_enable_, &QPushButton::clicked, this, [this]() {
      sendRobotRequest<mg400_msgs::srv::EnableRobot>(enable_client_, "Enable");
    });
  connect(
    button_disable_, &QPushButton::clicked, this, [this]() {
      sendRobotRequest<mg400_msgs::srv::DisableRobot>(disable_client_, "Disable");
    });
  connect(
    button_clear_error_, &QPushButton::clicked, this, [this]() {
      sendRobotRequest<mg400_msgs::srv::ClearError>(clear_error_client_, "Clear Error");
    });
  auto * timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &Mg400ControllerPanel::tick);
  timer->start(100);
  updateControls();
}

void Mg400ControllerPanel::onInitialize()
{
  initializeRos(getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node());
}

void Mg400ControllerPanel::initializeRos(const rclcpp::Node::SharedPtr & node)
{
  nh_ = node;
  callback_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, nh_->get_node_base_interface());

  // Run every ROS callback in tick(), on the Qt thread, including subscriptions.
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  rm_sub_ = nh_->create_subscription<RobotMode>(
    "/mg400/robot_mode", rclcpp::SensorDataQoS().keep_last(1),
    [this](const RobotMode::ConstSharedPtr msg) {current_robot_mode_ = msg->robot_mode;}, options);
  js_sub_ = nh_->create_subscription<JointState>(
    "/mg400/joint_states", rclcpp::SensorDataQoS().keep_last(1),
    [this](const JointState::ConstSharedPtr msg) {onJointState(msg);}, options);
  joint_movj_client_ = rclcpp_action::create_client<ActionT>(
    nh_, "/mg400/joint_mov_j",
    callback_group_);
  movj_client_ = rclcpp_action::create_client<MovJAction>(nh_, "/mg400/mov_j", callback_group_);
  enable_client_ = nh_->create_client<mg400_msgs::srv::EnableRobot>(
    "/mg400/enable_robot", rmw_qos_profile_services_default, callback_group_);
  disable_client_ = nh_->create_client<mg400_msgs::srv::DisableRobot>(
    "/mg400/disable_robot", rmw_qos_profile_services_default, callback_group_);
  clear_error_client_ = nh_->create_client<mg400_msgs::srv::ClearError>(
    "/mg400/clear_error", rmw_qos_profile_services_default, callback_group_);
}

void Mg400ControllerPanel::tick()
{
  if (nh_) {
    callback_group_executor_.spin_some();
  }
  if (service_pending_ && std::chrono::steady_clock::now() >= service_deadline_) {
    remove_service_request_();
    remove_service_request_ = {};
    service_pending_ = false;
    label_service_status_->setText("Robot service timed out.");
  }
  updateControls();
}

bool Mg400ControllerPanel::readGoal(ActionT::Goal & goal) const
{
  for (size_t i = 0; i < goal_inputs_.size(); ++i) {
    bool ok = false;
    const double degrees = goal_inputs_[i]->text().toDouble(&ok);
    if (!ok || !goal_inputs_[i]->hasAcceptableInput() || !std::isfinite(degrees)) {
      return false;
    }
    goal.joint_angles[i] = mg400_interface::degree2rad(degrees);
  }
  // Use the same limits (including the J2/J3 coupling) as the action server.
  return mg400_common::MG400IKUtil().InMG400Range(
    std::vector<double>(goal.joint_angles.begin(), goal.joint_angles.end()));
}

void Mg400ControllerPanel::updateControls()
{
  QString mode;
  switch (current_robot_mode_) {
    case RobotMode::INIT: mode = "INIT"; break;
    case RobotMode::BRAKE_OPEN: mode = "BRAKE_OPEN"; break;
    case RobotMode::DISABLED: mode = "DISABLED"; break;
    case RobotMode::ENABLE: mode = "ENABLE"; break;
    case RobotMode::BACKDRIVE: mode = "BACKDRIVE"; break;
    case RobotMode::RUNNING: mode = "RUNNING"; break;
    case RobotMode::RECORDING: mode = "RECORDING"; break;
    case RobotMode::ERROR: mode = "ERROR"; break;
    case RobotMode::PAUSE: mode = "PAUSE"; break;
    case RobotMode::JOG: mode = "JOG"; break;
    default: mode = "INVALID"; break;
  }
  label_mode_->setText("Robot Mode: " + mode);
  ActionT::Goal goal;
  const bool valid_goal = readGoal(goal);
  const bool ready = joint_movj_client_ && joint_movj_client_->action_server_is_ready();
  const bool idle = !goal_pending_ && !service_pending_;
  button_send_->setEnabled(
    current_robot_mode_ == RobotMode::ENABLE && ready && valid_goal && idle);
  button_send_->setToolTip(
    !valid_goal ? "Enter four valid angles within the MG400 joint limits." :
    !ready ? "Waiting for /mg400/joint_mov_j." :
    current_robot_mode_ != RobotMode::ENABLE ? "Enable the robot using Mg400Controller." : "");
  MovJAction::Goal pose_goal;
  button_send_movj_->setEnabled(
    current_robot_mode_ == RobotMode::ENABLE && movj_client_ &&
    movj_client_->action_server_is_ready() && readMovJGoal(pose_goal) && idle);
  button_enable_->setEnabled(
    current_robot_mode_ == RobotMode::DISABLED && idle && enable_client_ &&
    enable_client_->service_is_ready());
  button_disable_->setEnabled(
    (current_robot_mode_ == RobotMode::ENABLE || current_robot_mode_ == RobotMode::RUNNING) &&
    !service_pending_ && disable_client_ && disable_client_->service_is_ready());
  button_clear_error_->setEnabled(
    current_robot_mode_ != RobotMode::INIT && current_robot_mode_ != RobotMode::RUNNING &&
    idle && clear_error_client_ && clear_error_client_->service_is_ready());
  button_copy_->setEnabled(have_joint_state_ && idle);
  button_copy_pose_->setEnabled(have_pose_ && idle);
  for (auto * input : goal_inputs_) {
    input->setEnabled(idle);
  }
  for (auto * input : pose_inputs_) {
    input->setEnabled(idle);
  }
}

void Mg400ControllerPanel::onJointState(const JointState::ConstSharedPtr msg)
{
  // JointHandler publishes eight URDF joints. Physical J3 is j4_2, not j3_1;
  // physical J4 is j5. Match names so reordered messages work as well.
  const std::array<const char *, 4> names = {
    mg400_interface::J1_NAME, mg400_interface::J2_1_NAME,
    mg400_interface::J4_2_NAME, mg400_interface::J5_NAME};
  std::array<double, 4> angles;
  for (size_t i = 0; i < names.size(); ++i) {
    const auto it = std::find_if(
      msg->name.begin(), msg->name.end(),
      [&](const std::string & name) {return QString::fromStdString(name).endsWith(names[i]);});
    const auto index = static_cast<size_t>(std::distance(msg->name.begin(), it));
    if (it == msg->name.end() || index >= msg->position.size()) {
      return;
    }
    angles[i] = msg->position[index];
  }
  showCurrentAngles(angles);
}

void Mg400ControllerPanel::showCurrentAngles(const std::array<double, 4> & angles)
{
  if (!std::all_of(angles.begin(), angles.end(), [](double v) {return std::isfinite(v);})) {
    return;
  }
  current_angles_ = angles;
  have_joint_state_ = true;
  for (size_t i = 0; i < angles.size(); ++i) {
    current_labels_[i]->setText(QString::number(mg400_interface::rad2degree(angles[i]), 'f', 3));
  }
  geometry_msgs::msg::Pose pose;
  mg400_interface::JointHandler::getEndPose(angles, pose);
  showCurrentPose(pose);
}

void Mg400ControllerPanel::copyCurrentAngles()
{
  if (!have_joint_state_ || goal_pending_ || service_pending_) {
    return;
  }
  for (size_t i = 0; i < current_angles_.size(); ++i) {
    goal_inputs_[i]->setText(
      QString::number(mg400_interface::rad2degree(current_angles_[i]), 'f', 6));
  }
}

void Mg400ControllerPanel::sendGoal()
{
  ActionT::Goal goal;
  if (goal_pending_ || service_pending_ || current_robot_mode_ != RobotMode::ENABLE ||
    !readGoal(goal) ||
    !joint_movj_client_ || !joint_movj_client_->action_server_is_ready())
  {
    return;
  }

  rclcpp_action::Client<ActionT>::SendGoalOptions options;
  options.goal_response_callback =
    [this](const GoalHandle::SharedPtr & handle) {onGoalResponse("JointMovJ", bool(handle));};
  options.feedback_callback =
    [this](GoalHandle::SharedPtr, const ActionT::Feedback::ConstSharedPtr feedback) {
      showCurrentAngles(feedback->current_angles);
    };
  options.result_callback =
    [this](const GoalHandle::WrappedResult & result) {
      onResult(
        "JointMovJ", result.code, result.result && result.result->result,
        result.result ? &result.result->error_id : nullptr);
    };

  goal_pending_ = true;
  label_status_->setText("Sending JointMovJ...");
  updateControls();
  try {
    joint_movj_client_->async_send_goal(goal, options);
  } catch (const std::exception & e) {
    goal_pending_ = false;
    label_status_->setText(QString("Failed to send JointMovJ: %1").arg(e.what()));
    updateControls();
  }
}

bool Mg400ControllerPanel::readMovJGoal(MovJAction::Goal & goal) const
{
  std::array<double, 4> values;
  for (size_t i = 0; i < values.size(); ++i) {
    bool ok = false;
    values[i] = pose_inputs_[i]->text().toDouble(&ok);
    if (!ok || !pose_inputs_[i]->hasAcceptableInput() || !std::isfinite(values[i])) {
      return false;
    }
  }
  goal.pose.header.frame_id = "mg400_origin_link";
  goal.pose.pose.position.x = mg400_interface::mm2m(values[0]);
  goal.pose.pose.position.y = mg400_interface::mm2m(values[1]);
  goal.pose.pose.position.z = mg400_interface::mm2m(values[2]);
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, mg400_interface::degree2rad(values[3]));
  goal.pose.pose.orientation = tf2::toMsg(orientation);
  return true;
}

void Mg400ControllerPanel::showCurrentPose(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!std::isfinite(norm) || norm <= 0) {
    return;
  }
  const std::array<double, 4> values = {
    mg400_interface::m2mm(pose.position.x), mg400_interface::m2mm(pose.position.y),
    mg400_interface::m2mm(pose.position.z), mg400_interface::rad2degree(tf2::getYaw(q))};
  if (!std::all_of(values.begin(), values.end(), [](double v) {return std::isfinite(v);})) {
    return;
  }
  current_pose_values_ = values;
  have_pose_ = true;
  for (size_t i = 0; i < values.size(); ++i) {
    pose_labels_[i]->setText(QString::number(values[i], 'f', 3));
  }
}

void Mg400ControllerPanel::copyCurrentPose()
{
  if (!have_pose_ || goal_pending_ || service_pending_) {
    return;
  }
  for (size_t i = 0; i < current_pose_values_.size(); ++i) {
    pose_inputs_[i]->setText(QString::number(current_pose_values_[i], 'f', 6));
  }
}

void Mg400ControllerPanel::sendMovJ()
{
  MovJAction::Goal goal;
  if (goal_pending_ || service_pending_ || current_robot_mode_ != RobotMode::ENABLE ||
    !readMovJGoal(goal) || !movj_client_ || !movj_client_->action_server_is_ready())
  {
    return;
  }
  rclcpp_action::Client<MovJAction>::SendGoalOptions options;
  options.goal_response_callback =
    [this](const MovJGoalHandle::SharedPtr & handle) {onGoalResponse("MovJ", bool(handle));};
  options.feedback_callback =
    [this](MovJGoalHandle::SharedPtr, const MovJAction::Feedback::ConstSharedPtr feedback) {
      showCurrentPose(feedback->current_pose.pose);
    };
  options.result_callback =
    [this](const MovJGoalHandle::WrappedResult & result) {
      onResult(
        "MovJ", result.code, result.result && result.result->result,
        result.result ? &result.result->error_id : nullptr);
    };
  goal_pending_ = true;
  label_status_->setText("Sending MovJ...");
  updateControls();
  try {
    movj_client_->async_send_goal(goal, options);
  } catch (const std::exception & e) {
    goal_pending_ = false;
    label_status_->setText(QString("Failed to send MovJ: %1").arg(e.what()));
    updateControls();
  }
}

template<typename ServiceT>
void Mg400ControllerPanel::sendRobotRequest(
  const typename rclcpp::Client<ServiceT>::SharedPtr & client, const QString & name)
{
  if (service_pending_ || !client || !client->service_is_ready()) {
    return;
  }
  service_pending_ = true;
  service_deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  label_service_status_->setText(name + " requested...");
  updateControls();
  try {
    auto future = client->async_send_request(
      std::make_shared<typename ServiceT::Request>(),
      [this, name](typename rclcpp::Client<ServiceT>::SharedFuture response) {
        service_pending_ = false;
        remove_service_request_ = {};
        try {
          const auto result = response.get();
          label_service_status_->setText(
            result->result ? name + " succeeded." :
            QString("%1 failed (error ID: %2).").arg(name).arg(result->error_id));
        } catch (const std::exception & e) {
          label_service_status_->setText(QString("%1 failed: %2").arg(name).arg(e.what()));
        }
        updateControls();
      });
    const auto request_id = future.request_id;
    remove_service_request_ = [client, request_id]() {client->remove_pending_request(request_id);};
  } catch (const std::exception & e) {
    service_pending_ = false;
    label_service_status_->setText(QString("%1 failed: %2").arg(name).arg(e.what()));
    updateControls();
  }
}

void Mg400ControllerPanel::onGoalResponse(const QString & action, bool accepted)
{
  if (accepted) {
    label_status_->setText(action + " accepted; moving...");
  } else {
    goal_pending_ = false;
    label_status_->setText(action + " rejected by server.");
  }
  updateControls();
}

void Mg400ControllerPanel::onResult(
  const QString & action, rclcpp_action::ResultCode code, bool succeeded,
  const mg400_msgs::msg::ErrorID * error_ids)
{
  goal_pending_ = false;
  QString status;
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      status = succeeded ? " succeeded." : " failed.";
      break;
    case rclcpp_action::ResultCode::ABORTED: status = " aborted."; break;
    case rclcpp_action::ResultCode::CANCELED: status = " canceled."; break;
    default: status = " returned an unknown result."; break;
  }
  if (error_ids) {
    QStringList errors;
    for (const auto id : error_ids->controller.ids) {
      errors << QString("controller: %1").arg(id);
    }
    for (size_t i = 0; i < error_ids->servo.size(); ++i) {
      for (const auto id : error_ids->servo[i].ids) {
        errors << QString("servo %1: %2").arg(i + 1).arg(id);
      }
    }
    if (!errors.empty()) {
      status += " " + errors.join(", ");
    }
  }
  label_status_->setText(action + status);
  updateControls();
}
}  // namespace mg400_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mg400_rviz_plugin::Mg400ControllerPanel, rviz_common::Panel)
