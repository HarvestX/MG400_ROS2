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

#include <chrono>
#include <cmath>
#include <cstdint>

namespace mg400_rviz_plugin
{
namespace
{
constexpr double kMillimetersToMeters = 1e-3;
constexpr double kMetersToMillimeters = 1e3;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kRadiansToDegrees = 180.0 / kPi;
constexpr int kPercentParamDefault = 50;
constexpr int kPercentParamMin = 1;
constexpr int kPercentParamMax = 100;
constexpr int kCpParamDefault = 0;
constexpr int kCpParamMin = 0;
constexpr int kCpParamMax = 100;
constexpr int kAxisLabelWidth = 28;
constexpr int kUnitLabelWidth = 72;
constexpr int kActualValueWidth = 96;
constexpr int kGoalInputWidth = 72;
constexpr int kJogButtonWidth = 36;
constexpr int kJogServiceTimeoutMs = 200;

template<typename ServiceT, typename RequestSetupT>
bool callParamService(
  const rclcpp::Node::SharedPtr & node,
  rclcpp::executors::SingleThreadedExecutor & executor,
  const typename rclcpp::Client<ServiceT>::SharedPtr & client,
  const std::string & display_name,
  RequestSetupT setup_request,
  QStringList & sent,
  QStringList & failed)
{
  if (!node || !client) {
    failed.append(QString::fromStdString(display_name + ": no client"));
    return false;
  }

  using namespace std::chrono_literals;  // NOLINT
  if (!client->wait_for_service(1s)) {
    failed.append(QString::fromStdString(display_name + ": service not ready"));
    RCLCPP_ERROR(
      node->get_logger(), "\"%s\" is not ready", client->get_service_name());
    return false;
  }

  auto request = std::make_shared<typename ServiceT::Request>();
  setup_request(*request);
  auto future_result = client->async_send_request(request);

  if (executor.spin_until_future_complete(future_result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    failed.append(QString::fromStdString(display_name + ": request failed"));
    RCLCPP_ERROR(
      node->get_logger(), "\"%s\" service client: async_send_request failed",
      client->get_service_name());
    return false;
  }

  const auto response = future_result.get();
  if (!response->result) {
    failed.append(
      QString("%1: error_id=%2")
      .arg(QString::fromStdString(display_name))
      .arg(response->error_id));
    RCLCPP_ERROR(
      node->get_logger(), "\"%s\" service client: failed, error_id=%d",
      client->get_service_name(), response->error_id);
    return false;
  }

  sent.append(QString::fromStdString(display_name));
  return true;
}

const char * robotModeToString(const uint64_t mode)
{
  using RobotMode = mg400_msgs::msg::RobotMode;
  switch (mode) {
    case RobotMode::INIT:
      return "INIT";
    case RobotMode::BRAKE_OPEN:
      return "BRAKE_OPEN";
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
}

QString jogModeToString(const std::string & jog_mode)
{
  if (jog_mode.empty()) {
    return "STOP";
  }
  return QString::fromStdString(jog_mode);
}

}  // namespace

MG400InputGroup::MG400InputGroup(const std::string & prefix, const std::string & suffix)
{
  this->setSpacing(4);
  this->setContentsMargins(0, 0, 0, 0);

  this->prefix_ = new QLabel(prefix.data());
  this->prefix_->setFixedWidth(kAxisLabelWidth);
  this->addWidget(this->prefix_);
  this->suffix_ = new QLabel(suffix.data());
  this->suffix_->setFixedWidth(kUnitLabelWidth);
  this->addWidget(this->suffix_);
  this->joint_state_ = new QLabel("");
  this->joint_state_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  this->joint_state_->setFixedWidth(kActualValueWidth);
  this->addWidget(this->joint_state_);
  this->l_edit_ = new QLineEdit("");
  this->l_edit_->setFixedWidth(kGoalInputWidth);
  this->l_edit_->setPlaceholderText("0.0");
  this->addWidget(this->l_edit_);
  this->addStretch();
}

void MG400InputGroup::addJogButtons(QPushButton * positive_button, QPushButton * negative_button)
{
  if (!positive_button || !negative_button) {
    return;
  }

  positive_button->setFixedWidth(kJogButtonWidth);
  negative_button->setFixedWidth(kJogButtonWidth);

  const int insert_index = this->count() > 0 ? this->count() - 1 : 0;
  this->insertWidget(insert_index, positive_button);
  this->insertWidget(insert_index + 1, negative_button);
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
: Mg400PanelBase(parent),
  current_robot_mode_(mg400_msgs::msg::RobotMode::INIT),
  current_joint_state_({0.0, 0.0, 0.0, 0.0}),
  has_actual_state_(false)
{
  QVBoxLayout * layout = new QVBoxLayout;
  layout->addLayout(this->createNamespaceLayout());

  this->current_pose_.orientation.w = 1.0;

  auto * section_layout = new QHBoxLayout;
  section_layout->addWidget(this->createRobotSection(), 1);
  section_layout->addWidget(this->createMotionParamsSection(), 1);
  section_layout->addWidget(this->createPoseSection(), 2);
  section_layout->addWidget(this->createJointSection(), 2);
  layout->addLayout(section_layout);
  layout->addStretch();

  this->setLayout(layout);

  QTimer * output_timer = new QTimer(this);
  this->connect(output_timer, SIGNAL(timeout()), this, SLOT(tick()));
  output_timer->start(100);  // Timeout set to 100ms = 0.1s
  this->connect(this->button_enable_, SIGNAL(clicked()), this, SLOT(callbackEnableRobot()));
  this->connect(this->button_disable_, SIGNAL(clicked()), this, SLOT(callbackDisableRobot()));
  this->connect(this->button_clear_error_, SIGNAL(clicked()), this, SLOT(callbackClearError()));
  this->connect(this->button_send_params_, SIGNAL(clicked()), this, SLOT(callbackSendParams()));
  this->connect(this->button_send_movj_, SIGNAL(clicked()), this, SLOT(callbackSendMovJ()));
  this->connect(this->button_send_movl_, SIGNAL(clicked()), this, SLOT(callbackSendMovL()));
  this->connect(
    this->button_send_joint_movj_, SIGNAL(clicked()), this, SLOT(callbackSendJointMovJ()));
}

QWidget * Mg400ControllerPanel::createRobotSection()
{
  auto * group = new QGroupBox("Robot", this);
  auto * layout = new QVBoxLayout;

  QHBoxLayout * layout_mode = new QHBoxLayout;
  layout_mode->addWidget(new QLabel("Robot Mode:", group));
  label_mode_ = new QLabel(group);
  label_mode_->setMinimumWidth(100);
  layout_mode->addWidget(label_mode_);
  layout_mode->addStretch();
  layout->addLayout(layout_mode);

  QGridLayout * layout_mode_buttons = new QGridLayout;
  layout_mode_buttons->setHorizontalSpacing(6);
  layout_mode_buttons->setVerticalSpacing(6);
  this->button_enable_ = new QPushButton("Enable", group);
  this->button_disable_ = new QPushButton("Disable", group);
  this->button_clear_error_ = new QPushButton("Clear Error", group);
  auto * stop_button = new QPushButton("Stop", group);
  this->jog_buttons_.push_back(stop_button);
  connect(
    stop_button, &QPushButton::clicked, this, [this]() {
      this->sendMoveJog(MoveJog::STOP);
    });
  layout_mode_buttons->addWidget(this->button_enable_, 0, 0);
  layout_mode_buttons->addWidget(this->button_disable_, 0, 1);
  layout_mode_buttons->addWidget(this->button_clear_error_, 1, 0);
  layout_mode_buttons->addWidget(stop_button, 1, 1);
  layout->addLayout(layout_mode_buttons);

  QHBoxLayout * layout_status = new QHBoxLayout;
  layout_status->addWidget(new QLabel("Action:", group));
  label_action_status_ = new QLabel("Idle", group);
  label_action_status_->setWordWrap(true);
  layout_status->addWidget(label_action_status_, 1);
  layout->addLayout(layout_status);

  QHBoxLayout * layout_jog_status = new QHBoxLayout;
  layout_jog_status->addWidget(new QLabel("Jog:", group));
  label_jog_status_ = new QLabel("Idle", group);
  label_jog_status_->setWordWrap(true);
  layout_jog_status->addWidget(label_jog_status_, 1);
  layout->addLayout(layout_jog_status);
  layout->addStretch();

  group->setLayout(layout);
  return group;
}

QWidget * Mg400ControllerPanel::createMotionParamsSection()
{
  auto * group = new QGroupBox("Motion Params", this);
  auto * layout = new QVBoxLayout;
  auto * grid = new QGridLayout;

  grid->setHorizontalSpacing(10);
  grid->setVerticalSpacing(6);

  auto add_param_cell =
    [grid, group](const int row, const int column, QCheckBox * & checkbox, QSpinBox * & spinbox,
      const QString & label, const int value, const bool cp = false) {
      auto * cell = new QWidget(group);
      auto * cell_layout = new QHBoxLayout(cell);
      cell_layout->setContentsMargins(0, 0, 0, 0);
      cell_layout->setSpacing(4);

      checkbox = new QCheckBox(label, cell);
      checkbox->setMinimumWidth(92);
      spinbox = new QSpinBox(cell);
      spinbox->setRange(cp ? kCpParamMin : kPercentParamMin, cp ? kCpParamMax : kPercentParamMax);
      spinbox->setValue(value);
      spinbox->setSuffix(cp ? "" : "%");
      spinbox->setEnabled(false);
      spinbox->setFixedWidth(72);
      QObject::connect(checkbox, SIGNAL(toggled(bool)), spinbox, SLOT(setEnabled(bool)));

      cell_layout->addWidget(checkbox);
      cell_layout->addWidget(spinbox);
      cell_layout->addStretch();
      grid->addWidget(cell, row, column);
    };

  add_param_cell(
    0, 0, checkbox_speed_factor_, spin_speed_factor_, "SpeedFactor", kPercentParamDefault);
  add_param_cell(0, 1, checkbox_cp_, spin_cp_, "CP", kCpParamDefault, true);
  add_param_cell(1, 0, checkbox_speed_j_, spin_speed_j_, "SpeedJ", kPercentParamDefault);
  add_param_cell(1, 1, checkbox_acc_j_, spin_acc_j_, "AccJ", kPercentParamDefault);
  add_param_cell(2, 0, checkbox_speed_l_, spin_speed_l_, "SpeedL", kPercentParamDefault);
  add_param_cell(2, 1, checkbox_acc_l_, spin_acc_l_, "AccL", kPercentParamDefault);

  layout->addLayout(grid);

  this->button_send_params_ = new QPushButton("Send Params", group);
  layout->addWidget(this->button_send_params_);

  this->label_param_status_ = new QLabel("Idle", group);
  this->label_param_status_->setWordWrap(true);
  layout->addWidget(this->label_param_status_);
  layout->addStretch();

  group->setLayout(layout);
  return group;
}

QWidget * Mg400ControllerPanel::createPoseSection()
{
  auto * group = new QGroupBox("Pose / MovJ / MovL", this);
  auto * layout = new QVBoxLayout;

  QHBoxLayout * layout_pose_index = new QHBoxLayout;
  layout_pose_index->addWidget(new QLabel("Current Pose / Goal Pose", group));
  layout_pose_index->addStretch();
  layout->addLayout(layout_pose_index);

  QHBoxLayout * layout_movj = new QHBoxLayout;
  QVBoxLayout * layout_movj_goal = new QVBoxLayout;

  this->input_x_ = new MG400InputGroup("X", "[mm]");
  this->addJogButtons(this->input_x_, group, MoveJog::X_NEGATIVE, MoveJog::X_POSITIVE);
  layout_movj_goal->addLayout(this->input_x_);

  this->input_y_ = new MG400InputGroup("Y", "[mm]");
  this->addJogButtons(this->input_y_, group, MoveJog::Y_NEGATIVE, MoveJog::Y_POSITIVE);
  layout_movj_goal->addLayout(this->input_y_);

  this->input_z_ = new MG400InputGroup("Z", "[mm]");
  this->addJogButtons(this->input_z_, group, MoveJog::Z_NEGATIVE, MoveJog::Z_POSITIVE);
  layout_movj_goal->addLayout(this->input_z_);

  this->input_r_ = new MG400InputGroup("Rx", "[degree]");
  this->addJogButtons(this->input_r_, group, MoveJog::RX_NEGATIVE, MoveJog::RX_POSITIVE);
  layout_movj_goal->addLayout(this->input_r_);

  layout_movj->addLayout(layout_movj_goal);
  QVBoxLayout * layout_pose_buttons = new QVBoxLayout;
  this->button_send_movj_ = new QPushButton("Send MovJ", group);
  layout_pose_buttons->addWidget(this->button_send_movj_);
  this->button_send_movl_ = new QPushButton("Send MovL", group);
  layout_pose_buttons->addWidget(this->button_send_movl_);
  layout_pose_buttons->addStretch();
  layout_movj->addLayout(layout_pose_buttons);

  layout->addLayout(layout_movj);
  layout->addStretch();

  group->setLayout(layout);
  return group;
}

QWidget * Mg400ControllerPanel::createJointSection()
{
  auto * group = new QGroupBox("Joint / JointMovJ", this);
  auto * layout = new QVBoxLayout;

  QHBoxLayout * layout_joint_index = new QHBoxLayout;
  layout_joint_index->addWidget(new QLabel("Current Joint / Goal Joint", group));
  layout_joint_index->addStretch();
  layout->addLayout(layout_joint_index);

  QHBoxLayout * layout_joint_movj = new QHBoxLayout;
  QVBoxLayout * layout_joint_goal = new QVBoxLayout;

  this->input_j1_ = new MG400InputGroup("J1", "[degree]");
  this->addJogButtons(this->input_j1_, group, MoveJog::J1_NEGATIVE, MoveJog::J1_POSITIVE);
  layout_joint_goal->addLayout(this->input_j1_);

  this->input_j2_ = new MG400InputGroup("J2", "[degree]");
  this->addJogButtons(this->input_j2_, group, MoveJog::J2_NEGATIVE, MoveJog::J2_POSITIVE);
  layout_joint_goal->addLayout(this->input_j2_);

  this->input_j3_ = new MG400InputGroup("J3", "[degree]");
  this->addJogButtons(this->input_j3_, group, MoveJog::J3_NEGATIVE, MoveJog::J3_POSITIVE);
  layout_joint_goal->addLayout(this->input_j3_);

  this->input_j4_ = new MG400InputGroup("J4", "[degree]");
  this->addJogButtons(this->input_j4_, group, MoveJog::J4_NEGATIVE, MoveJog::J4_POSITIVE);
  layout_joint_goal->addLayout(this->input_j4_);

  layout_joint_movj->addLayout(layout_joint_goal);
  this->button_send_joint_movj_ = new QPushButton("Send JointMovJ", group);
  layout_joint_movj->addWidget(this->button_send_joint_movj_);

  layout->addLayout(layout_joint_movj);
  layout->addStretch();

  group->setLayout(layout);
  return group;
}

void Mg400ControllerPanel::onRosNodeInitialized()
{
  this->setupRosInterfaces();
}

void Mg400ControllerPanel::onNamespaceChanged()
{
  this->setupRosInterfaces();
}

void Mg400ControllerPanel::setupRosInterfaces()
{
  if (!this->rawNode()) {
    return;
  }

  this->current_robot_mode_ = mg400_msgs::msg::RobotMode::INIT;
  {
    const std::lock_guard<std::mutex> lock(this->actual_state_mutex_);
    this->current_pose_ = geometry_msgs::msg::Pose();
    this->current_pose_.orientation.w = 1.0;
    this->current_joint_state_ = {0.0, 0.0, 0.0, 0.0};
    this->has_actual_state_ = false;
  }

  rm_sub_ = this->rawNode()->create_subscription<RobotMode>(
    this->makeTopic("robot_mode"), rclcpp::SensorDataQoS().keep_last(1),
    [&](const mg400_msgs::msg::RobotMode::ConstSharedPtr msg) {
      this->current_robot_mode_ = msg->robot_mode;
    });
  realtime_feedback_sub_ = this->rawNode()->create_subscription<RealtimeFeedback>(
    this->makeTopic("realtime_feedback"), rclcpp::SensorDataQoS().keep_last(1),
    [&](const RealtimeFeedback::ConstSharedPtr msg) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = msg->tool_vector_actual[0] * kMillimetersToMeters;
      pose.position.y = msg->tool_vector_actual[1] * kMillimetersToMeters;
      pose.position.z = msg->tool_vector_actual[2] * kMillimetersToMeters;

      tf2::Quaternion quat;
      quat.setRPY(0.0, 0.0, msg->tool_vector_actual[3] * kDegreesToRadians);
      pose.orientation.x = quat.getX();
      pose.orientation.y = quat.getY();
      pose.orientation.z = quat.getZ();
      pose.orientation.w = quat.getW();

      const std::array<double, 4> joint_state = {
        msg->q_actual[0] * kDegreesToRadians,
        msg->q_actual[1] * kDegreesToRadians,
        msg->q_actual[2] * kDegreesToRadians,
        msg->q_actual[3] * kDegreesToRadians
      };

      const std::lock_guard<std::mutex> lock(this->actual_state_mutex_);
      this->current_pose_ = pose;
      this->current_joint_state_ = joint_state;
      this->has_actual_state_ = true;
    });

  mg400_enable_robot_clnt_ =
    this->rawNode()->create_client<mg400_msgs::srv::EnableRobot>(
    this->makeService("enable_robot"), rmw_qos_profile_default, this->callbackGroup());
  mg400_disable_robot_clnt_ =
    this->rawNode()->create_client<mg400_msgs::srv::DisableRobot>(
    this->makeService("disable_robot"), rmw_qos_profile_default, this->callbackGroup());
  mg400_clear_error_clnt_ =
    this->rawNode()->create_client<mg400_msgs::srv::ClearError>(
    this->makeService("clear_error"), rmw_qos_profile_default, this->callbackGroup());
  mg400_speed_factor_clnt_ =
    this->rawNode()->create_client<mg400_msgs::srv::SpeedFactor>(
    this->makeService("speed_factor"), rmw_qos_profile_default, this->callbackGroup());
  mg400_speed_j_clnt_ =
    this->rawNode()->create_client<mg400_msgs::srv::SpeedJ>(
    this->makeService("speed_j"), rmw_qos_profile_default, this->callbackGroup());
  mg400_speed_l_clnt_ =
    this->rawNode()->create_client<mg400_msgs::srv::SpeedL>(
    this->makeService("speed_l"), rmw_qos_profile_default, this->callbackGroup());
  mg400_acc_j_clnt_ =
    this->rawNode()->create_client<mg400_msgs::srv::AccJ>(
    this->makeService("acc_j"), rmw_qos_profile_default, this->callbackGroup());
  mg400_acc_l_clnt_ =
    this->rawNode()->create_client<mg400_msgs::srv::AccL>(
    this->makeService("acc_l"), rmw_qos_profile_default, this->callbackGroup());
  mg400_cp_clnt_ =
    this->rawNode()->create_client<mg400_msgs::srv::CP>(
    this->makeService("cp"), rmw_qos_profile_default, this->callbackGroup());
  mg400_move_jog_clnt_ =
    this->rawNode()->create_client<MoveJogService>(
    this->makeService("move_jog"), rmw_qos_profile_default, this->callbackGroup());

  this->mg400_movj_clnt_ = rclcpp_action::create_client<MovJAction>(
    this->rawNode(), this->makeAction("mov_j"), this->callbackGroup());
  this->mg400_movl_clnt_ = rclcpp_action::create_client<MovLAction>(
    this->rawNode(), this->makeAction("mov_l"), this->callbackGroup());
  this->mg400_joint_movj_clnt_ = rclcpp_action::create_client<JointMovJAction>(
    this->rawNode(), this->makeAction("joint_mov_j"), this->callbackGroup());
}

void Mg400ControllerPanel::tick()
{
  const auto robot_enabled = this->current_robot_mode_ == RobotMode::ENABLE;
  const auto jog_enabled = robot_enabled || this->current_robot_mode_ == RobotMode::JOG;
  if (robot_enabled) {
    this->input_x_->enableLine();
    this->input_y_->enableLine();
    this->input_z_->enableLine();
    this->input_r_->enableLine();
    this->input_j1_->enableLine();
    this->input_j2_->enableLine();
    this->input_j3_->enableLine();
    this->input_j4_->enableLine();
    this->label_mode_->setStyleSheet("color: green;");
  } else if (this->current_robot_mode_ == RobotMode::DISABLED) {
    this->input_x_->disableLine();
    this->input_y_->disableLine();
    this->input_z_->disableLine();
    this->input_r_->disableLine();
    this->input_j1_->disableLine();
    this->input_j2_->disableLine();
    this->input_j3_->disableLine();
    this->input_j4_->disableLine();
    this->label_mode_->setStyleSheet("color: blue;");
  } else {
    this->input_x_->disableLine();
    this->input_y_->disableLine();
    this->input_z_->disableLine();
    this->input_r_->disableLine();
    this->input_j1_->disableLine();
    this->input_j2_->disableLine();
    this->input_j3_->disableLine();
    this->input_j4_->disableLine();
    this->label_mode_->setStyleSheet("");
  }

  this->button_enable_->setEnabled(this->current_robot_mode_ == RobotMode::DISABLED);
  this->button_disable_->setEnabled(robot_enabled);
  this->button_clear_error_->setEnabled(
    static_cast<bool>(this->rawNode()) && static_cast<bool>(this->mg400_clear_error_clnt_));
  this->button_send_params_->setEnabled(static_cast<bool>(this->rawNode()));
  this->button_send_movj_->setEnabled(robot_enabled);
  this->button_send_movl_->setEnabled(robot_enabled);
  this->button_send_joint_movj_->setEnabled(robot_enabled);
  for (auto * button : this->jog_buttons_) {
    if (button) {
      button->setEnabled(jog_enabled && static_cast<bool>(this->mg400_move_jog_clnt_));
    }
  }
  this->label_mode_->setText(robotModeToString(this->current_robot_mode_));

  geometry_msgs::msg::Pose current_pose;
  std::array<double, 4> current_joint_state;
  bool has_actual_state = false;
  {
    const std::lock_guard<std::mutex> lock(this->actual_state_mutex_);
    current_pose = this->current_pose_;
    current_joint_state = this->current_joint_state_;
    has_actual_state = this->has_actual_state_;
  }

  if (!has_actual_state) {
    this->input_x_->joint_state_->setText("");
    this->input_y_->joint_state_->setText("");
    this->input_z_->joint_state_->setText("");
    this->input_r_->joint_state_->setText("");
    this->input_j1_->joint_state_->setText("");
    this->input_j2_->joint_state_->setText("");
    this->input_j3_->joint_state_->setText("");
    this->input_j4_->joint_state_->setText("");
    return;
  }

  this->input_x_->joint_state_->setText(
    QString::number(current_pose.position.x * kMetersToMillimeters, 'f', 1));
  this->input_y_->joint_state_->setText(
    QString::number(current_pose.position.y * kMetersToMillimeters, 'f', 1));
  this->input_z_->joint_state_->setText(
    QString::number(current_pose.position.z * kMetersToMillimeters, 'f', 1));
  tf2::Quaternion q(
    current_pose.orientation.x,
    current_pose.orientation.y,
    current_pose.orientation.z,
    current_pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  this->input_r_->joint_state_->setText(QString::number(yaw * kRadiansToDegrees, 'f', 1));

  auto set_joint_state_text =
    [&current_joint_state](MG400InputGroup * input, const size_t index) {
      input->joint_state_->setText(
        QString::number(current_joint_state[index] * kRadiansToDegrees, 'f', 1));
    };
  set_joint_state_text(this->input_j1_, 0);
  set_joint_state_text(this->input_j2_, 1);
  set_joint_state_text(this->input_j3_, 2);
  set_joint_state_text(this->input_j4_, 3);
}

void Mg400ControllerPanel::callbackSendMovJ()
{
  if (!this->mg400_movj_clnt_) {
    return;
  }
  if (!this->mg400_movj_clnt_->action_server_is_ready()) {
    const auto action_name = this->makeAction("mov_j");
    this->updateActionStatus("MovJ action server is not ready");
    RCLCPP_ERROR(
      this->rawNode()->get_logger(), "\"%s\" action server is not ready",
      action_name.c_str());
    return;
  }

  auto goal_msg = MovJAction::Goal();
  goal_msg.pose = this->createGoalPose();
  this->applyJointMotionParams(goal_msg);
  this->updateActionStatus("Sending MovJ goal");

  auto send_goal_option = rclcpp_action::Client<MovJAction>::SendGoalOptions();

  using namespace std::placeholders; // NOLINT
  send_goal_option.goal_response_callback =
    std::bind(&Mg400ControllerPanel::onMovJGoalResponse, this, _1);
  send_goal_option.feedback_callback =
    std::bind(&Mg400ControllerPanel::onMovJFeedback, this, _1, _2);
  send_goal_option.result_callback =
    std::bind(&Mg400ControllerPanel::onMovJResult, this, _1);

  mg400_movj_clnt_->async_send_goal(goal_msg, send_goal_option);
}

void Mg400ControllerPanel::callbackSendMovL()
{
  if (!this->mg400_movl_clnt_) {
    return;
  }
  if (!this->mg400_movl_clnt_->action_server_is_ready()) {
    const auto action_name = this->makeAction("mov_l");
    this->updateActionStatus("MovL action server is not ready");
    RCLCPP_ERROR(
      this->rawNode()->get_logger(), "\"%s\" action server is not ready",
      action_name.c_str());
    return;
  }

  auto goal_msg = MovLAction::Goal();
  goal_msg.pose = this->createGoalPose();
  this->applyLinearMotionParams(goal_msg);
  this->updateActionStatus("Sending MovL goal");

  auto send_goal_option = rclcpp_action::Client<MovLAction>::SendGoalOptions();

  using namespace std::placeholders; // NOLINT
  send_goal_option.goal_response_callback =
    std::bind(&Mg400ControllerPanel::onMovLGoalResponse, this, _1);
  send_goal_option.feedback_callback =
    std::bind(&Mg400ControllerPanel::onMovLFeedback, this, _1, _2);
  send_goal_option.result_callback =
    std::bind(&Mg400ControllerPanel::onMovLResult, this, _1);

  mg400_movl_clnt_->async_send_goal(goal_msg, send_goal_option);
}

void Mg400ControllerPanel::callbackSendJointMovJ()
{
  if (!this->mg400_joint_movj_clnt_) {
    return;
  }
  if (!this->mg400_joint_movj_clnt_->action_server_is_ready()) {
    const auto action_name = this->makeAction("joint_mov_j");
    this->updateActionStatus("JointMovJ action server is not ready");
    RCLCPP_ERROR(
      this->rawNode()->get_logger(), "\"%s\" action server is not ready",
      action_name.c_str());
    return;
  }

  auto goal_msg = JointMovJAction::Goal();
  goal_msg.joint_angles[0] = this->input_j1_->getValue() * kDegreesToRadians;
  goal_msg.joint_angles[1] = this->input_j2_->getValue() * kDegreesToRadians;
  goal_msg.joint_angles[2] = this->input_j3_->getValue() * kDegreesToRadians;
  goal_msg.joint_angles[3] = this->input_j4_->getValue() * kDegreesToRadians;
  this->applyJointMotionParams(goal_msg);
  this->updateActionStatus("Sending JointMovJ goal");

  auto send_goal_option = rclcpp_action::Client<JointMovJAction>::SendGoalOptions();

  using namespace std::placeholders; // NOLINT
  send_goal_option.goal_response_callback =
    std::bind(&Mg400ControllerPanel::onJointMovJGoalResponse, this, _1);
  send_goal_option.feedback_callback =
    std::bind(&Mg400ControllerPanel::onJointMovJFeedback, this, _1, _2);
  send_goal_option.result_callback =
    std::bind(&Mg400ControllerPanel::onJointMovJResult, this, _1);

  mg400_joint_movj_clnt_->async_send_goal(goal_msg, send_goal_option);
}

void Mg400ControllerPanel::callbackSendParams()
{
  if (!this->rawNode()) {
    return;
  }

  QStringList sent;
  QStringList failed;

  if (this->checkbox_speed_factor_->isChecked()) {
    callParamService<mg400_msgs::srv::SpeedFactor>(
      this->rawNode(), this->callbackGroupExecutor(), this->mg400_speed_factor_clnt_,
      "SpeedFactor",
      [this](mg400_msgs::srv::SpeedFactor::Request & request) {
        request.ratio = static_cast<uint8_t>(this->spin_speed_factor_->value());
      },
      sent, failed);
  }

  if (this->checkbox_speed_j_->isChecked()) {
    callParamService<mg400_msgs::srv::SpeedJ>(
      this->rawNode(), this->callbackGroupExecutor(), this->mg400_speed_j_clnt_,
      "SpeedJ",
      [this](mg400_msgs::srv::SpeedJ::Request & request) {
        request.r = static_cast<uint8_t>(this->spin_speed_j_->value());
      },
      sent, failed);
  }

  if (this->checkbox_speed_l_->isChecked()) {
    callParamService<mg400_msgs::srv::SpeedL>(
      this->rawNode(), this->callbackGroupExecutor(), this->mg400_speed_l_clnt_,
      "SpeedL",
      [this](mg400_msgs::srv::SpeedL::Request & request) {
        request.r = static_cast<uint8_t>(this->spin_speed_l_->value());
      },
      sent, failed);
  }

  if (this->checkbox_acc_j_->isChecked()) {
    callParamService<mg400_msgs::srv::AccJ>(
      this->rawNode(), this->callbackGroupExecutor(), this->mg400_acc_j_clnt_,
      "AccJ",
      [this](mg400_msgs::srv::AccJ::Request & request) {
        request.r = static_cast<uint8_t>(this->spin_acc_j_->value());
      },
      sent, failed);
  }

  if (this->checkbox_acc_l_->isChecked()) {
    callParamService<mg400_msgs::srv::AccL>(
      this->rawNode(), this->callbackGroupExecutor(), this->mg400_acc_l_clnt_,
      "AccL",
      [this](mg400_msgs::srv::AccL::Request & request) {
        request.r = static_cast<uint8_t>(this->spin_acc_l_->value());
      },
      sent, failed);
  }

  if (this->checkbox_cp_->isChecked()) {
    callParamService<mg400_msgs::srv::CP>(
      this->rawNode(), this->callbackGroupExecutor(), this->mg400_cp_clnt_,
      "CP",
      [this](mg400_msgs::srv::CP::Request & request) {
        request.r = static_cast<uint16_t>(this->spin_cp_->value());
      },
      sent, failed);
  }

  if (sent.isEmpty() && failed.isEmpty()) {
    this->updateParamStatus("No params selected");
    return;
  }

  QString status;
  if (!sent.isEmpty()) {
    status += "Sent: " + sent.join(", ");
  }
  if (!failed.isEmpty()) {
    if (!status.isEmpty()) {
      status += "\n";
    }
    status += "Failed: " + failed.join(", ");
  }
  this->updateParamStatus(status);
}

void Mg400ControllerPanel::addJogButtons(
  MG400InputGroup * input, QWidget * parent,
  const std::string & negative_mode, const std::string & positive_mode)
{
  if (!input) {
    return;
  }

  auto * positive_button = new QPushButton("+", parent);
  positive_button->setToolTip(QString("MoveJog %1").arg(jogModeToString(positive_mode)));
  this->connectJogButton(positive_button, positive_mode);

  auto * negative_button = new QPushButton("-", parent);
  negative_button->setToolTip(QString("MoveJog %1").arg(jogModeToString(negative_mode)));
  this->connectJogButton(negative_button, negative_mode);

  input->addJogButtons(positive_button, negative_button);
}

void Mg400ControllerPanel::connectJogButton(QPushButton * button, const std::string & jog_mode)
{
  if (!button) {
    return;
  }

  this->jog_buttons_.push_back(button);
  connect(
    button, &QPushButton::pressed, this, [this, jog_mode]() {
      this->sendMoveJog(jog_mode);
    });
  connect(
    button, &QPushButton::released, this, [this]() {
      this->sendMoveJog(MoveJog::STOP);
    });
}

void Mg400ControllerPanel::sendMoveJog(const std::string & jog_mode)
{
  if (!this->rawNode() || !this->mg400_move_jog_clnt_) {
    return;
  }

  const auto display_mode = jogModeToString(jog_mode);
  if (!this->mg400_move_jog_clnt_->service_is_ready()) {
    this->updateJogStatus("MoveJog service is not ready");
    RCLCPP_ERROR(
      this->rawNode()->get_logger(), "\"%s\" service is not ready",
      this->mg400_move_jog_clnt_->get_service_name());
    return;
  }

  auto request = std::make_shared<MoveJogService::Request>();
  request->jog.jog_mode = jog_mode;
  this->updateJogStatus(QString("Sending MoveJog %1").arg(display_mode));

  auto future_result = this->mg400_move_jog_clnt_->async_send_request(request);
  const auto future_status = this->callbackGroupExecutor().spin_until_future_complete(
    future_result, std::chrono::milliseconds(kJogServiceTimeoutMs));
  if (future_status != rclcpp::FutureReturnCode::SUCCESS) {
    this->mg400_move_jog_clnt_->remove_pending_request(future_result);
    this->updateJogStatus(QString("MoveJog %1 timed out").arg(display_mode));
    RCLCPP_WARN(
      this->rawNode()->get_logger(), "\"%s\" service client: response timeout",
      this->mg400_move_jog_clnt_->get_service_name());
    return;
  }

  const auto response = future_result.get();
  if (response->error_id != 0) {
    this->updateJogStatus(
      QString("MoveJog %1 failed: error_id=%2").arg(display_mode).arg(response->error_id));
    RCLCPP_ERROR(
      this->rawNode()->get_logger(), "\"%s\" service client: failed, error_id=%d",
      this->mg400_move_jog_clnt_->get_service_name(), response->error_id);
    return;
  }

  this->updateJogStatus(QString("MoveJog %1 sent").arg(display_mode));
}

void Mg400ControllerPanel::callbackEnableRobot()
{
  if (!this->mg400_enable_robot_clnt_) {
    return;
  }

  using namespace std::chrono_literals;  // NOLINT
  if (!mg400_enable_robot_clnt_->wait_for_service(1s)) {
    RCLCPP_ERROR(
      this->rawNode()->get_logger(), "\"%s\" is not ready",
      mg400_enable_robot_clnt_->get_service_name());
    return;
  }

  auto req = std::make_shared<mg400_msgs::srv::EnableRobot::Request>();

  auto future_result = mg400_enable_robot_clnt_->async_send_request(req);

  if (this->callbackGroupExecutor().spin_until_future_complete(future_result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      this->rawNode()->get_logger(), "\"%s\" service client: async_send_request failed",
      mg400_enable_robot_clnt_->get_service_name());
    return;
  }

  const auto wrapped_result = future_result.get();
  if (!wrapped_result->result) {
    RCLCPP_ERROR(
      this->rawNode()->get_logger(), "\"%s\" service client: failed",
      mg400_enable_robot_clnt_->get_service_name());
  }
}

void Mg400ControllerPanel::callbackDisableRobot()
{
  if (!this->mg400_disable_robot_clnt_) {
    return;
  }

  using namespace std::chrono_literals;  // NOLINT
  if (!mg400_disable_robot_clnt_->wait_for_service(1s)) {
    RCLCPP_ERROR(
      this->rawNode()->get_logger(), "\"%s\" is not ready",
      mg400_disable_robot_clnt_->get_service_name());
    return;
  }

  auto req = std::make_shared<mg400_msgs::srv::DisableRobot::Request>();

  auto future_result = mg400_disable_robot_clnt_->async_send_request(req);

  if (this->callbackGroupExecutor().spin_until_future_complete(future_result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      this->rawNode()->get_logger(), "\"%s\" service client: async_send_request failed",
      mg400_disable_robot_clnt_->get_service_name());
    return;
  }

  const auto wrapped_result = future_result.get();
  if (!wrapped_result->result) {
    RCLCPP_ERROR(
      this->rawNode()->get_logger(), "\"%s\" service client: failed",
      mg400_disable_robot_clnt_->get_service_name());
  }
}

void Mg400ControllerPanel::callbackClearError()
{
  if (!this->mg400_clear_error_clnt_) {
    return;
  }

  using namespace std::chrono_literals;  // NOLINT
  if (!mg400_clear_error_clnt_->wait_for_service(1s)) {
    this->updateActionStatus("ClearError service is not ready");
    RCLCPP_ERROR(
      this->rawNode()->get_logger(), "\"%s\" is not ready",
      mg400_clear_error_clnt_->get_service_name());
    return;
  }

  auto req = std::make_shared<mg400_msgs::srv::ClearError::Request>();

  this->updateActionStatus("Sending ClearError request");
  auto future_result = mg400_clear_error_clnt_->async_send_request(req);

  if (this->callbackGroupExecutor().spin_until_future_complete(future_result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    this->updateActionStatus("ClearError request failed");
    RCLCPP_ERROR(
      this->rawNode()->get_logger(), "\"%s\" service client: async_send_request failed",
      mg400_clear_error_clnt_->get_service_name());
    return;
  }

  const auto wrapped_result = future_result.get();
  if (!wrapped_result->result) {
    this->updateActionStatus(
      QString("ClearError failed: error_id=%1").arg(wrapped_result->error_id));
    RCLCPP_ERROR(
      this->rawNode()->get_logger(), "\"%s\" service client: failed, error_id=%d",
      mg400_clear_error_clnt_->get_service_name(), wrapped_result->error_id);
    return;
  }

  this->updateActionStatus("ClearError succeeded");
}

geometry_msgs::msg::PoseStamped Mg400ControllerPanel::createGoalPose()
{
  auto pose = geometry_msgs::msg::PoseStamped();
  pose.header.frame_id = "mg400_origin_link";
  pose.header.stamp.sec = 0;
  pose.header.stamp.nanosec = 0;
  pose.pose.position.x = this->input_x_->getValue() * kMillimetersToMeters;
  pose.pose.position.y = this->input_y_->getValue() * kMillimetersToMeters;
  pose.pose.position.z = this->input_z_->getValue() * kMillimetersToMeters;

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, this->input_r_->getValue() * kDegreesToRadians);
  pose.pose.orientation.x = quat.getX();
  pose.pose.orientation.y = quat.getY();
  pose.pose.orientation.z = quat.getZ();
  pose.pose.orientation.w = quat.getW();

  return pose;
}

void Mg400ControllerPanel::applyJointMotionParams(MovJAction::Goal & goal)
{
  goal.set_speed_j = this->checkbox_speed_j_->isChecked();
  goal.speed_j = static_cast<uint8_t>(this->spin_speed_j_->value());
  goal.set_acc_j = this->checkbox_acc_j_->isChecked();
  goal.acc_j = static_cast<uint8_t>(this->spin_acc_j_->value());
  goal.set_cp = this->checkbox_cp_->isChecked();
  goal.cp = static_cast<uint8_t>(this->spin_cp_->value());
}

void Mg400ControllerPanel::applyLinearMotionParams(MovLAction::Goal & goal)
{
  goal.set_speed_l = this->checkbox_speed_l_->isChecked();
  goal.speed_l = static_cast<uint8_t>(this->spin_speed_l_->value());
  goal.set_acc_l = this->checkbox_acc_l_->isChecked();
  goal.acc_l = static_cast<uint8_t>(this->spin_acc_l_->value());
  goal.set_cp = this->checkbox_cp_->isChecked();
  goal.cp = static_cast<uint8_t>(this->spin_cp_->value());
}

void Mg400ControllerPanel::applyJointMotionParams(JointMovJAction::Goal & goal)
{
  goal.set_speed_j = this->checkbox_speed_j_->isChecked();
  goal.speed_j = static_cast<uint8_t>(this->spin_speed_j_->value());
  goal.set_acc_j = this->checkbox_acc_j_->isChecked();
  goal.acc_j = static_cast<uint8_t>(this->spin_acc_j_->value());
  goal.set_cp = this->checkbox_cp_->isChecked();
  goal.cp = static_cast<uint8_t>(this->spin_cp_->value());
}

void Mg400ControllerPanel::updateActionStatus(const QString & text)
{
  if (!this->label_action_status_) {
    return;
  }
  QMetaObject::invokeMethod(
    this->label_action_status_, "setText", Qt::QueuedConnection, Q_ARG(QString, text));
}

void Mg400ControllerPanel::updateParamStatus(const QString & text)
{
  if (!this->label_param_status_) {
    return;
  }
  QMetaObject::invokeMethod(
    this->label_param_status_, "setText", Qt::QueuedConnection, Q_ARG(QString, text));
}

void Mg400ControllerPanel::updateJogStatus(const QString & text)
{
  if (!this->label_jog_status_) {
    return;
  }
  QMetaObject::invokeMethod(
    this->label_jog_status_, "setText", Qt::QueuedConnection, Q_ARG(QString, text));
}

void Mg400ControllerPanel::handleGoalResponse(
  const std::string & action_name, const bool accepted)
{
  if (!accepted) {
    this->updateActionStatus(QString::fromStdString(action_name + " goal rejected"));
    RCLCPP_ERROR(
      this->rawNode()->get_logger(), "%s goal was rejected by server",
      action_name.c_str());
    return;
  }

  this->updateActionStatus(QString::fromStdString(action_name + " goal accepted"));
  RCLCPP_INFO(
    this->rawNode()->get_logger(), "%s goal accepted by server, waiting for result",
    action_name.c_str());
}

void Mg400ControllerPanel::handlePoseFeedback(
  const std::string & action_name, const geometry_msgs::msg::PoseStamped & pose)
{
  this->updateActionStatus(
    QString("%1 feedback: x=%2 mm, y=%3 mm, z=%4 mm")
    .arg(QString::fromStdString(action_name))
    .arg(pose.pose.position.x * kMetersToMillimeters, 0, 'f', 1)
    .arg(pose.pose.position.y * kMetersToMillimeters, 0, 'f', 1)
    .arg(pose.pose.position.z * kMetersToMillimeters, 0, 'f', 1));
  RCLCPP_INFO(
    this->rawNode()->get_logger(), "%s current pose: (%lf, %lf, %lf)",
    action_name.c_str(),
    pose.pose.position.x,
    pose.pose.position.y,
    pose.pose.position.z);
}

void Mg400ControllerPanel::handleJointFeedback(
  const std::string & action_name, const JointMovJAction::Feedback::ConstSharedPtr feedback)
{
  if (!feedback) {
    return;
  }

  this->updateActionStatus(
    QString("%1 feedback: J1=%2 deg, J2=%3 deg, J3=%4 deg, J4=%5 deg")
    .arg(QString::fromStdString(action_name))
    .arg(feedback->current_angles[0] * kRadiansToDegrees, 0, 'f', 1)
    .arg(feedback->current_angles[1] * kRadiansToDegrees, 0, 'f', 1)
    .arg(feedback->current_angles[2] * kRadiansToDegrees, 0, 'f', 1)
    .arg(feedback->current_angles[3] * kRadiansToDegrees, 0, 'f', 1));
  RCLCPP_INFO(
    this->rawNode()->get_logger(), "%s current angles: (%lf, %lf, %lf, %lf)",
    action_name.c_str(),
    feedback->current_angles[0],
    feedback->current_angles[1],
    feedback->current_angles[2],
    feedback->current_angles[3]);
}

void Mg400ControllerPanel::handleActionResult(
  const std::string & action_name, const rclcpp_action::ResultCode code, const bool result)
{
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      if (result) {
        this->updateActionStatus(QString::fromStdString(action_name + " succeeded"));
        RCLCPP_INFO(this->rawNode()->get_logger(), "%s succeeded", action_name.c_str());
      } else {
        this->updateActionStatus(QString::fromStdString(action_name + " failed"));
        RCLCPP_ERROR(this->rawNode()->get_logger(), "%s failed", action_name.c_str());
      }
      return;
    case rclcpp_action::ResultCode::ABORTED:
      this->updateActionStatus(QString::fromStdString(action_name + " aborted"));
      RCLCPP_ERROR(this->rawNode()->get_logger(), "%s goal was aborted", action_name.c_str());
      return;
    case rclcpp_action::ResultCode::CANCELED:
      this->updateActionStatus(QString::fromStdString(action_name + " canceled"));
      RCLCPP_ERROR(this->rawNode()->get_logger(), "%s goal was canceled", action_name.c_str());
      return;
    default:
      this->updateActionStatus(QString::fromStdString(action_name + " returned unknown result"));
      RCLCPP_ERROR(
        this->rawNode()->get_logger(), "%s returned unknown result code", action_name.c_str());
      return;
  }
}

void Mg400ControllerPanel::onMovJGoalResponse(const MovJGoalHandle::SharedPtr & goal_handle)
{
  this->handleGoalResponse("MovJ", static_cast<bool>(goal_handle));
}

void Mg400ControllerPanel::onMovJFeedback(
  MovJGoalHandle::SharedPtr, const MovJAction::Feedback::ConstSharedPtr feedback)
{
  if (feedback) {
    this->handlePoseFeedback("MovJ", feedback->current_pose);
  }
}

void Mg400ControllerPanel::onMovJResult(const MovJGoalHandle::WrappedResult & result)
{
  this->handleActionResult("MovJ", result.code, result.result && result.result->result);
}

void Mg400ControllerPanel::onMovLGoalResponse(const MovLGoalHandle::SharedPtr & goal_handle)
{
  this->handleGoalResponse("MovL", static_cast<bool>(goal_handle));
}

void Mg400ControllerPanel::onMovLFeedback(
  MovLGoalHandle::SharedPtr, const MovLAction::Feedback::ConstSharedPtr feedback)
{
  if (feedback) {
    this->handlePoseFeedback("MovL", feedback->current_pose);
  }
}

void Mg400ControllerPanel::onMovLResult(const MovLGoalHandle::WrappedResult & result)
{
  this->handleActionResult("MovL", result.code, result.result && result.result->result);
}

void Mg400ControllerPanel::onJointMovJGoalResponse(
  const JointMovJGoalHandle::SharedPtr & goal_handle)
{
  this->handleGoalResponse("JointMovJ", static_cast<bool>(goal_handle));
}

void Mg400ControllerPanel::onJointMovJFeedback(
  JointMovJGoalHandle::SharedPtr, const JointMovJAction::Feedback::ConstSharedPtr feedback)
{
  this->handleJointFeedback("JointMovJ", feedback);
}

void Mg400ControllerPanel::onJointMovJResult(
  const JointMovJGoalHandle::WrappedResult & result)
{
  this->handleActionResult("JointMovJ", result.code, result.result && result.result->result);
}
}  // namespace mg400_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mg400_rviz_plugin::Mg400ControllerPanel, rviz_common::Panel)
