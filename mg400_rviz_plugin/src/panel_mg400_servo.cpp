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

#include "mg400_rviz_plugin/panel_mg400_servo.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace mg400_rviz_plugin
{
namespace
{
constexpr double kMillimetersToMeters = 1e-3;
constexpr double kMetersToMillimeters = 1e3;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kRadiansToDegrees = 180.0 / kPi;
constexpr int kServoControlPeriodMs = 15;
constexpr int kServoServiceTimeoutMs = 500;
constexpr int kTargetButtonWidth = 42;
constexpr double kServoDefaultLinearSpeedMillimetersPerSecond = 20.0;
constexpr double kServoDefaultAngularSpeedDegreesPerSecond = 15.0;
constexpr double kServoDefaultJointSpeedDegreesPerSecond = 15.0;
constexpr double kUnusedServoOption = 0.0;
constexpr double kJoyControlActiveTimeoutSec = 2.0;
constexpr const char * kJoyControlActiveStatus = "Disabled: mg400_joy is active";

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
}  // namespace

Mg400ServoPanel::Mg400ServoPanel(QWidget * parent)
: Mg400PanelBase(parent),
  joy_control_overlay_(nullptr),
  current_robot_mode_(mg400_msgs::msg::RobotMode::INIT),
  current_joint_state_({0.0, 0.0, 0.0, 0.0}),
  has_actual_state_(false),
  servo_control_type_(ServoControlType::SERVO_P),
  servo_mode_enabled_(false),
  servo_target_initialized_(false),
  servo_target_directions_({0, 0, 0, 0}),
  servo_joint_target_initialized_(false),
  servo_joint_target_({0.0, 0.0, 0.0, 0.0}),
  servo_joint_directions_({0, 0, 0, 0}),
  last_servo_target_update_time_(std::chrono::steady_clock::now()),
  joy_control_active_(false),
  last_joy_control_active_time_(std::chrono::steady_clock::time_point::min())
{
  this->current_pose_.orientation.w = 1.0;
  this->servo_target_pose_.orientation.w = 1.0;

  auto * layout = new QVBoxLayout;
  layout->addLayout(this->createNamespaceLayout());
  layout->addWidget(this->createStatusSection());
  layout->addWidget(this->createTargetSection());
  layout->addWidget(this->createJointTargetSection());
  layout->addStretch();
  this->setLayout(layout);

  this->joy_control_overlay_ = new QFrame(this);
  this->joy_control_overlay_->setObjectName("joyControlOverlay");
  this->joy_control_overlay_->setStyleSheet(
    "QFrame#joyControlOverlay { background-color: rgba(0, 0, 0, 175); }"
    "QLabel#joyControlOverlayTitle { color: white; font-size: 18px; font-weight: 700; }"
    "QLabel#joyControlOverlayBody { color: #f2f2f2; font-size: 12px; }");
  auto * overlay_layout = new QVBoxLayout(this->joy_control_overlay_);
  overlay_layout->setContentsMargins(18, 18, 18, 18);
  overlay_layout->addStretch();
  auto * overlay_title = new QLabel("mg400_joy is active", this->joy_control_overlay_);
  overlay_title->setObjectName("joyControlOverlayTitle");
  overlay_title->setAlignment(Qt::AlignCenter);
  overlay_layout->addWidget(overlay_title);
  auto * overlay_body =
    new QLabel(
    "RViz Servo controls are disabled to avoid competing servo targets.",
    this->joy_control_overlay_);
  overlay_body->setObjectName("joyControlOverlayBody");
  overlay_body->setAlignment(Qt::AlignCenter);
  overlay_body->setWordWrap(true);
  overlay_layout->addWidget(overlay_body);
  overlay_layout->addStretch();
  this->joy_control_overlay_->hide();

  auto * output_timer = new QTimer(this);
  this->connect(output_timer, SIGNAL(timeout()), this, SLOT(tick()));
  output_timer->start(100);

  auto * servo_timer = new QTimer(this);
  this->connect(servo_timer, SIGNAL(timeout()), this, SLOT(onServoControlTimer()));
  servo_timer->start(kServoControlPeriodMs);

  this->connect(this->button_servo_mode_, SIGNAL(clicked()), this, SLOT(callbackServoMode()));
  connect(
    this->radio_servo_p_, &QRadioButton::toggled, this, [this](const bool checked) {
      if (checked) {
        this->setServoControlType(ServoControlType::SERVO_P);
      }
    });
  connect(
    this->radio_servo_j_, &QRadioButton::toggled, this, [this](const bool checked) {
      if (checked) {
        this->setServoControlType(ServoControlType::SERVO_J);
      }
    });
}

void Mg400ServoPanel::resizeEvent(QResizeEvent * event)
{
  rviz_common::Panel::resizeEvent(event);
  if (this->joy_control_overlay_) {
    this->joy_control_overlay_->setGeometry(this->rect());
  }
}

QWidget * Mg400ServoPanel::createStatusSection()
{
  auto * group = new QGroupBox("ServoMode", this);
  auto * layout = new QVBoxLayout;

  auto * robot_mode_layout = new QHBoxLayout;
  robot_mode_layout->addWidget(new QLabel("Robot Mode:", group));
  this->label_robot_mode_ = new QLabel("INIT", group);
  this->label_robot_mode_->setMinimumWidth(100);
  robot_mode_layout->addWidget(this->label_robot_mode_);
  robot_mode_layout->addStretch();
  layout->addLayout(robot_mode_layout);

  auto * status_layout = new QHBoxLayout;
  status_layout->addWidget(new QLabel("ServoMode:", group));
  this->label_servo_status_ = new QLabel("Inactive", group);
  this->label_servo_status_->setWordWrap(true);
  status_layout->addWidget(this->label_servo_status_, 1);
  layout->addLayout(status_layout);

  auto * command_layout = new QHBoxLayout;
  command_layout->addWidget(new QLabel("Command:", group));
  this->radio_servo_p_ = new QRadioButton("ServoP", group);
  this->radio_servo_j_ = new QRadioButton("ServoJ", group);
  this->radio_servo_p_->setChecked(true);
  command_layout->addWidget(this->radio_servo_p_);
  command_layout->addWidget(this->radio_servo_j_);
  command_layout->addStretch();
  layout->addLayout(command_layout);

  this->button_servo_mode_ = new QPushButton("Enter ServoMode", group);
  layout->addWidget(this->button_servo_mode_);
  layout->addStretch();

  group->setLayout(layout);
  return group;
}

QWidget * Mg400ServoPanel::createTargetSection()
{
  auto * group = new QGroupBox("ServoP Target", this);
  auto * layout = new QVBoxLayout;
  auto * speed_layout = new QGridLayout;
  auto * target_grid = new QGridLayout;
  speed_layout->setHorizontalSpacing(6);
  speed_layout->setVerticalSpacing(6);
  target_grid->setHorizontalSpacing(6);
  target_grid->setVerticalSpacing(6);

  this->spin_linear_speed_ = new QDoubleSpinBox(group);
  this->spin_linear_speed_->setRange(0.1, 200.0);
  this->spin_linear_speed_->setSingleStep(1.0);
  this->spin_linear_speed_->setDecimals(1);
  this->spin_linear_speed_->setSuffix(" mm/s");
  this->spin_linear_speed_->setValue(kServoDefaultLinearSpeedMillimetersPerSecond);

  this->spin_angular_speed_ = new QDoubleSpinBox(group);
  this->spin_angular_speed_->setRange(0.1, 180.0);
  this->spin_angular_speed_->setSingleStep(1.0);
  this->spin_angular_speed_->setDecimals(1);
  this->spin_angular_speed_->setSuffix(" deg/s");
  this->spin_angular_speed_->setValue(kServoDefaultAngularSpeedDegreesPerSecond);

  speed_layout->addWidget(new QLabel("Linear", group), 0, 0);
  speed_layout->addWidget(this->spin_linear_speed_, 0, 1);
  speed_layout->addWidget(new QLabel("Angular", group), 1, 0);
  speed_layout->addWidget(this->spin_angular_speed_, 1, 1);
  layout->addLayout(speed_layout);

  auto create_target_cell =
    [this, group](
    const QString & axis_label, const size_t axis_index) {
      auto * cell = new QWidget(group);
      auto * cell_layout = new QHBoxLayout(cell);
      cell_layout->setContentsMargins(0, 0, 0, 0);
      cell_layout->setSpacing(4);

      auto * label = new QLabel(axis_label, cell);
      label->setMinimumWidth(24);
      label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
      cell_layout->addWidget(label);

      auto * positive_button = new QPushButton("+", cell);
      positive_button->setFixedWidth(kTargetButtonWidth);
      this->connectTargetButton(positive_button, axis_index, 1);
      cell_layout->addWidget(positive_button);

      auto * negative_button = new QPushButton("-", cell);
      negative_button->setFixedWidth(kTargetButtonWidth);
      this->connectTargetButton(negative_button, axis_index, -1);
      cell_layout->addWidget(negative_button);

      cell_layout->addStretch();
      return cell;
    };

  target_grid->addWidget(create_target_cell("X", 0), 0, 0);
  target_grid->addWidget(create_target_cell("Y", 1), 0, 1);
  target_grid->addWidget(create_target_cell("Z", 2), 1, 0);
  target_grid->addWidget(create_target_cell("Rx", 3), 1, 1);
  layout->addLayout(target_grid);

  this->label_servo_target_ = new QLabel("Target: -", group);
  this->label_servo_target_->setWordWrap(true);
  layout->addWidget(this->label_servo_target_);
  layout->addStretch();

  group->setLayout(layout);
  return group;
}

QWidget * Mg400ServoPanel::createJointTargetSection()
{
  auto * group = new QGroupBox("ServoJ Target", this);
  auto * layout = new QVBoxLayout;
  auto * speed_layout = new QGridLayout;
  auto * target_grid = new QGridLayout;
  speed_layout->setHorizontalSpacing(6);
  speed_layout->setVerticalSpacing(6);
  target_grid->setHorizontalSpacing(6);
  target_grid->setVerticalSpacing(6);

  this->spin_joint_speed_ = new QDoubleSpinBox(group);
  this->spin_joint_speed_->setRange(0.1, 180.0);
  this->spin_joint_speed_->setSingleStep(1.0);
  this->spin_joint_speed_->setDecimals(1);
  this->spin_joint_speed_->setSuffix(" deg/s");
  this->spin_joint_speed_->setValue(kServoDefaultJointSpeedDegreesPerSecond);

  speed_layout->addWidget(new QLabel("Joint", group), 0, 0);
  speed_layout->addWidget(this->spin_joint_speed_, 0, 1);
  layout->addLayout(speed_layout);

  auto create_joint_cell =
    [this, group](
    const QString & axis_label, const size_t axis_index) {
      auto * cell = new QWidget(group);
      auto * cell_layout = new QHBoxLayout(cell);
      cell_layout->setContentsMargins(0, 0, 0, 0);
      cell_layout->setSpacing(4);

      auto * label = new QLabel(axis_label, cell);
      label->setMinimumWidth(24);
      label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
      cell_layout->addWidget(label);

      auto * positive_button = new QPushButton("+", cell);
      positive_button->setFixedWidth(kTargetButtonWidth);
      this->connectJointTargetButton(positive_button, axis_index, 1);
      cell_layout->addWidget(positive_button);

      auto * negative_button = new QPushButton("-", cell);
      negative_button->setFixedWidth(kTargetButtonWidth);
      this->connectJointTargetButton(negative_button, axis_index, -1);
      cell_layout->addWidget(negative_button);

      cell_layout->addStretch();
      return cell;
    };

  target_grid->addWidget(create_joint_cell("J1", 0), 0, 0);
  target_grid->addWidget(create_joint_cell("J2", 1), 0, 1);
  target_grid->addWidget(create_joint_cell("J3", 2), 1, 0);
  target_grid->addWidget(create_joint_cell("J4", 3), 1, 1);
  layout->addLayout(target_grid);

  this->label_servo_joint_target_ = new QLabel("Joint Target: -", group);
  this->label_servo_joint_target_->setWordWrap(true);
  layout->addWidget(this->label_servo_joint_target_);
  layout->addStretch();

  group->setLayout(layout);
  return group;
}

void Mg400ServoPanel::onRosNodeInitialized()
{
  this->setupRosInterfaces();
}

void Mg400ServoPanel::onNamespaceChanged()
{
  this->setupRosInterfaces();
}

void Mg400ServoPanel::setupRosInterfaces()
{
  if (!this->rawNode()) {
    return;
  }

  {
    const std::lock_guard<std::mutex> lock(this->robot_mode_mutex_);
    this->current_robot_mode_ = mg400_msgs::msg::RobotMode::INIT;
  }
  {
    const std::lock_guard<std::mutex> lock(this->actual_state_mutex_);
    this->current_pose_ = geometry_msgs::msg::Pose();
    this->current_pose_.orientation.w = 1.0;
    this->current_joint_state_ = {0.0, 0.0, 0.0, 0.0};
    this->has_actual_state_ = false;
  }
  {
    const std::lock_guard<std::mutex> lock(this->servo_state_mutex_);
    this->servo_mode_enabled_ = false;
    this->servo_target_initialized_ = false;
    this->servo_target_pose_ = geometry_msgs::msg::Pose();
    this->servo_target_pose_.orientation.w = 1.0;
    this->servo_target_directions_ = {0, 0, 0, 0};
    this->servo_joint_target_initialized_ = false;
    this->servo_joint_target_ = {0.0, 0.0, 0.0, 0.0};
    this->servo_joint_directions_ = {0, 0, 0, 0};
    this->last_servo_target_update_time_ = std::chrono::steady_clock::now();
  }
  {
    const std::lock_guard<std::mutex> lock(this->joy_control_mutex_);
    this->joy_control_active_ = false;
    this->last_joy_control_active_time_ = std::chrono::steady_clock::time_point::min();
  }
  this->updateServoStatus("Inactive");
  this->updateServoTargetLabel();
  this->updateServoJointTargetLabel();

  this->robot_mode_sub_ = this->rawNode()->create_subscription<RobotMode>(
    this->makeTopic("robot_mode"), rclcpp::SensorDataQoS().keep_last(1),
    [this](const RobotMode::ConstSharedPtr msg) {
      const std::lock_guard<std::mutex> lock(this->robot_mode_mutex_);
      this->current_robot_mode_ = msg->robot_mode;
    });

  this->realtime_feedback_sub_ = this->rawNode()->create_subscription<RealtimeFeedback>(
    this->makeTopic("realtime_feedback"), rclcpp::SensorDataQoS().keep_last(1),
    [this](const RealtimeFeedback::ConstSharedPtr msg) {
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

  this->servo_mode_enabled_sub_ = this->rawNode()->create_subscription<std_msgs::msg::Bool>(
    this->makeTopic("servo_mode_enabled"),
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
      bool should_initialize = false;
      {
        const std::lock_guard<std::mutex> lock(this->servo_state_mutex_);
        should_initialize = msg->data && !this->servo_mode_enabled_;
        this->servo_mode_enabled_ = msg->data;
        if (!msg->data) {
          this->servo_target_directions_ = {0, 0, 0, 0};
          this->servo_joint_directions_ = {0, 0, 0, 0};
        }
      }
      if (should_initialize) {
        if (this->initializeSelectedServoTarget()) {
          this->updateServoStatus("Active");
        } else {
          this->updateServoStatus("Active, waiting for realtime feedback");
        }
      } else if (!msg->data) {
        this->updateServoStatus("Inactive");
      }
    });

  this->joy_control_active_sub_ = this->rawNode()->create_subscription<std_msgs::msg::Bool>(
    this->makeTopic("joy_control_active"),
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
      const std::lock_guard<std::mutex> lock(this->joy_control_mutex_);
      this->joy_control_active_ = msg->data;
      this->last_joy_control_active_time_ = std::chrono::steady_clock::now();
    });

  this->servo_p_pub_ = this->rawNode()->create_publisher<ServoP>(
    this->makeTopic("servo_p"),
    rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
  this->servo_j_pub_ = this->rawNode()->create_publisher<ServoJ>(
    this->makeTopic("servo_j"),
    rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());

  this->servo_mode_clnt_ =
    this->rawNode()->create_client<ServoModeService>(
    this->makeService("servo_mode"), rmw_qos_profile_default, this->callbackGroup());
}

void Mg400ServoPanel::tick()
{
  RobotMode::_robot_mode_type robot_mode;
  {
    const std::lock_guard<std::mutex> lock(this->robot_mode_mutex_);
    robot_mode = this->current_robot_mode_;
  }

  bool servo_mode_enabled = false;
  ServoControlType servo_control_type = ServoControlType::SERVO_P;
  {
    const std::lock_guard<std::mutex> lock(this->servo_state_mutex_);
    servo_mode_enabled = this->servo_mode_enabled_;
    servo_control_type = this->servo_control_type_;
  }
  const bool joy_control_active = this->joyControlIsActive();

  const auto robot_enabled = robot_mode == RobotMode::ENABLE;
  const auto servo_p_selected = servo_control_type == ServoControlType::SERVO_P;
  const auto servo_j_selected = servo_control_type == ServoControlType::SERVO_J;
  this->label_robot_mode_->setText(robotModeToString(robot_mode));
  if (robot_enabled) {
    this->label_robot_mode_->setStyleSheet("color: green;");
  } else if (robot_mode == RobotMode::DISABLED) {
    this->label_robot_mode_->setStyleSheet("color: blue;");
  } else {
    this->label_robot_mode_->setStyleSheet("");
  }

  this->button_servo_mode_->setEnabled(
    !joy_control_active &&
    static_cast<bool>(this->rawNode()) &&
    static_cast<bool>(this->servo_mode_clnt_) &&
    (robot_enabled || servo_mode_enabled));
  this->button_servo_mode_->setText(
    joy_control_active ? "mg400_joy active" :
    (servo_mode_enabled ? "Exit ServoMode" : "Enter ServoMode"));
  this->radio_servo_p_->setEnabled(!joy_control_active && !servo_mode_enabled);
  this->radio_servo_j_->setEnabled(!joy_control_active && !servo_mode_enabled);

  for (auto * button : this->target_buttons_) {
    if (button) {
      button->setEnabled(
        !joy_control_active && servo_mode_enabled && servo_p_selected &&
        static_cast<bool>(this->servo_p_pub_));
    }
  }
  for (auto * button : this->servo_j_buttons_) {
    if (button) {
      button->setEnabled(
        !joy_control_active && servo_mode_enabled && servo_j_selected &&
        static_cast<bool>(this->servo_j_pub_));
    }
  }
  this->spin_linear_speed_->setEnabled(!joy_control_active && !servo_mode_enabled);
  this->spin_angular_speed_->setEnabled(!joy_control_active && !servo_mode_enabled);
  this->spin_joint_speed_->setEnabled(!joy_control_active && !servo_mode_enabled);
  if (joy_control_active) {
    this->label_servo_status_->setText(kJoyControlActiveStatus);
  } else if (this->label_servo_status_->text() == kJoyControlActiveStatus) {
    this->label_servo_status_->setText(servo_mode_enabled ? "Active" : "Inactive");
  }
  this->updateJoyControlOverlay(joy_control_active);
  this->updateServoTargetLabel();
  this->updateServoJointTargetLabel();
}

void Mg400ServoPanel::callbackServoMode()
{
  bool enable = true;
  {
    const std::lock_guard<std::mutex> lock(this->servo_state_mutex_);
    enable = !this->servo_mode_enabled_;
  }
  this->requestServoMode(enable);
}

void Mg400ServoPanel::onServoControlTimer()
{
  this->publishServoTarget();
}

void Mg400ServoPanel::connectTargetButton(
  QPushButton * button, const size_t axis_index, const int direction)
{
  if (!button) {
    return;
  }

  this->target_buttons_.push_back(button);
  connect(
    button, &QPushButton::pressed, this, [this, axis_index, direction]() {
      this->setServoTargetDirection(axis_index, direction);
    });
  connect(
    button, &QPushButton::released, this, [this, axis_index]() {
      this->setServoTargetDirection(axis_index, 0);
    });
}

void Mg400ServoPanel::connectJointTargetButton(
  QPushButton * button, const size_t axis_index, const int direction)
{
  if (!button) {
    return;
  }

  this->servo_j_buttons_.push_back(button);
  connect(
    button, &QPushButton::pressed, this, [this, axis_index, direction]() {
      this->setServoJointDirection(axis_index, direction);
    });
  connect(
    button, &QPushButton::released, this, [this, axis_index]() {
      this->setServoJointDirection(axis_index, 0);
    });
}

void Mg400ServoPanel::setServoControlType(const ServoControlType servo_control_type)
{
  const std::lock_guard<std::mutex> lock(this->servo_state_mutex_);
  if (this->servo_mode_enabled_) {
    return;
  }
  this->servo_control_type_ = servo_control_type;
}

void Mg400ServoPanel::requestServoMode(const bool enable)
{
  if (!this->rawNode() || !this->servo_mode_clnt_) {
    return;
  }
  if (this->joyControlIsActive()) {
    this->updateServoStatus(kJoyControlActiveStatus);
    return;
  }

  if (enable) {
    bool has_actual_state = false;
    {
      const std::lock_guard<std::mutex> lock(this->actual_state_mutex_);
      has_actual_state = this->has_actual_state_;
    }
    if (!has_actual_state) {
      this->updateServoStatus("No realtime feedback");
      return;
    }
  }

  if (!this->servo_mode_clnt_->wait_for_service(std::chrono::milliseconds(kServoServiceTimeoutMs)))
  {
    this->updateServoStatus("ServoMode service is not ready");
    RCLCPP_ERROR(
      this->rawNode()->get_logger(), "\"%s\" service is not ready",
      this->servo_mode_clnt_->get_service_name());
    return;
  }

  auto request = std::make_shared<ServoModeService::Request>();
  request->enable = enable;
  this->updateServoStatus(enable ? "Entering ServoMode" : "Exiting ServoMode");

  auto future_result = this->servo_mode_clnt_->async_send_request(request);
  const auto future_status = this->callbackGroupExecutor().spin_until_future_complete(
    future_result, std::chrono::milliseconds(kServoServiceTimeoutMs));
  if (future_status != rclcpp::FutureReturnCode::SUCCESS) {
    this->servo_mode_clnt_->remove_pending_request(future_result);
    this->updateServoStatus(enable ? "ServoMode enter timed out" : "ServoMode exit timed out");
    RCLCPP_WARN(
      this->rawNode()->get_logger(), "\"%s\" service client: response timeout",
      this->servo_mode_clnt_->get_service_name());
    return;
  }

  const auto response = future_result.get();
  if (response->error_id != 0) {
    this->updateServoStatus(
      QString("ServoMode failed: error_id=%1 %2")
      .arg(response->error_id)
      .arg(QString::fromStdString(response->message)));
    RCLCPP_ERROR(
      this->rawNode()->get_logger(), "\"%s\" service client: failed, error_id=%d, message=%s",
      this->servo_mode_clnt_->get_service_name(), response->error_id,
      response->message.c_str());
    return;
  }

  {
    const std::lock_guard<std::mutex> lock(this->servo_state_mutex_);
    this->servo_mode_enabled_ = enable;
    if (!enable) {
      this->servo_target_directions_ = {0, 0, 0, 0};
      this->servo_joint_directions_ = {0, 0, 0, 0};
    }
  }

  if (enable) {
    if (!this->initializeSelectedServoTarget()) {
      this->updateServoStatus("ServoMode active but realtime feedback is unavailable");
      return;
    }
    this->updateServoStatus("Active");
    this->publishServoTarget();
  } else {
    this->updateServoStatus("Inactive");
  }
}

bool Mg400ServoPanel::initializeServoTargetFromCurrentPose()
{
  geometry_msgs::msg::Pose pose;
  {
    const std::lock_guard<std::mutex> lock(this->actual_state_mutex_);
    if (!this->has_actual_state_) {
      return false;
    }
    pose = this->current_pose_;
  }

  {
    const std::lock_guard<std::mutex> lock(this->servo_state_mutex_);
    this->servo_target_pose_ = pose;
    this->servo_target_initialized_ = true;
    this->servo_target_directions_ = {0, 0, 0, 0};
    this->last_servo_target_update_time_ = std::chrono::steady_clock::now();
  }
  this->updateServoTargetLabel();
  return true;
}

bool Mg400ServoPanel::initializeServoJointTargetFromCurrentJointState()
{
  std::array<double, 4> joint_state;
  {
    const std::lock_guard<std::mutex> lock(this->actual_state_mutex_);
    if (!this->has_actual_state_) {
      return false;
    }
    joint_state = this->current_joint_state_;
  }

  {
    const std::lock_guard<std::mutex> lock(this->servo_state_mutex_);
    this->servo_joint_target_ = joint_state;
    this->servo_joint_target_initialized_ = true;
    this->servo_joint_directions_ = {0, 0, 0, 0};
    this->last_servo_target_update_time_ = std::chrono::steady_clock::now();
  }
  this->updateServoJointTargetLabel();
  return true;
}

bool Mg400ServoPanel::initializeSelectedServoTarget()
{
  ServoControlType servo_control_type = ServoControlType::SERVO_P;
  {
    const std::lock_guard<std::mutex> lock(this->servo_state_mutex_);
    servo_control_type = this->servo_control_type_;
  }

  if (servo_control_type == ServoControlType::SERVO_J) {
    return this->initializeServoJointTargetFromCurrentJointState();
  }
  return this->initializeServoTargetFromCurrentPose();
}

void Mg400ServoPanel::setServoTargetDirection(
  const size_t axis_index, const int direction)
{
  if (axis_index >= this->servo_target_directions_.size()) {
    return;
  }

  const std::lock_guard<std::mutex> lock(this->servo_state_mutex_);
  this->servo_target_directions_[axis_index] = direction;
  this->last_servo_target_update_time_ = std::chrono::steady_clock::now();
}

void Mg400ServoPanel::setServoJointDirection(
  const size_t axis_index, const int direction)
{
  if (axis_index >= this->servo_joint_directions_.size()) {
    return;
  }

  const std::lock_guard<std::mutex> lock(this->servo_state_mutex_);
  this->servo_joint_directions_[axis_index] = direction;
  this->last_servo_target_update_time_ = std::chrono::steady_clock::now();
}

void Mg400ServoPanel::updateServoTargetPose()
{
  const auto now = std::chrono::steady_clock::now();
  const double linear_speed =
    this->spin_linear_speed_->value() * kMillimetersToMeters;
  const double angular_speed =
    this->spin_angular_speed_->value() * kDegreesToRadians;

  const std::lock_guard<std::mutex> lock(this->servo_state_mutex_);
  if (!this->servo_mode_enabled_ || !this->servo_target_initialized_) {
    this->last_servo_target_update_time_ = now;
    return;
  }

  const std::chrono::duration<double> elapsed = now - this->last_servo_target_update_time_;
  this->last_servo_target_update_time_ = now;
  const double dt = elapsed.count();
  if (dt <= 0.0) {
    return;
  }

  this->servo_target_pose_.position.x +=
    static_cast<double>(this->servo_target_directions_[0]) * linear_speed * dt;
  this->servo_target_pose_.position.y +=
    static_cast<double>(this->servo_target_directions_[1]) * linear_speed * dt;
  this->servo_target_pose_.position.z +=
    static_cast<double>(this->servo_target_directions_[2]) * linear_speed * dt;

  if (this->servo_target_directions_[3] != 0) {
    tf2::Quaternion q(
      this->servo_target_pose_.orientation.x,
      this->servo_target_pose_.orientation.y,
      this->servo_target_pose_.orientation.z,
      this->servo_target_pose_.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw += static_cast<double>(this->servo_target_directions_[3]) * angular_speed * dt;
    q.setRPY(roll, pitch, yaw);
    this->servo_target_pose_.orientation.x = q.getX();
    this->servo_target_pose_.orientation.y = q.getY();
    this->servo_target_pose_.orientation.z = q.getZ();
    this->servo_target_pose_.orientation.w = q.getW();
  }
}

void Mg400ServoPanel::updateServoJointTarget()
{
  const auto now = std::chrono::steady_clock::now();
  const double joint_speed =
    this->spin_joint_speed_->value() * kDegreesToRadians;

  const std::lock_guard<std::mutex> lock(this->servo_state_mutex_);
  if (!this->servo_mode_enabled_ || !this->servo_joint_target_initialized_) {
    this->last_servo_target_update_time_ = now;
    return;
  }

  const std::chrono::duration<double> elapsed = now - this->last_servo_target_update_time_;
  this->last_servo_target_update_time_ = now;
  const double dt = elapsed.count();
  if (dt <= 0.0) {
    return;
  }

  for (size_t i = 0; i < this->servo_joint_target_.size(); ++i) {
    this->servo_joint_target_[i] +=
      static_cast<double>(this->servo_joint_directions_[i]) * joint_speed * dt;
  }
}

void Mg400ServoPanel::publishServoTarget()
{
  if (!this->rawNode()) {
    return;
  }
  if (this->joyControlIsActive()) {
    return;
  }

  ServoControlType servo_control_type = ServoControlType::SERVO_P;
  {
    const std::lock_guard<std::mutex> lock(this->servo_state_mutex_);
    servo_control_type = this->servo_control_type_;
  }

  if (servo_control_type == ServoControlType::SERVO_J) {
    this->publishServoJointTarget();
  } else {
    this->publishServoPoseTarget();
  }
}

void Mg400ServoPanel::publishServoPoseTarget()
{
  if (!this->rawNode() || !this->servo_p_pub_) {
    return;
  }

  bool should_initialize = false;
  {
    const std::lock_guard<std::mutex> lock(this->servo_state_mutex_);
    if (!this->servo_mode_enabled_) {
      return;
    }
    should_initialize = !this->servo_target_initialized_;
  }

  if (should_initialize && !this->initializeServoTargetFromCurrentPose()) {
    return;
  }

  this->updateServoTargetPose();

  geometry_msgs::msg::Pose target_pose;
  {
    const std::lock_guard<std::mutex> lock(this->servo_state_mutex_);
    if (!this->servo_mode_enabled_ || !this->servo_target_initialized_) {
      return;
    }
    target_pose = this->servo_target_pose_;
  }

  ServoP msg;
  msg.pose.header.frame_id = "mg400_origin_link";
  msg.pose.header.stamp = this->rawNode()->get_clock()->now();
  msg.pose.pose = target_pose;
  msg.t = kUnusedServoOption;
  msg.aheadtime = kUnusedServoOption;
  msg.gain = kUnusedServoOption;
  this->servo_p_pub_->publish(msg);
}

void Mg400ServoPanel::publishServoJointTarget()
{
  if (!this->rawNode() || !this->servo_j_pub_) {
    return;
  }

  bool should_initialize = false;
  {
    const std::lock_guard<std::mutex> lock(this->servo_state_mutex_);
    if (!this->servo_mode_enabled_) {
      return;
    }
    should_initialize = !this->servo_joint_target_initialized_;
  }

  if (should_initialize && !this->initializeServoJointTargetFromCurrentJointState()) {
    return;
  }

  this->updateServoJointTarget();

  std::array<double, 4> target_joint_angles;
  {
    const std::lock_guard<std::mutex> lock(this->servo_state_mutex_);
    if (!this->servo_mode_enabled_ || !this->servo_joint_target_initialized_) {
      return;
    }
    target_joint_angles = this->servo_joint_target_;
  }

  ServoJ msg;
  msg.joint_angles = target_joint_angles;
  msg.t = kUnusedServoOption;
  msg.aheadtime = kUnusedServoOption;
  msg.gain = kUnusedServoOption;
  this->servo_j_pub_->publish(msg);
}

bool Mg400ServoPanel::joyControlIsActive()
{
  const std::lock_guard<std::mutex> lock(this->joy_control_mutex_);
  if (!this->joy_control_active_) {
    return false;
  }

  const std::chrono::duration<double> elapsed =
    std::chrono::steady_clock::now() - this->last_joy_control_active_time_;
  return elapsed.count() <= kJoyControlActiveTimeoutSec;
}

void Mg400ServoPanel::updateJoyControlOverlay(const bool active)
{
  if (!this->joy_control_overlay_) {
    return;
  }

  this->joy_control_overlay_->setGeometry(this->rect());
  this->joy_control_overlay_->setVisible(active);
  if (active) {
    this->joy_control_overlay_->raise();
  }
}

void Mg400ServoPanel::updateServoStatus(const QString & text)
{
  if (!this->label_servo_status_) {
    return;
  }
  QMetaObject::invokeMethod(
    this->label_servo_status_, "setText", Qt::QueuedConnection, Q_ARG(QString, text));
}

void Mg400ServoPanel::updateServoTargetLabel()
{
  if (!this->label_servo_target_) {
    return;
  }

  geometry_msgs::msg::Pose target_pose;
  bool initialized = false;
  {
    const std::lock_guard<std::mutex> lock(this->servo_state_mutex_);
    initialized = this->servo_target_initialized_;
    target_pose = this->servo_target_pose_;
  }

  QString text = "Target: -";
  if (initialized) {
    tf2::Quaternion q(
      target_pose.orientation.x,
      target_pose.orientation.y,
      target_pose.orientation.z,
      target_pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    text = QString("Target: X=%1 mm, Y=%2 mm, Z=%3 mm, Rx=%4 deg")
      .arg(target_pose.position.x * kMetersToMillimeters, 0, 'f', 1)
      .arg(target_pose.position.y * kMetersToMillimeters, 0, 'f', 1)
      .arg(target_pose.position.z * kMetersToMillimeters, 0, 'f', 1)
      .arg(yaw * kRadiansToDegrees, 0, 'f', 1);
  }

  QMetaObject::invokeMethod(
    this->label_servo_target_, "setText", Qt::QueuedConnection, Q_ARG(QString, text));
}

void Mg400ServoPanel::updateServoJointTargetLabel()
{
  if (!this->label_servo_joint_target_) {
    return;
  }

  std::array<double, 4> target_joint_angles;
  bool initialized = false;
  {
    const std::lock_guard<std::mutex> lock(this->servo_state_mutex_);
    initialized = this->servo_joint_target_initialized_;
    target_joint_angles = this->servo_joint_target_;
  }

  QString text = "Joint Target: -";
  if (initialized) {
    text = QString("Joint Target: J1=%1 deg, J2=%2 deg, J3=%3 deg, J4=%4 deg")
      .arg(target_joint_angles[0] * kRadiansToDegrees, 0, 'f', 1)
      .arg(target_joint_angles[1] * kRadiansToDegrees, 0, 'f', 1)
      .arg(target_joint_angles[2] * kRadiansToDegrees, 0, 'f', 1)
      .arg(target_joint_angles[3] * kRadiansToDegrees, 0, 'f', 1);
  }

  QMetaObject::invokeMethod(
    this->label_servo_joint_target_, "setText", Qt::QueuedConnection, Q_ARG(QString, text));
}
}  // namespace mg400_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mg400_rviz_plugin::Mg400ServoPanel, rviz_common::Panel)
