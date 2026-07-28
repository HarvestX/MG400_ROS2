// Copyright 2026 Yuki Yamamoto
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

#include "mg400_operation_gui/main_window.hpp"

#include <QCloseEvent>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QStackedWidget>
#include <QTime>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <functional>
#include <utility>

namespace mg400_operation_gui
{
namespace
{

constexpr double PI = 3.14159265358979323846;
constexpr double SLIDER_SCALE = 10.0;
constexpr double DEFAULT_FILTER_CUTOFF_HZ = 1.0;

QString serviceError(const QString & name, const int error_id)
{
  return QStringLiteral("%1 に失敗しました (error_id=%2)").arg(name).arg(error_id);
}

}  // namespace

MainWindow::MainWindow(const rclcpp::NodeOptions & options)
: QMainWindow(),
  rclcpp::Node("mg400_operation_gui", "mg400", options),
  servo_p_frame_id_(this->declare_parameter<std::string>(
      "servo_p_frame_id", "mg400_origin_link"))
{
  const auto command_period_ms = std::clamp<std::int64_t>(
    this->declare_parameter<std::int64_t>("command_period_ms", 30), 10, 90);

  this->setupUi();
  this->createRosInterfaces(std::chrono::milliseconds(command_period_ms));
  this->appendLog(
    QStringLiteral("起動しました。ROS namespace: %1")
    .arg(QString::fromStdString(this->get_namespace())));
  this->appendLog(
    QStringLiteral("ServoP target frame: %1 / 指令周期: %2 ms")
    .arg(QString::fromStdString(this->servo_p_frame_id_))
    .arg(command_period_ms));
  this->updateUiState();
}

void MainWindow::setupUi()
{
  this->setWindowTitle(QStringLiteral("MG400 Servo Control"));
  this->setMinimumSize(720, 720);

  auto * central = new QWidget(this);
  auto * root = new QVBoxLayout(central);
  root->setSpacing(10);

  auto * status_group = new QGroupBox(QStringLiteral("MG400 状態"), central);
  auto * status_layout = new QGridLayout(status_group);
  status_layout->addWidget(new QLabel(QStringLiteral("RobotMode"), status_group), 0, 0);
  status_layout->addWidget(new QLabel(QStringLiteral("ControlState"), status_group), 0, 2);
  status_layout->addWidget(new QLabel(QStringLiteral("Lease ID"), status_group), 1, 0);
  status_layout->addWidget(new QLabel(QStringLiteral("Target送信"), status_group), 1, 2);
  this->robot_mode_value_ = new QLabel(QStringLiteral("未受信"), status_group);
  this->control_state_value_ = new QLabel(QStringLiteral("未受信"), status_group);
  this->lease_value_ = new QLabel(QStringLiteral("0"), status_group);
  this->command_value_ = new QLabel(QStringLiteral("OFF"), status_group);
  status_layout->addWidget(this->robot_mode_value_, 0, 1);
  status_layout->addWidget(this->control_state_value_, 0, 3);
  status_layout->addWidget(this->lease_value_, 1, 1);
  status_layout->addWidget(this->command_value_, 1, 3);
  root->addWidget(status_group);

  auto * current_value_group = new QGroupBox(
    QStringLiteral("現在値 (realtime_feedback)"), central);
  auto * current_value_layout = new QGridLayout(current_value_group);
  const std::array<QString, 4> joint_labels{{
    QStringLiteral("J1"), QStringLiteral("J2"),
    QStringLiteral("J3"), QStringLiteral("J4")}};
  const std::array<QString, 4> pose_labels{{
    QStringLiteral("X"), QStringLiteral("Y"),
    QStringLiteral("Z"), QStringLiteral("Rx")}};
  for (std::size_t index = 0; index < joint_labels.size(); ++index) {
    const int column = static_cast<int>(index * 2);
    current_value_layout->addWidget(
      new QLabel(joint_labels.at(index), current_value_group), 0, column);
    this->current_joint_value_labels_.at(index) =
      new QLabel(QStringLiteral("-- deg"), current_value_group);
    this->current_joint_value_labels_.at(index)->setAlignment(
      Qt::AlignRight | Qt::AlignVCenter);
    current_value_layout->addWidget(
      this->current_joint_value_labels_.at(index), 0, column + 1);

    current_value_layout->addWidget(
      new QLabel(pose_labels.at(index), current_value_group), 1, column);
    this->current_pose_value_labels_.at(index) = new QLabel(
      index < 3 ? QStringLiteral("-- mm") : QStringLiteral("-- deg"),
      current_value_group);
    this->current_pose_value_labels_.at(index)->setAlignment(
      Qt::AlignRight | Qt::AlignVCenter);
    current_value_layout->addWidget(
      this->current_pose_value_labels_.at(index), 1, column + 1);
    current_value_layout->setColumnStretch(column + 1, 1);
  }
  root->addWidget(current_value_group);

  auto * robot_group = new QGroupBox(QStringLiteral("ロボット操作"), central);
  auto * robot_layout = new QHBoxLayout(robot_group);
  this->enable_button_ = new QPushButton(QStringLiteral("Enable"), robot_group);
  this->disable_button_ = new QPushButton(QStringLiteral("Disable"), robot_group);
  this->clear_error_button_ = new QPushButton(QStringLiteral("ClearError"), robot_group);
  robot_layout->addWidget(this->enable_button_);
  robot_layout->addWidget(this->disable_button_);
  robot_layout->addWidget(this->clear_error_button_);
  root->addWidget(robot_group);

  auto * servo_group = new QGroupBox(QStringLiteral("ServoJ / ServoP"), central);
  auto * servo_layout = new QVBoxLayout(servo_group);
  auto * mode_layout = new QHBoxLayout();
  mode_layout->addWidget(new QLabel(QStringLiteral("Servoモード:"), servo_group));
  this->servo_j_radio_ = new QRadioButton(QStringLiteral("ServoJ"), servo_group);
  this->servo_p_radio_ = new QRadioButton(QStringLiteral("ServoP"), servo_group);
  this->servo_j_radio_->setChecked(true);
  mode_layout->addWidget(this->servo_j_radio_);
  mode_layout->addWidget(this->servo_p_radio_);
  mode_layout->addStretch();
  servo_layout->addLayout(mode_layout);

  this->target_stack_ = new QStackedWidget(servo_group);
  this->target_stack_->addWidget(
    this->createAxisPanel(
      {QStringLiteral("J1"), QStringLiteral("J2"), QStringLiteral("J3"),
        QStringLiteral("J4")},
      {-160.0, -25.0, -25.0, -160.0},
      {160.0, 85.0, 105.0, 160.0},
      {QStringLiteral(" deg"), QStringLiteral(" deg"), QStringLiteral(" deg"),
        QStringLiteral(" deg")},
      this->servo_j_axes_));
  this->target_stack_->addWidget(
    this->createAxisPanel(
      {QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Z"),
        QStringLiteral("Yaw (R)")},
      {-500.0, -500.0, -500.0, -180.0},
      {500.0, 500.0, 500.0, 180.0},
      {QStringLiteral(" mm"), QStringLiteral(" mm"), QStringLiteral(" mm"),
        QStringLiteral(" deg")},
      this->servo_p_axes_));
  servo_layout->addWidget(this->target_stack_);

  auto * target_action_layout = new QHBoxLayout();
  this->load_target_button_ = new QPushButton(
    QStringLiteral("現在値をスライダーへ読込"), servo_group);
  this->target_help_ = new QLabel(
    QStringLiteral("安全のため、現在値の読込後にスライダーを操作できます。"), servo_group);
  this->target_help_->setWordWrap(true);
  target_action_layout->addWidget(this->load_target_button_);
  target_action_layout->addWidget(this->target_help_, 1);
  servo_layout->addLayout(target_action_layout);

  auto * filter_layout = new QHBoxLayout();
  filter_layout->addWidget(new QLabel(QStringLiteral("Target LPF cutoff:"), servo_group));
  this->filter_cutoff_spin_box_ = new QDoubleSpinBox(servo_group);
  this->filter_cutoff_spin_box_->setRange(0.1, 10.0);
  this->filter_cutoff_spin_box_->setDecimals(1);
  this->filter_cutoff_spin_box_->setSingleStep(0.1);
  this->filter_cutoff_spin_box_->setValue(DEFAULT_FILTER_CUTOFF_HZ);
  this->filter_cutoff_spin_box_->setSuffix(QStringLiteral(" Hz"));
  this->filter_cutoff_spin_box_->setToolTip(
    QStringLiteral("小さいほど滑らかに、大きいほど素早くtargetへ追従します。"));
  filter_layout->addWidget(this->filter_cutoff_spin_box_);
  filter_layout->addStretch();
  servo_layout->addLayout(filter_layout);

  auto * mode_command_layout = new QHBoxLayout();
  this->mode_start_button_ = new QPushButton(
    QStringLiteral("Servoモード開始（lease取得）"), servo_group);
  this->mode_stop_button_ = new QPushButton(
    QStringLiteral("Servoモード終了（IDLE）"), servo_group);
  this->mode_start_button_->setMinimumHeight(48);
  this->mode_stop_button_->setMinimumHeight(48);
  mode_command_layout->addWidget(this->mode_start_button_);
  mode_command_layout->addWidget(this->mode_stop_button_);
  servo_layout->addLayout(mode_command_layout);

  auto * publish_command_layout = new QHBoxLayout();
  this->publish_start_button_ = new QPushButton(
    QStringLiteral("Target送信開始"), servo_group);
  this->publish_stop_button_ = new QPushButton(
    QStringLiteral("Target送信停止"), servo_group);
  this->publish_start_button_->setMinimumHeight(48);
  this->publish_stop_button_->setMinimumHeight(48);
  this->publish_start_button_->setStyleSheet(
    QStringLiteral(
      "QPushButton:enabled { background: #2e7d32; color: white; "
      "font-weight: bold; }"));
  this->publish_stop_button_->setStyleSheet(
    QStringLiteral(
      "QPushButton:enabled { background: #c62828; color: white; "
      "font-weight: bold; }"));
  publish_command_layout->addWidget(this->publish_start_button_);
  publish_command_layout->addWidget(this->publish_stop_button_);
  servo_layout->addLayout(publish_command_layout);
  root->addWidget(servo_group, 1);

  auto * log_group = new QGroupBox(QStringLiteral("操作ログ"), central);
  auto * log_layout = new QVBoxLayout(log_group);
  this->log_view_ = new QPlainTextEdit(log_group);
  this->log_view_->setReadOnly(true);
  this->log_view_->setMaximumBlockCount(300);
  log_layout->addWidget(this->log_view_);
  root->addWidget(log_group, 1);

  this->setCentralWidget(central);

  this->close_timeout_ = new QTimer(this);
  this->close_timeout_->setSingleShot(true);
  QObject::connect(
    this->close_timeout_, &QTimer::timeout, this, [this]() {
      this->appendLog(
        QStringLiteral(
          "Servo停止応答を待たずに終了します。"
          "watchdogによる安全停止を確認してください。"),
        true);
      this->finishClose();
    });

  QObject::connect(
    this->enable_button_, &QPushButton::clicked, this, [this]() {
      this->callEnableRobot();
    });
  QObject::connect(
    this->disable_button_, &QPushButton::clicked, this, [this]() {
      this->callDisableRobot();
    });
  QObject::connect(
    this->clear_error_button_, &QPushButton::clicked, this, [this]() {
      this->callClearError();
    });
  QObject::connect(
    this->load_target_button_, &QPushButton::clicked, this, [this]() {
      this->pending_servo_start_ = false;
      this->loadCurrentTarget();
    });
  QObject::connect(
    this->mode_start_button_, &QPushButton::clicked, this, [this]() {
      this->beginServoStart();
    });
  QObject::connect(
    this->mode_stop_button_, &QPushButton::clicked, this, [this]() {
      this->requestServoStop();
    });
  QObject::connect(
    this->publish_start_button_, &QPushButton::clicked, this, [this]() {
      this->startPublishing();
    });
  QObject::connect(
    this->publish_stop_button_, &QPushButton::clicked, this, [this]() {
      this->stopPublishing();
    });
  QObject::connect(
    this->servo_j_radio_, &QRadioButton::toggled, this, [this](bool checked) {
      if (checked) {
        this->target_stack_->setCurrentIndex(0);
        this->updateUiState();
      }
    });
  QObject::connect(
    this->servo_p_radio_, &QRadioButton::toggled, this, [this](bool checked) {
      if (checked) {
        this->target_stack_->setCurrentIndex(1);
        this->updateUiState();
      }
    });
}

QWidget * MainWindow::createAxisPanel(
  const std::array<QString, 4> & labels,
  const std::array<double, 4> & minimums,
  const std::array<double, 4> & maximums,
  const std::array<QString, 4> & suffixes,
  std::array<AxisWidgets, 4> & axes)
{
  auto * panel = new QWidget(this);
  auto * layout = new QGridLayout(panel);
  layout->setColumnStretch(1, 1);

  for (std::size_t index = 0; index < axes.size(); ++index) {
    auto * label = new QLabel(labels.at(index), panel);
    auto * slider = new QSlider(Qt::Horizontal, panel);
    auto * spin_box = new QDoubleSpinBox(panel);
    slider->setRange(
      static_cast<int>(std::lround(minimums.at(index) * SLIDER_SCALE)),
      static_cast<int>(std::lround(maximums.at(index) * SLIDER_SCALE)));
    slider->setSingleStep(1);
    slider->setPageStep(10);
    spin_box->setRange(minimums.at(index), maximums.at(index));
    spin_box->setDecimals(1);
    spin_box->setSingleStep(0.1);
    spin_box->setSuffix(suffixes.at(index));
    spin_box->setMinimumWidth(125);
    slider->setEnabled(false);
    spin_box->setEnabled(false);

    QObject::connect(
      slider, &QSlider::valueChanged, spin_box, [spin_box](const int value) {
        const QSignalBlocker blocker(spin_box);
        spin_box->setValue(static_cast<double>(value) / SLIDER_SCALE);
      });
    QObject::connect(
      spin_box, qOverload<double>(&QDoubleSpinBox::valueChanged), slider,
      [slider](const double value) {
        const QSignalBlocker blocker(slider);
        slider->setValue(static_cast<int>(std::lround(value * SLIDER_SCALE)));
      });

    layout->addWidget(label, static_cast<int>(index), 0);
    layout->addWidget(slider, static_cast<int>(index), 1);
    layout->addWidget(spin_box, static_cast<int>(index), 2);
    axes.at(index) = AxisWidgets{slider, spin_box};
  }
  return panel;
}

void MainWindow::createRosInterfaces(const std::chrono::milliseconds command_period)
{
  this->enable_client_ = this->create_client<EnableRobot>("enable_robot");
  this->disable_client_ = this->create_client<DisableRobot>("disable_robot");
  this->clear_error_client_ = this->create_client<ClearError>("clear_error");
  this->control_state_client_ = this->create_client<ChangeControlState>("change_control_state");

  this->servo_j_publisher_ = this->create_publisher<mg400_msgs::msg::ServoJ>(
    "servo_j/target", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  this->servo_p_publisher_ = this->create_publisher<mg400_msgs::msg::ServoP>(
    "servo_p/target", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  this->realtime_feedback_subscription_ =
    this->create_subscription<mg400_msgs::msg::RealtimeFeedback>(
    "realtime_feedback", rclcpp::SensorDataQoS(),
    std::bind(&MainWindow::handleRealtimeFeedback, this, std::placeholders::_1));
  this->robot_mode_subscription_ = this->create_subscription<mg400_msgs::msg::RobotMode>(
    "robot_mode", rclcpp::SensorDataQoS(),
    std::bind(&MainWindow::handleRobotMode, this, std::placeholders::_1));
  const auto control_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  this->control_state_subscription_ =
    this->create_subscription<mg400_msgs::msg::ControlState>(
    "control_state", control_qos,
    std::bind(&MainWindow::handleControlState, this, std::placeholders::_1));

  this->command_timer_ = this->create_wall_timer(
    command_period, [this]() {
      this->publishServoTarget();
    });
}

void MainWindow::updateUiState()
{
  this->robot_mode_value_->setText(
    this->robot_mode_received_ ? robotModeName(this->robot_mode_) : QStringLiteral("未受信"));
  this->control_state_value_->setText(
    this->control_state_received_ ? controlStateName(this->control_state_) :
    QStringLiteral("未受信"));
  this->lease_value_->setText(QString::number(this->lease_id_));
  this->command_value_->setText(
    this->command_active_ ? QStringLiteral("ON") :
    QStringLiteral("OFF"));
  this->command_value_->setStyleSheet(
    this->command_active_ ? QStringLiteral("color: #2e7d32; font-weight: bold;") :
    QStringLiteral("color: #c62828; font-weight: bold;"));

  const bool owns_servo = this->lease_id_ != 0;
  const bool controls_available = !this->service_busy_ && !this->closing_;
  this->enable_button_->setEnabled(controls_available && !owns_servo);
  this->disable_button_->setEnabled(controls_available && !owns_servo);
  this->clear_error_button_->setEnabled(controls_available);
  this->servo_j_radio_->setEnabled(controls_available && !owns_servo);
  this->servo_p_radio_->setEnabled(controls_available && !owns_servo);
  this->load_target_button_->setEnabled(
    controls_available && !owns_servo && this->realtime_feedback_received_);

  const bool ready_for_servo =
    controls_available && !owns_servo && this->robot_mode_received_ &&
    this->control_state_received_ && this->realtime_feedback_received_ &&
    this->robot_mode_ == mg400_msgs::msg::RobotMode::ENABLE &&
    this->control_state_ == mg400_msgs::msg::ControlState::IDLE;
  this->mode_start_button_->setEnabled(ready_for_servo);
  this->mode_stop_button_->setEnabled(controls_available && owns_servo);
  this->publish_start_button_->setEnabled(
    controls_available && owns_servo && !this->command_active_ &&
    this->selectedTargetInitialized());
  this->publish_stop_button_->setEnabled(
    controls_available && owns_servo && this->command_active_);
  this->filter_cutoff_spin_box_->setEnabled(!this->closing_);

  const bool j_selected = this->selectedMode() == ServoMode::SERVO_J;
  this->setAxesEnabled(
    this->servo_j_axes_, j_selected && this->servo_j_target_initialized_ &&
    !this->service_busy_ && !this->closing_);
  this->setAxesEnabled(
    this->servo_p_axes_, !j_selected && this->servo_p_target_initialized_ &&
    !this->service_busy_ && !this->closing_);

  if (this->selectedTargetInitialized()) {
    this->target_help_->setText(
      this->command_active_ ?
      QStringLiteral("指令送信中です。スライダー変更が周期指令へ反映されます。") :
      owns_servo ?
      QStringLiteral("Servoモード中です。Target送信開始後に周期指令を送信します。") :
      QStringLiteral("現在値を基準にスライダーを操作できます。"));
  } else {
    this->target_help_->setText(
      this->realtime_feedback_received_ ?
      QStringLiteral("安全のため、現在値の読込後にスライダーを操作できます。") :
      QStringLiteral("realtime_feedback の受信を待っています。"));
  }
}

void MainWindow::setServiceBusy(const bool busy)
{
  this->service_busy_ = busy;
  this->updateUiState();
}

void MainWindow::setAxesEnabled(std::array<AxisWidgets, 4> & axes, const bool enabled)
{
  for (auto & axis : axes) {
    axis.slider->setEnabled(enabled);
    axis.spin_box->setEnabled(enabled);
  }
}

bool MainWindow::setAxisValues(
  std::array<AxisWidgets, 4> & axes, const std::array<double, 4> & values)
{
  for (std::size_t index = 0; index < axes.size(); ++index) {
    if (!std::isfinite(values.at(index)) ||
      values.at(index) < axes.at(index).spin_box->minimum() ||
      values.at(index) > axes.at(index).spin_box->maximum())
    {
      return false;
    }
  }

  for (std::size_t index = 0; index < axes.size(); ++index) {
    const QSignalBlocker spin_blocker(axes.at(index).spin_box);
    const QSignalBlocker slider_blocker(axes.at(index).slider);
    axes.at(index).spin_box->setValue(values.at(index));
    axes.at(index).slider->setValue(
      static_cast<int>(std::lround(values.at(index) * SLIDER_SCALE)));
  }
  return true;
}

std::array<double, 4> MainWindow::axisValues(
  const std::array<AxisWidgets, 4> & axes) const
{
  return {{
    axes.at(0).spin_box->value(), axes.at(1).spin_box->value(),
    axes.at(2).spin_box->value(), axes.at(3).spin_box->value()}};
}

std::array<double, 4> MainWindow::filteredTargetValues()
{
  const bool servo_j_active = this->active_mode_ == ServoMode::SERVO_J;
  const auto raw_values = this->axisValues(
    servo_j_active ? this->servo_j_axes_ : this->servo_p_axes_);
  auto & filtered_values = servo_j_active ?
    this->servo_j_filtered_target_ : this->servo_p_filtered_target_;
  auto & filter_initialized = servo_j_active ?
    this->servo_j_filter_initialized_ : this->servo_p_filter_initialized_;

  if (!filter_initialized) {
    filtered_values = raw_values;
    filter_initialized = true;
  }

  const auto now = std::chrono::steady_clock::now();
  const double elapsed_seconds = std::max(
    0.0, std::chrono::duration<double>(now - this->filter_updated_at_).count());
  this->filter_updated_at_ = now;
  const double cutoff_hz = this->filter_cutoff_spin_box_->value();
  const double alpha = 1.0 - std::exp(-2.0 * PI * cutoff_hz * elapsed_seconds);
  for (std::size_t index = 0; index < filtered_values.size(); ++index) {
    filtered_values.at(index) += alpha * (raw_values.at(index) - filtered_values.at(index));
  }
  return filtered_values;
}

MainWindow::ServoMode MainWindow::selectedMode() const
{
  return this->servo_p_radio_ && this->servo_p_radio_->isChecked() ?
         ServoMode::SERVO_P : ServoMode::SERVO_J;
}

bool MainWindow::selectedTargetInitialized() const
{
  return this->selectedMode() == ServoMode::SERVO_J ?
         this->servo_j_target_initialized_ : this->servo_p_target_initialized_;
}

void MainWindow::callEnableRobot()
{
  if (!this->enable_client_->service_is_ready()) {
    this->appendLog(QStringLiteral("enable_robot サービスが利用できません。"), true);
    return;
  }
  auto request = std::make_shared<EnableRobot::Request>();
  request->num_of_params = EnableRobot::Request::NO_PARAM;
  this->setServiceBusy(true);
  this->appendLog(QStringLiteral("Enable を要求しました。"));
  this->enable_client_->async_send_request(
    request, [this](rclcpp::Client<EnableRobot>::SharedFuture future) {
      try {
        const auto response = future.get();
        this->appendLog(
          response->result ? QStringLiteral("Enable に成功しました。") :
          serviceError(QStringLiteral("Enable"), response->error_id), !response->result);
      } catch (const std::exception & error) {
        this->appendLog(QStringLiteral("Enable 応答エラー: %1").arg(error.what()), true);
      }
      this->setServiceBusy(false);
      this->continueCloseIfNeeded();
    });
}

void MainWindow::callDisableRobot()
{
  if (this->lease_id_ != 0) {
    this->appendLog(QStringLiteral("先にServoモードを終了してください。"), true);
    return;
  }
  if (!this->disable_client_->service_is_ready()) {
    this->appendLog(QStringLiteral("disable_robot サービスが利用できません。"), true);
    return;
  }
  auto request = std::make_shared<DisableRobot::Request>();
  this->setServiceBusy(true);
  this->appendLog(QStringLiteral("Disable を要求しました。"));
  this->disable_client_->async_send_request(
    request, [this](rclcpp::Client<DisableRobot>::SharedFuture future) {
      try {
        const auto response = future.get();
        this->appendLog(
          response->result ? QStringLiteral("Disable に成功しました。") :
          serviceError(QStringLiteral("Disable"), response->error_id), !response->result);
      } catch (const std::exception & error) {
        this->appendLog(QStringLiteral("Disable 応答エラー: %1").arg(error.what()), true);
      }
      this->setServiceBusy(false);
      this->continueCloseIfNeeded();
    });
}

void MainWindow::callClearError()
{
  if (!this->clear_error_client_->service_is_ready()) {
    this->appendLog(QStringLiteral("clear_error サービスが利用できません。"), true);
    return;
  }
  auto request = std::make_shared<ClearError::Request>();
  this->setServiceBusy(true);
  this->appendLog(QStringLiteral("ClearError を要求しました。"));
  this->clear_error_client_->async_send_request(
    request, [this](rclcpp::Client<ClearError>::SharedFuture future) {
      try {
        const auto response = future.get();
        this->appendLog(
          response->result ? QStringLiteral("ClearError に成功しました。") :
          serviceError(QStringLiteral("ClearError"), response->error_id), !response->result);
      } catch (const std::exception & error) {
        this->appendLog(QStringLiteral("ClearError 応答エラー: %1").arg(error.what()), true);
      }
      this->setServiceBusy(false);
      this->continueCloseIfNeeded();
    });
}

void MainWindow::loadCurrentTarget()
{
  if (!this->realtime_feedback_received_) {
    this->pending_servo_start_ = false;
    this->appendLog(QStringLiteral("realtime_feedback をまだ受信していません。"), true);
    return;
  }

  const bool servo_j_selected = this->selectedMode() == ServoMode::SERVO_J;
  const auto & current_values = servo_j_selected ?
    this->current_joint_angles_ : this->current_pose_;
  const bool loaded = this->setAxisValues(
    servo_j_selected ? this->servo_j_axes_ : this->servo_p_axes_, current_values);

  if (loaded) {
    if (servo_j_selected) {
      this->servo_j_filtered_target_ = current_values;
      this->servo_j_filter_initialized_ = true;
      this->servo_j_target_initialized_ = true;
      this->appendLog(
        QStringLiteral("realtime_feedback の現在関節角をスライダーへ読み込みました。"));
    } else {
      this->servo_p_filtered_target_ = current_values;
      this->servo_p_filter_initialized_ = true;
      this->servo_p_target_initialized_ = true;
      this->appendLog(
        QStringLiteral("realtime_feedback の現在TCP poseをスライダーへ読み込みました。"));
    }
  } else {
    this->appendLog(
      servo_j_selected ?
      QStringLiteral("realtime_feedback の関節角が不正、またはUI範囲外です。") :
      QStringLiteral("realtime_feedback のTCP poseが不正、またはUI範囲外です。"),
      true);
  }

  const bool should_start = loaded && this->pending_servo_start_ && !this->closing_;
  this->pending_servo_start_ = false;
  this->updateUiState();
  if (should_start) {
    this->beginServoStart();
  } else {
    this->continueCloseIfNeeded();
  }
}

void MainWindow::beginServoStart()
{
  if (this->service_busy_ || this->lease_id_ != 0 || this->closing_) {
    return;
  }
  if (!this->selectedTargetInitialized()) {
    this->pending_servo_start_ = true;
    this->appendLog(
      QStringLiteral("Servo開始前に最新の realtime_feedback を現在値として読み込みます。"));
    this->loadCurrentTarget();
    return;
  }
  if (!this->control_state_client_->service_is_ready()) {
    this->appendLog(QStringLiteral("change_control_state サービスが利用できません。"), true);
    return;
  }

  const ServoMode requested_mode = this->selectedMode();
  auto request = std::make_shared<ChangeControlState::Request>();
  request->target_state.control_state = requested_mode == ServoMode::SERVO_J ?
    mg400_msgs::msg::ControlState::SERVO_J : mg400_msgs::msg::ControlState::SERVO_P;
  request->lease_id = 0;
  this->setServiceBusy(true);
  this->appendLog(
    requested_mode == ServoMode::SERVO_J ? QStringLiteral("ServoJモードを要求しました。") :
    QStringLiteral("ServoPモードを要求しました。"));
  this->control_state_client_->async_send_request(
    request, [this, requested_mode](rclcpp::Client<ChangeControlState>::SharedFuture future) {
      bool started = false;
      try {
        const auto response = future.get();
        const std::uint8_t expected_state = requested_mode == ServoMode::SERVO_J ?
        mg400_msgs::msg::ControlState::SERVO_J : mg400_msgs::msg::ControlState::SERVO_P;
        started = response->success && response->lease_id != 0 &&
        response->current_state.control_state == expected_state;
        if (started) {
          this->lease_id_ = response->lease_id;
          this->active_mode_ = requested_mode;
          this->command_active_ = false;
          this->appendLog(
            QStringLiteral("Servoモード開始: lease_id=%1 (%2)。Target送信は停止中です。")
            .arg(this->lease_id_)
            .arg(QString::fromStdString(response->message)));
        } else {
          this->appendLog(
            QStringLiteral("Servoモード開始に失敗しました: %1")
            .arg(QString::fromStdString(response->message)), true);
        }
      } catch (const std::exception & error) {
        this->appendLog(QStringLiteral("Servo開始応答エラー: %1").arg(error.what()), true);
      }
      this->setServiceBusy(false);
      if (!started) {
        this->command_active_ = false;
      }
      this->updateUiState();
      this->continueCloseIfNeeded();
    });
}

void MainWindow::startPublishing()
{
  if (this->service_busy_ || this->closing_ || this->command_active_ ||
    this->lease_id_ == 0 || !this->selectedTargetInitialized())
  {
    return;
  }

  this->filter_updated_at_ = std::chrono::steady_clock::now();
  this->command_active_ = true;
  this->publishServoTarget();
  this->appendLog(
    QStringLiteral("Targetの周期送信を開始しました (LPF cutoff: %1 Hz)。")
    .arg(this->filter_cutoff_spin_box_->value(), 0, 'f', 1));
  this->updateUiState();
}

void MainWindow::stopPublishing()
{
  if (!this->command_active_) {
    return;
  }

  this->command_active_ = false;
  this->appendLog(
    QStringLiteral(
      "Targetの周期送信を停止しました。無指令状態が30秒続くとMG400側がIDLEへ戻ります。"));
  this->updateUiState();
}

void MainWindow::requestServoStop()
{
  this->command_active_ = false;
  this->updateUiState();
  if (this->lease_id_ == 0) {
    this->appendLog(QStringLiteral("停止対象のServo leaseはありません。"));
    this->continueCloseIfNeeded();
    return;
  }
  if (this->service_busy_ || this->stop_request_in_flight_) {
    return;
  }
  if (!this->control_state_client_->service_is_ready()) {
    this->appendLog(
      QStringLiteral("change_control_state サービスが利用できません。指令送信は停止しました。"),
      true);
    return;
  }

  const std::uint64_t stopping_lease = this->lease_id_;
  auto request = std::make_shared<ChangeControlState::Request>();
  request->target_state.control_state = mg400_msgs::msg::ControlState::IDLE;
  request->lease_id = stopping_lease;
  this->stop_request_in_flight_ = true;
  this->setServiceBusy(true);
  this->appendLog(QStringLiteral("Target送信を停止し、IDLEへの遷移を要求しました。"));
  this->control_state_client_->async_send_request(
    request, [this, stopping_lease](rclcpp::Client<ChangeControlState>::SharedFuture future) {
      try {
        const auto response = future.get();
        if (response->success &&
        response->current_state.control_state == mg400_msgs::msg::ControlState::IDLE)
        {
          if (this->lease_id_ == stopping_lease) {
            this->lease_id_ = 0;
          }
          this->appendLog(
            QStringLiteral("Servoを停止しました: %1")
            .arg(QString::fromStdString(response->message)));
        } else {
          this->appendLog(
            QStringLiteral("Servo停止に失敗しました: %1。指令送信はOFFのままです。")
            .arg(QString::fromStdString(response->message)), true);
        }
      } catch (const std::exception & error) {
        this->appendLog(QStringLiteral("Servo停止応答エラー: %1").arg(error.what()), true);
      }
      this->stop_request_in_flight_ = false;
      this->setServiceBusy(false);
      this->continueCloseIfNeeded();
    });
}

void MainWindow::publishServoTarget()
{
  if (!this->command_active_ || this->lease_id_ == 0) {
    return;
  }

  if (this->active_mode_ == ServoMode::SERVO_J) {
    const auto degrees = this->filteredTargetValues();
    mg400_msgs::msg::ServoJ message;
    message.lease_id = this->lease_id_;
    for (std::size_t index = 0; index < degrees.size(); ++index) {
      message.joint_angles.at(index) = degrees.at(index) * PI / 180.0;
    }
    this->servo_j_publisher_->publish(std::move(message));
    return;
  }

  const auto values = this->filteredTargetValues();
  const double yaw = values.at(3) * PI / 180.0;
  mg400_msgs::msg::ServoP message;
  message.lease_id = this->lease_id_;
  message.pose.header.stamp = this->now();
  message.pose.header.frame_id = this->servo_p_frame_id_;
  message.pose.pose.position.x = values.at(0) / 1000.0;
  message.pose.pose.position.y = values.at(1) / 1000.0;
  message.pose.pose.position.z = values.at(2) / 1000.0;
  message.pose.pose.orientation.x = 0.0;
  message.pose.pose.orientation.y = 0.0;
  message.pose.pose.orientation.z = std::sin(yaw / 2.0);
  message.pose.pose.orientation.w = std::cos(yaw / 2.0);
  this->servo_p_publisher_->publish(std::move(message));
}

void MainWindow::handleRobotMode(
  const mg400_msgs::msg::RobotMode::ConstSharedPtr message)
{
  const std::uint64_t previous = this->robot_mode_;
  this->robot_mode_ = message->robot_mode;
  this->robot_mode_received_ = true;
  if (previous != this->robot_mode_) {
    this->appendLog(QStringLiteral("RobotMode: %1").arg(robotModeName(this->robot_mode_)));
  }
  this->updateUiState();
}

void MainWindow::handleRealtimeFeedback(
  const mg400_msgs::msg::RealtimeFeedback::ConstSharedPtr message)
{
  for (std::size_t index = 0; index < this->current_joint_angles_.size(); ++index) {
    this->current_joint_angles_.at(index) = message->q_actual.at(index);
    this->current_joint_value_labels_.at(index)->setText(
      QStringLiteral("%1 deg").arg(message->q_actual.at(index), 0, 'f', 2));
  }
  this->current_pose_ = {{
    message->tool_vector_actual.at(0),
    message->tool_vector_actual.at(1),
    message->tool_vector_actual.at(2),
    std::remainder(message->tool_vector_actual.at(3), 360.0),
  }};
  for (std::size_t index = 0; index < this->current_pose_value_labels_.size(); ++index) {
    this->current_pose_value_labels_.at(index)->setText(
      index < 3 ?
      QStringLiteral("%1 mm").arg(message->tool_vector_actual.at(index), 0, 'f', 2) :
      QStringLiteral("%1 deg").arg(message->tool_vector_actual.at(index), 0, 'f', 2));
  }

  const bool first_message = !this->realtime_feedback_received_;
  this->realtime_feedback_received_ = true;
  if (first_message) {
    this->appendLog(QStringLiteral("realtime_feedback の受信を開始しました。"));
    this->updateUiState();
  }
}

void MainWindow::handleControlState(
  const mg400_msgs::msg::ControlState::ConstSharedPtr message)
{
  const std::uint8_t previous = this->control_state_;
  this->control_state_ = message->control_state;
  this->control_state_received_ = true;

  const std::uint8_t expected_state = this->active_mode_ == ServoMode::SERVO_J ?
    mg400_msgs::msg::ControlState::SERVO_J : mg400_msgs::msg::ControlState::SERVO_P;
  if (this->lease_id_ != 0 && this->control_state_ != expected_state &&
    !this->stop_request_in_flight_)
  {
    this->command_active_ = false;
    this->lease_id_ = 0;
    this->appendLog(
      QStringLiteral("Servo leaseが失われたため、指令送信を停止しました。"), true);
  }
  if (previous != this->control_state_) {
    this->appendLog(
      QStringLiteral("ControlState: %1").arg(controlStateName(this->control_state_)));
  }
  this->updateUiState();
  this->continueCloseIfNeeded();
}

void MainWindow::appendLog(const QString & message, const bool error)
{
  if (!this->log_view_) {
    return;
  }
  const QString prefix = error ? QStringLiteral("ERROR") : QStringLiteral("INFO ");
  this->log_view_->appendPlainText(
    QStringLiteral("[%1] %2  %3")
    .arg(QTime::currentTime().toString(QStringLiteral("HH:mm:ss.zzz")), prefix, message));
}

void MainWindow::closeEvent(QCloseEvent * event)
{
  if (this->close_allowed_ || this->lease_id_ == 0) {
    event->accept();
    return;
  }

  event->ignore();
  if (this->closing_) {
    return;
  }
  this->closing_ = true;
  this->pending_servo_start_ = false;
  this->command_active_ = false;
  this->appendLog(QStringLiteral("終了前にServoを停止しています。"));
  this->updateUiState();
  this->close_timeout_->start(3500);
  this->continueCloseIfNeeded();
}

void MainWindow::continueCloseIfNeeded()
{
  if (!this->closing_) {
    return;
  }
  if (this->lease_id_ == 0) {
    this->finishClose();
  } else if (!this->service_busy_ && !this->stop_request_in_flight_) {
    this->requestServoStop();
  }
}

void MainWindow::finishClose()
{
  if (this->close_allowed_) {
    return;
  }
  this->command_active_ = false;
  this->close_allowed_ = true;
  this->close_timeout_->stop();
  QTimer::singleShot(
    0, this, [this]() {
      this->close();
    });
}

QString MainWindow::robotModeName(const std::uint64_t mode)
{
  switch (mode) {
    case mg400_msgs::msg::RobotMode::INIT:
      return QStringLiteral("INIT (1)");
    case mg400_msgs::msg::RobotMode::BRAKE_OPEN:
      return QStringLiteral("BRAKE_OPEN (2)");
    case mg400_msgs::msg::RobotMode::POWER_STATUS:
      return QStringLiteral("POWER_STATUS (3)");
    case mg400_msgs::msg::RobotMode::DISABLED:
      return QStringLiteral("DISABLED (4)");
    case mg400_msgs::msg::RobotMode::ENABLE:
      return QStringLiteral("ENABLE (5)");
    case mg400_msgs::msg::RobotMode::BACKDRIVE:
      return QStringLiteral("BACKDRIVE (6)");
    case mg400_msgs::msg::RobotMode::RUNNING:
      return QStringLiteral("RUNNING (7)");
    case mg400_msgs::msg::RobotMode::RECORDING:
      return QStringLiteral("RECORDING (8)");
    case mg400_msgs::msg::RobotMode::ERROR:
      return QStringLiteral("ERROR (9)");
    case mg400_msgs::msg::RobotMode::PAUSE:
      return QStringLiteral("PAUSE (10)");
    case mg400_msgs::msg::RobotMode::JOG:
      return QStringLiteral("JOG (11)");
    case mg400_msgs::msg::RobotMode::INVALID:
      return QStringLiteral("INVALID (12)");
    default:
      return QStringLiteral("UNKNOWN (%1)").arg(mode);
  }
}

QString MainWindow::controlStateName(const std::uint8_t state)
{
  switch (state) {
    case mg400_msgs::msg::ControlState::UNAVAILABLE:
      return QStringLiteral("UNAVAILABLE");
    case mg400_msgs::msg::ControlState::IDLE:
      return QStringLiteral("IDLE");
    case mg400_msgs::msg::ControlState::SERVO_J:
      return QStringLiteral("SERVO_J");
    case mg400_msgs::msg::ControlState::SERVO_P:
      return QStringLiteral("SERVO_P");
    default:
      return QStringLiteral("UNKNOWN (%1)").arg(state);
  }
}

}  // namespace mg400_operation_gui
