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

#include "mg400_rviz_plugin/panel_mg400_monitor.hpp"

#include <mg400_msgs/msg/robot_mode.hpp>

namespace mg400_rviz_plugin
{
namespace
{
constexpr int DETAIL_PRECISION = 3;

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

QString robotModeBadgeText(const uint64_t mode)
{
  using RobotMode = mg400_msgs::msg::RobotMode;
  switch (mode) {
    case RobotMode::INIT:
      return "Init";
    case RobotMode::BRAKE_OPEN:
      return "Brake Open";
    case RobotMode::DISABLED:
      return "Disabled";
    case RobotMode::ENABLE:
      return "Enable";
    case RobotMode::BACKDRIVE:
      return "Backdrive";
    case RobotMode::RUNNING:
      return "Running";
    case RobotMode::RECORDING:
      return "Recording";
    case RobotMode::ERROR:
      return "Error";
    case RobotMode::PAUSE:
      return "Pause";
    case RobotMode::JOG:
      return "Jog";
    case RobotMode::INVALID:
      return "Invalid";
    default:
      return "Unknown";
  }
}

QString robotModeBadgeColor(const uint64_t mode)
{
  using RobotMode = mg400_msgs::msg::RobotMode;
  switch (mode) {
    case RobotMode::ENABLE:
    case RobotMode::RUNNING:
      return "#1f8f4d";
    case RobotMode::DISABLED:
      return "#64748b";
    case RobotMode::ERROR:
    case RobotMode::INVALID:
      return "#c2410c";
    case RobotMode::PAUSE:
      return "#b7791f";
    case RobotMode::JOG:
    case RobotMode::BACKDRIVE:
      return "#2563eb";
    case RobotMode::RECORDING:
      return "#7c3aed";
    case RobotMode::INIT:
    case RobotMode::BRAKE_OPEN:
    default:
      return "#475569";
  }
}

QString formatDouble(const double value)
{
  return QString::number(value, 'f', DETAIL_PRECISION);
}

template<typename ArrayT>
QString formatArray(const ArrayT & values)
{
  QStringList text;
  for (const auto value : values) {
    text << formatDouble(value);
  }
  return text.join(", ");
}

QString formatHex64(const uint64_t value)
{
  return QString("0x%1").arg(
    static_cast<qulonglong>(value), 16, 16, QLatin1Char('0'));
}

QString formatUInt(const uint64_t value)
{
  return QString::number(static_cast<qulonglong>(value));
}

template<typename ArrayT>
QString formatUIntArray(const ArrayT & values)
{
  QStringList text;
  for (const auto value : values) {
    text << QString::number(static_cast<uint>(value));
  }
  return text.join(", ");
}

QString formatStamp(const builtin_interfaces::msg::Time & stamp)
{
  return QString("%1.%2")
         .arg(stamp.sec)
         .arg(stamp.nanosec, 9, 10, QLatin1Char('0'));
}

const QStringList & realtimeFeedbackFieldNames()
{
  static const QStringList names = {
    "stamp",
    "message_size",
    "digital_inputs",
    "digital_outputs",
    "robot_mode",
    "time_stamp",
    "run_time",
    "test_value",
    "speed_scaling",
    "v_robot",
    "i_robot",
    "program_state",
    "safety_io_in",
    "safety_io_out",
    "q_target",
    "qd_target",
    "qdd_target",
    "i_target",
    "m_target",
    "q_actual",
    "qd_actual",
    "i_actual",
    "actual_tcp_force",
    "tool_vector_actual",
    "tcp_speed_actual",
    "tcp_force",
    "tool_vector_target",
    "tcp_speed_target",
    "motor_temperatures",
    "joint_modes",
    "v_actual",
    "hand_type",
    "user",
    "tool",
    "run_queued_cmd",
    "pause_cmd_flag",
    "velocity_ratio",
    "acceleration_ratio",
    "xyz_velocity_ratio",
    "r_velocity_ratio",
    "xyz_acceleration_ratio",
    "r_acceleration_ratio",
    "brake_status",
    "enable_status",
    "drag_status",
    "running_status",
    "error_status",
    "jog_status_cr",
    "cr_robot_type",
    "drag_button_signal",
    "enable_button_signal",
    "record_button_signal",
    "reappear_button_signal",
    "jaw_button_signal",
    "six_force_online",
    "collision_state",
    "arm_approach_state",
    "j4_approach_state",
    "j5_approach_state",
    "j6_approach_state",
    "vibration_dis_z",
    "current_command_id",
    "m_actual",
    "load",
    "center_x",
    "center_y",
    "center_z",
    "user_coordinates",
    "tool_coordinates",
    "six_force_value",
    "target_quaternion",
    "actual_quaternion",
    "auto_manual_mode",
    "export_status",
    "safety_state",
    "safe_state",
  };
  return names;
}

void initializeValueTable(QTableWidget * table, const QStringList & field_names)
{
  table->setColumnCount(2);
  table->setRowCount(field_names.size());
  table->setHorizontalHeaderLabels(QStringList({"Field", "Value"}));
  table->verticalHeader()->setVisible(false);
  table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  table->horizontalHeader()->setStretchLastSection(false);
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table->setSelectionBehavior(QAbstractItemView::SelectRows);
  table->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
  table->setAlternatingRowColors(true);
  table->setWordWrap(false);
  table->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);

  auto value_font = table->font();
  value_font.setStyleHint(QFont::Monospace);
  value_font.setFixedPitch(true);

  for (int i = 0; i < field_names.size(); ++i) {
    table->setItem(i, 0, new QTableWidgetItem(field_names.at(i)));
    auto * item = new QTableWidgetItem("-");
    item->setFont(value_font);
    item->setToolTip("-");
    table->setItem(i, 1, item);
  }
}

QLabel * createBadgeLabel(QWidget * parent)
{
  auto * label = new QLabel("-", parent);
  label->setAlignment(Qt::AlignCenter);
  label->setMinimumWidth(104);
  label->setMinimumHeight(24);
  label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  return label;
}

void setBadge(QLabel * label, const QString & text, const QString & color)
{
  label->setText(text);
  label->setStyleSheet(
    QString(
      "QLabel {"
      "background-color: %1;"
      "color: white;"
      "border-radius: 7px;"
      "padding: 3px 10px;"
      "font-weight: bold;"
      "}").arg(color));
}

QLabel * createSummaryLineLabel(QWidget * parent)
{
  auto * label = new QLabel("-", parent);
  label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  return label;
}

QWidget * createLineSummaryGroup(const QString & title, QLabel * & value_label, QWidget * parent)
{
  auto * group = new QGroupBox(title, parent);
  auto * layout = new QVBoxLayout;
  layout->setContentsMargins(6, 8, 6, 6);
  value_label = createSummaryLineLabel(group);
  layout->addWidget(value_label);
  group->setLayout(layout);
  return group;
}

void setSummaryStatus(QLabel * label, const QString & text, const QString & color)
{
  setBadge(label, text, color);
}

void setRobotModeBadge(QLabel * label, const uint64_t mode)
{
  setBadge(label, robotModeBadgeText(mode), robotModeBadgeColor(mode));
}

template<typename ArrayT>
QString formatTcpSummary(const ArrayT & values)
{
  return QString("X [mm]: %1, Y [mm]: %2, Z [mm]: %3, Rx [deg]: %4")
         .arg(values[0], 0, 'f', 1)
         .arg(values[1], 0, 'f', 1)
         .arg(values[2], 0, 'f', 1)
         .arg(values[3], 0, 'f', 1);
}

template<typename ArrayT>
QString formatJointSummary(const ArrayT & values)
{
  return QString("J1 [deg]: %1, J2 [deg]: %2, J3 [deg]: %3, J4 [deg]: %4")
         .arg(values[0], 0, 'f', 1)
         .arg(values[1], 0, 'f', 1)
         .arg(values[2], 0, 'f', 1)
         .arg(values[3], 0, 'f', 1);
}

void setTableValue(
  QTableWidget * table, const int row, const QString & value,
  const QColor & color = QColor())
{
  auto * item = table->item(row, 1);
  if (!item) {
    item = new QTableWidgetItem;
    table->setItem(row, 1, item);
  }
  item->setText(value);
  item->setToolTip(value);

  if (color.isValid()) {
    item->setForeground(QBrush(color));
  } else {
    item->setForeground(QBrush(table->palette().text()));
  }
}
}  // namespace

Mg400MonitorPanel::Mg400MonitorPanel(QWidget * parent)
: Mg400PanelBase(parent),
  latest_realtime_feedback_received_time_(0, 0, RCL_ROS_TIME),
  has_realtime_feedback_(false)
{
  auto * layout = new QVBoxLayout;
  layout->addLayout(this->createNamespaceLayout());
  layout->addWidget(this->createMonitorSection());
  this->setLayout(layout);

  auto * output_timer = new QTimer(this);
  this->connect(output_timer, SIGNAL(timeout()), this, SLOT(tick()));
  output_timer->start(100);
}

QWidget * Mg400MonitorPanel::createMonitorSection()
{
  auto * container = new QWidget(this);
  auto * layout = new QVBoxLayout;
  layout->setContentsMargins(0, 0, 0, 0);
  container->setMinimumWidth(380);
  container->setMaximumWidth(560);

  auto * status_group = new QGroupBox("Status", container);
  auto * status_layout = new QHBoxLayout;
  status_layout->setContentsMargins(6, 8, 6, 6);
  status_layout->setSpacing(6);
  label_feedback_status_ = createBadgeLabel(status_group);
  label_feedback_robot_mode_ = createBadgeLabel(status_group);
  status_layout->addWidget(new QLabel("realtime:", status_group));
  status_layout->addWidget(label_feedback_status_);
  status_layout->addSpacing(18);
  status_layout->addWidget(new QLabel("robot mode:", status_group));
  status_layout->addWidget(label_feedback_robot_mode_);
  status_layout->addStretch();
  status_group->setLayout(status_layout);
  layout->addWidget(status_group);

  layout->addWidget(createLineSummaryGroup("TCP Actual", label_tcp_actual_, container));
  layout->addWidget(createLineSummaryGroup("TCP Target", label_tcp_target_, container));
  layout->addWidget(createLineSummaryGroup("Joint Actual", label_joint_actual_, container));
  layout->addWidget(createLineSummaryGroup("Joint Target", label_joint_target_, container));

  layout->addWidget(new QLabel("Realtime Feedback Details", container));
  realtime_feedback_table_ = new QTableWidget(container);
  initializeValueTable(realtime_feedback_table_, realtimeFeedbackFieldNames());
  realtime_feedback_table_->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  realtime_feedback_table_->setMinimumHeight(260);
  layout->addWidget(realtime_feedback_table_, 1);

  container->setLayout(layout);
  return container;
}

void Mg400MonitorPanel::onRosNodeInitialized()
{
  this->setupRosInterfaces();
}

void Mg400MonitorPanel::onNamespaceChanged()
{
  this->setupRosInterfaces();
}

void Mg400MonitorPanel::setupRosInterfaces()
{
  if (!this->rawNode()) {
    return;
  }

  {
    const std::lock_guard<std::mutex> lock(this->realtime_feedback_mutex_);
    this->latest_realtime_feedback_ = RealtimeFeedback();
    this->latest_realtime_feedback_received_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    this->has_realtime_feedback_ = false;
  }

  realtime_feedback_sub_ = this->rawNode()->create_subscription<RealtimeFeedback>(
    this->makeTopic("realtime_feedback"), rclcpp::SensorDataQoS().keep_last(1),
    [this](const RealtimeFeedback::ConstSharedPtr msg) {
      const std::lock_guard<std::mutex> lock(this->realtime_feedback_mutex_);
      this->latest_realtime_feedback_ = *msg;
      this->latest_realtime_feedback_received_time_ = this->rawNode()->now();
      this->has_realtime_feedback_ = true;
    });
}

void Mg400MonitorPanel::tick()
{
  this->updateMonitorLabels();
}

void Mg400MonitorPanel::updateMonitorLabels()
{
  RealtimeFeedback feedback;
  rclcpp::Time received_time(0, 0, RCL_ROS_TIME);
  bool has_feedback = false;
  {
    const std::lock_guard<std::mutex> lock(this->realtime_feedback_mutex_);
    feedback = this->latest_realtime_feedback_;
    received_time = this->latest_realtime_feedback_received_time_;
    has_feedback = this->has_realtime_feedback_;
  }

  if (!has_feedback || !this->rawNode()) {
    setSummaryStatus(this->label_feedback_status_, "No data", "#64748b");
    setBadge(this->label_feedback_robot_mode_, "Unknown", "#64748b");
    this->label_tcp_actual_->setText("-");
    this->label_tcp_target_->setText("-");
    this->label_joint_actual_->setText("-");
    this->label_joint_target_->setText("-");
    this->clearRealtimeFeedbackTable();
    return;
  }

  const auto age_seconds = (this->rawNode()->now() - received_time).seconds();
  if (age_seconds > 1.0) {
    setSummaryStatus(this->label_feedback_status_, "Stale", "#b7791f");
  } else {
    setSummaryStatus(this->label_feedback_status_, "Receiving", "#1f8f4d");
  }

  setRobotModeBadge(this->label_feedback_robot_mode_, feedback.robot_mode);
  this->label_tcp_actual_->setText(formatTcpSummary(feedback.tool_vector_actual));
  this->label_tcp_target_->setText(formatTcpSummary(feedback.tool_vector_target));
  this->label_joint_actual_->setText(formatJointSummary(feedback.q_actual));
  this->label_joint_target_->setText(formatJointSummary(feedback.q_target));
  this->updateRealtimeFeedbackTable(feedback);
}

void Mg400MonitorPanel::clearRealtimeFeedbackTable()
{
  if (!this->realtime_feedback_table_) {
    return;
  }

  for (int row = 0; row < this->realtime_feedback_table_->rowCount(); ++row) {
    setTableValue(this->realtime_feedback_table_, row, "-");
  }
}

void Mg400MonitorPanel::updateRealtimeFeedbackTable(const RealtimeFeedback & feedback)
{
  if (!this->realtime_feedback_table_) {
    return;
  }

  int row = 0;
  setTableValue(this->realtime_feedback_table_, row++, formatStamp(feedback.stamp));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.message_size));
  setTableValue(this->realtime_feedback_table_, row++, formatHex64(feedback.digital_inputs));
  setTableValue(this->realtime_feedback_table_, row++, formatHex64(feedback.digital_outputs));
  setTableValue(
    this->realtime_feedback_table_, row++,
    QString("%1 (%2)").arg(robotModeToString(feedback.robot_mode)).arg(feedback.robot_mode));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.time_stamp));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.run_time));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.test_value));
  setTableValue(this->realtime_feedback_table_, row++, formatDouble(feedback.speed_scaling));
  setTableValue(this->realtime_feedback_table_, row++, formatDouble(feedback.v_robot));
  setTableValue(this->realtime_feedback_table_, row++, formatDouble(feedback.i_robot));
  setTableValue(this->realtime_feedback_table_, row++, formatDouble(feedback.program_state));
  setTableValue(this->realtime_feedback_table_, row++, formatUIntArray(feedback.safety_io_in));
  setTableValue(this->realtime_feedback_table_, row++, formatUIntArray(feedback.safety_io_out));

  setTableValue(this->realtime_feedback_table_, row++, formatArray(feedback.q_target));
  setTableValue(this->realtime_feedback_table_, row++, formatArray(feedback.qd_target));
  setTableValue(this->realtime_feedback_table_, row++, formatArray(feedback.qdd_target));
  setTableValue(this->realtime_feedback_table_, row++, formatArray(feedback.i_target));
  setTableValue(this->realtime_feedback_table_, row++, formatArray(feedback.m_target));
  setTableValue(this->realtime_feedback_table_, row++, formatArray(feedback.q_actual));
  setTableValue(this->realtime_feedback_table_, row++, formatArray(feedback.qd_actual));
  setTableValue(this->realtime_feedback_table_, row++, formatArray(feedback.i_actual));
  setTableValue(this->realtime_feedback_table_, row++, formatArray(feedback.actual_tcp_force));
  setTableValue(this->realtime_feedback_table_, row++, formatArray(feedback.tool_vector_actual));
  setTableValue(this->realtime_feedback_table_, row++, formatArray(feedback.tcp_speed_actual));
  setTableValue(this->realtime_feedback_table_, row++, formatArray(feedback.tcp_force));
  setTableValue(this->realtime_feedback_table_, row++, formatArray(feedback.tool_vector_target));
  setTableValue(this->realtime_feedback_table_, row++, formatArray(feedback.tcp_speed_target));
  setTableValue(this->realtime_feedback_table_, row++, formatArray(feedback.motor_temperatures));
  setTableValue(this->realtime_feedback_table_, row++, formatArray(feedback.joint_modes));
  setTableValue(this->realtime_feedback_table_, row++, formatArray(feedback.v_actual));

  setTableValue(this->realtime_feedback_table_, row++, formatUIntArray(feedback.hand_type));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.user));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.tool));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.run_queued_cmd));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.pause_cmd_flag));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.velocity_ratio));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.acceleration_ratio));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.xyz_velocity_ratio));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.r_velocity_ratio));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.xyz_acceleration_ratio));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.r_acceleration_ratio));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.brake_status));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.enable_status));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.drag_status));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.running_status));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.error_status));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.jog_status_cr));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.cr_robot_type));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.drag_button_signal));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.enable_button_signal));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.record_button_signal));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.reappear_button_signal));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.jaw_button_signal));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.six_force_online));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.collision_state));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.arm_approach_state));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.j4_approach_state));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.j5_approach_state));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.j6_approach_state));

  setTableValue(
    this->realtime_feedback_table_, row++,
    formatDouble(feedback.vibration_dis_z));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.current_command_id));
  setTableValue(this->realtime_feedback_table_, row++, formatArray(feedback.m_actual));
  setTableValue(this->realtime_feedback_table_, row++, formatDouble(feedback.load));
  setTableValue(this->realtime_feedback_table_, row++, formatDouble(feedback.center_x));
  setTableValue(this->realtime_feedback_table_, row++, formatDouble(feedback.center_y));
  setTableValue(this->realtime_feedback_table_, row++, formatDouble(feedback.center_z));
  setTableValue(this->realtime_feedback_table_, row++, formatArray(feedback.user_coordinates));
  setTableValue(this->realtime_feedback_table_, row++, formatArray(feedback.tool_coordinates));
  setTableValue(this->realtime_feedback_table_, row++, formatArray(feedback.six_force_value));
  setTableValue(this->realtime_feedback_table_, row++, formatArray(feedback.target_quaternion));
  setTableValue(this->realtime_feedback_table_, row++, formatArray(feedback.actual_quaternion));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.auto_manual_mode));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.export_status));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.safety_state));
  setTableValue(this->realtime_feedback_table_, row++, formatUInt(feedback.safe_state));
}
}  // namespace mg400_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mg400_rviz_plugin::Mg400MonitorPanel, rviz_common::Panel)
