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

#include "mg400_rviz_plugin/param_identify.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <set>
#include <stdexcept>
#include <string>

#include <mg400_common/mg400_ik_util.hpp>
#include <mg400_interface/joint_handler.hpp>
#include <rviz_common/display_context.hpp>
#include <yaml-cpp/yaml.h>

namespace mg400_rviz_plugin
{
namespace
{
constexpr double kPositionTolerance = 0.5 * M_PI / 180.0;
constexpr double kStableTolerance = 0.2 * M_PI / 180.0;
YAML::Node yamlTime(int64_t nanoseconds)
{
  const auto stamp = rclcpp::Time(nanoseconds).operator builtin_interfaces::msg::Time();
  YAML::Node node;
  node["sec"] = stamp.sec;
  node["nanosec"] = stamp.nanosec;
  return node;
}

QString snapshotPath(const QString & path)
{
  const QFileInfo file(path);
  return file.absoluteDir().filePath(file.completeBaseName() + ".yaml");
}

void renameFile(const QString & source, const QString & destination)
{
  if (std::rename(
      QFile::encodeName(source).constData(),
      QFile::encodeName(destination).constData()) != 0)
  {
    throw std::runtime_error(std::strerror(errno));
  }
}

void saveRecordingPair(const QString & directory, const QString & path)
{
  const std::array<QString, 2> outputs = {path, snapshotPath(path)};
  std::array<QString, 2> staged;
  std::array<QString, 2> backups;
  std::array<bool, 2> backed_up{};
  std::array<bool, 2> installed{};
  for (size_t i = 0; i < outputs.size(); ++i) {
    const QFileInfo output(outputs[i]);
    if (output.isSymLink() || (output.exists() && !output.isFile())) {
      throw std::runtime_error("output is no longer a regular file");
    }
    staged[i] = QDir(directory).filePath(output.fileName());
    backups[i] = QDir(directory).filePath(QString("previous-%1").arg(i));
  }
  try {
    for (size_t i = 0; i < outputs.size(); ++i) {
      if (QFileInfo::exists(outputs[i])) {
        renameFile(outputs[i], backups[i]);
        backed_up[i] = true;
      }
    }
    for (size_t i = 0; i < outputs.size(); ++i) {
      renameFile(staged[i], outputs[i]);
      installed[i] = true;
    }
  } catch (const std::exception & error) {
    // Return both the new data and the old pair to their original locations.
    // Retain the staging directory if any filesystem operation fails.
    bool restored = true;
    for (size_t i = 0; i < outputs.size(); ++i) {
      if (installed[i]) {
        restored &= std::rename(
          QFile::encodeName(outputs[i]).constData(),
          QFile::encodeName(staged[i]).constData()) == 0;
      }
      if (backed_up[i]) {
        restored &= std::rename(
          QFile::encodeName(backups[i]).constData(),
          QFile::encodeName(outputs[i]).constData()) == 0;
      }
    }
    throw std::runtime_error(
            std::string(error.what()) +
            (restored ? "" : "; previous files could not be fully restored"));
  }
}

const QStringList kKinds = {"base", "train", "check", "move"};

bool validAngles(const std::array<double, 4> & angles)
{
  return std::all_of(angles.begin(), angles.end(), [](double v) {return std::isfinite(v);}) &&
         mg400_common::MG400IKUtil().InMG400Range(
    std::vector<double>(angles.begin(), angles.end()));
}
void yamlKeys(const YAML::Node & node, const std::set<std::string> & keys, const char * name)
{
  if (!node.IsMap() || node.size() != keys.size()) {
    throw std::runtime_error(std::string(name) + ": missing or unexpected settings");
  }
  std::set<std::string> seen;
  for (const auto & entry : node) {
    const auto key = entry.first.as<std::string>();
    if (!keys.count(key) || !seen.insert(key).second) {
      throw std::runtime_error(std::string(name) + ": unknown or duplicate key " + key);
    }
  }
}

double yamlNumber(const YAML::Node & node, const char * name, double minimum, double maximum)
{
  const double value = node.as<double>();
  if (!std::isfinite(value) || value < minimum || value > maximum) {
    throw std::runtime_error(std::string(name) + ": value outside permitted range");
  }
  return value;
}

int yamlInteger(const YAML::Node & node, const char * name, int minimum, int maximum)
{
  const int value = node.as<int>();
  if (value < minimum || value > maximum) {
    throw std::runtime_error(std::string(name) + ": integer outside permitted range");
  }
  return value;
}

template<size_t N>
std::array<double, N> yamlVector(const YAML::Node & node, const char * name)
{
  if (!node.IsSequence() || node.size() != N) {
    throw std::runtime_error(std::string(name) + ": wrong array length");
  }
  std::array<double, N> values;
  for (size_t i = 0; i < N; ++i) {
    values[i] = node[i].as<double>();
    if (!std::isfinite(values[i])) {
      throw std::runtime_error(std::string(name) + ": values must be finite");
    }
  }
  return values;
}

}  // namespace

IdentifyPanel::IdentifyPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * outer = new QVBoxLayout(this);
  auto * scroll = new QScrollArea;
  scroll->setWidgetResizable(true);
  auto * content = new QWidget;
  auto * layout = new QVBoxLayout(content);
  scroll->setWidget(content);
  outer->addWidget(scroll);
  auto * robot = new QHBoxLayout;
  enable_ = new QPushButton("Enable");
  enable_->setObjectName("enable_robot");
  disable_ = new QPushButton("Stop / Disable");
  disable_->setObjectName("disable_robot");
  clear_ = new QPushButton("Clear Error");
  clear_->setObjectName("clear_error");
  for (auto * button : {enable_, disable_, clear_}) {
    robot->addWidget(button);
  }
  layout->addLayout(robot);
  mode_label_ = new QLabel("Robot Mode: INIT");
  mode_label_->setObjectName("robot_mode");
  layout->addWidget(mode_label_);
  service_status_ = new QLabel;
  service_status_->setObjectName("service_status");
  service_status_->setWordWrap(true);
  layout->addWidget(service_status_);
  load_config_ = new QPushButton("Load YAML...");
  load_config_->setObjectName("load_config");
  layout->addWidget(load_config_);
  config_status_ = new QLabel("No experiment YAML loaded.");
  config_status_->setObjectName("config_status");
  config_status_->setWordWrap(true);
  config_status_->setTextFormat(Qt::PlainText);
  layout->addWidget(config_status_);
  auto * payload = new QGroupBox("Enable payload (from YAML)");
  auto * payload_layout = new QGridLayout(payload);
  const QStringList fields = {"load", "center_x", "center_y", "center_z"};
  const QStringList labels = {"Load [kg]", "Center X [mm]", "Center Y [mm]", "Center Z [mm]"};
  for (size_t i = 0; i < payload_values_.size(); ++i) {
    auto * value = new QLabel("--");
    payload_values_[i] = value;
    value->setObjectName("enable_" + fields[i]);
    payload_layout->addWidget(new QLabel(labels[i]), i, 0);
    payload_layout->addWidget(value, i, 1);
  }
  layout->addWidget(payload);
  settings_summary_ = new QLabel("Motion and measurement settings are read from YAML.");
  settings_summary_->setObjectName("settings_summary");
  settings_summary_->setWordWrap(true);
  layout->addWidget(settings_summary_);
  poses_ = new QTableWidget(0, 5);
  poses_->setObjectName("identify_poses");
  poses_->setHorizontalHeaderLabels({"Kind", "J1 [deg]", "J2 [deg]", "J3 [deg]", "J4 [deg]"});
  poses_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  poses_->setSelectionMode(QAbstractItemView::NoSelection);
  poses_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  poses_->setMinimumHeight(180);
  layout->addWidget(poses_);
  run_ = new QPushButton("Run and record...");
  run_->setObjectName("run_identify");
  layout->addWidget(run_);
  run_status_ = new QLabel("Load YAML, review its settings, then Enable and Run.");
  run_status_->setObjectName("run_status");
  run_status_->setWordWrap(true);
  layout->addWidget(run_status_);
  auto * table = new QGridLayout;
  const QStringList headings = {"Joint", "Angle [deg]", "Actual [A]", "Target [A]"};
  for (int column = 0; column < headings.size(); ++column) {
    table->addWidget(new QLabel(headings[column]), 0, column);
  }
  for (size_t i = 0; i < angles_.size(); ++i) {
    table->addWidget(new QLabel(QString("J%1").arg(i + 1)), i + 1, 0);
    angles_[i] = new QLabel("--");
    actual_[i] = new QLabel("--");
    target_[i] = new QLabel("--");
    const std::array<QLabel *, 3> cells = {angles_[i], actual_[i], target_[i]};
    const QStringList names = {"angle", "actual", "target"};
    for (size_t j = 0; j < cells.size(); ++j) {
      cells[j]->setObjectName(QString("identify_%1_j%2").arg(names[j]).arg(i + 1));
      table->addWidget(cells[j], i + 1, j + 1);
    }
  }
  layout->addLayout(table);
  telemetry_status_ = new QLabel;
  telemetry_status_->setObjectName("telemetry_status");
  telemetry_status_->setWordWrap(true);
  layout->addWidget(telemetry_status_);
  recording_status_ = new QLabel("No recording.");
  recording_status_->setObjectName("recording_status");
  recording_status_->setWordWrap(true);
  layout->addWidget(recording_status_);

  connect(
    enable_, &QPushButton::clicked, this, [this]() {
      auto request = std::make_shared<Enable::Request>(settings_.payload);
      if (phase_ == Phase::Idle && mode_ == RobotMode::DISABLED && settings_loaded_) {
        requestService<Enable>(
          enable_client_, request, "Enable", [this, request](bool ok) {
            payload_confirmed_ = ok;
            if (ok) {
              enabled_payload_ = *request;
            }
          });
      }
    });
  connect(disable_, &QPushButton::clicked, this, [this]() {endRun("Stopped by operator.", true);});
  connect(
    clear_, &QPushButton::clicked, this, [this]() {
      requestService<mg400_msgs::srv::ClearError>(
        clear_client_,
        std::make_shared<mg400_msgs::srv::ClearError::Request>(), "Clear Error");
    });
  connect(
    load_config_, &QPushButton::clicked, this, [this]() {
      const auto path = QFileDialog::getOpenFileName(
        this, "Load experiment settings", config_path_, "YAML (*.yaml *.yml)");
      if (!path.isEmpty()) {
        loadConfig(path);
      }
    });
  connect(
    run_, &QPushButton::clicked, this, [this]() {
      const auto filename = QString("param_identify_%1.mcap").arg(
        QDateTime::currentDateTimeUtc().toString("yyyyMMdd_HHmmss_zzz"));
      const auto path = QFileDialog::getSaveFileName(
        this, "Run and record", filename, "MCAP (*.mcap)", nullptr,
        QFileDialog::DontConfirmOverwrite);
      if (!path.isEmpty()) {
        startSequence(path.endsWith(".mcap", Qt::CaseInsensitive) ? path : path + ".mcap");
      }
    });
  auto * timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &IdentifyPanel::tick);
  timer->start(20);
  refreshControls();
}

IdentifyPanel::~IdentifyPanel()
{
  if (phase_ != Phase::Idle && disable_client_ && disable_client_->service_is_ready()) {
    // Best-effort stop on panel close. JointMovJ cancellation alone does not stop hardware.
    try {
      disable_client_->async_send_request(std::make_shared<mg400_msgs::srv::DisableRobot::Request>());
    } catch (const std::exception &) {
    }
  }
  stopRecording();
}

void IdentifyPanel::onInitialize()
{
  initializeRos(getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node());
}

void IdentifyPanel::initializeRos(const rclcpp::Node::SharedPtr & node)
{
  node_ = node;
  group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(group_, node->get_node_base_interface());
  rclcpp::SubscriptionOptions options;
  options.callback_group = group_;
  joints_sub_ = node->create_subscription<JointState>(
    "/mg400/joint_states", rclcpp::SensorDataQoS().keep_last(100),
    [this](const JointState::ConstSharedPtr msg) {onJoints(msg);}, options);
  currents_sub_ = node->create_subscription<JointCurrents>(
    "/mg400/joint_currents", rclcpp::SensorDataQoS().keep_last(100),
    [this](const JointCurrents::ConstSharedPtr msg) {onCurrents(msg);}, options);
  mode_sub_ = node->create_subscription<RobotMode>(
    "/mg400/robot_mode", rclcpp::SensorDataQoS().keep_last(1),
    [this](const RobotMode::ConstSharedPtr msg) {
      if ((mode_ == RobotMode::ENABLE || mode_ == RobotMode::RUNNING) &&
      msg->robot_mode != RobotMode::ENABLE && msg->robot_mode != RobotMode::RUNNING)
      {
        payload_confirmed_ = false;
      }
      mode_ = msg->robot_mode;
      mode_received_ = Clock::now();
      recordMessage(*msg, mode_sub_->get_topic_name());
    }, options);
  motion_ = rclcpp_action::create_client<Action>(node, "/mg400/joint_mov_j", group_);
  enable_client_ = node->create_client<Enable>(
    "/mg400/enable_robot", rmw_qos_profile_services_default, group_);
  disable_client_ = node->create_client<mg400_msgs::srv::DisableRobot>(
    "/mg400/disable_robot", rmw_qos_profile_services_default, group_);
  clear_client_ = node->create_client<mg400_msgs::srv::ClearError>(
    "/mg400/clear_error", rmw_qos_profile_services_default, group_);
}

void IdentifyPanel::appendPoseRow(const Pose & pose)
{
  const int row = poses_->rowCount();
  poses_->insertRow(row);
  poses_->setItem(row, 0, new QTableWidgetItem(pose.kind));
  for (size_t i = 0; i < pose.radians.size(); ++i) {
    const auto value = QString::number(mg400_interface::rad2degree(pose.radians[i]), 'g', 10);
    auto * item = new QTableWidgetItem(value);
    item->setToolTip(value);
    poses_->setItem(row, i + 1, item);
  }
}

bool IdentifyPanel::loadConfig(const QString & path)
{
  if (phase_ != Phase::Idle || recording_ || service_pending_ ||
    (mode_ != RobotMode::INIT && mode_ != RobotMode::DISABLED))
  {
    run_status_->setText("Disable the robot before loading experiment settings.");
    return false;
  }
  try {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly) || file.size() > 64 * 1024 * 1024) {
      throw std::runtime_error("Cannot read YAML (maximum 64 MiB).");
    }
    const auto bytes = file.readAll();
    const auto documents = YAML::LoadAll(bytes.toStdString());
    if (documents.size() != 1) {
      throw std::runtime_error("Expected one YAML document");
    }
    auto root = documents.front();
    root.remove("recording");
    yamlKeys(
      root, {"schema_version", "payload", "motion", "measurement", "identify", "poses"},
      "experiment");
    yamlInteger(root["schema_version"], "schema_version", 1, 1);
    Settings settings;
    const auto payload = root["payload"];
    const bool meters = static_cast<bool>(payload["center_x_m"]);
    const std::string suffix = meters ? "_m" : "_mm";
    yamlKeys(
      payload, {"load_kg", "center_x" + suffix, "center_y" + suffix,
        "center_z" + suffix}, "payload");
    const double mm_per_unit = meters ? 1000.0 : 1.0;
    std::array<double, 4> values = {
      yamlNumber(payload["load_kg"], "payload.load_kg", 0, 0.75), 0, 0, 0};
    for (size_t i = 0; i < 3; ++i) {
      const auto key = std::string("center_") + "xyz"[i] + suffix;
      values[i + 1] = mm_per_unit * yamlNumber(
        payload[key], key.c_str(), -500 / mm_per_unit, 500 / mm_per_unit);
    }
    for (const auto value : values) {
      if (std::abs(value * 1000 - std::round(value * 1000)) > 1e-7) {
        throw std::runtime_error("Enable payload supports at most three decimals in kg/mm");
      }
    }
    settings.payload.num_of_params = Enable::Request::FOUR_PARAM;
    settings.payload.load = values[0];
    settings.payload.center_x = values[1];
    settings.payload.center_y = values[2];
    settings.payload.center_z = values[3];
    const auto motion = root["motion"];
    yamlKeys(motion, {"speed_percent", "acceleration_percent"}, "motion");
    settings.speed_percent = yamlInteger(motion["speed_percent"], "motion.speed_percent", 1, 100);
    settings.acceleration_percent = yamlInteger(
      motion["acceleration_percent"],
      "motion.acceleration_percent", 1, 100);
    const auto measurement = root["measurement"];
    yamlKeys(measurement, {"settle_sec", "record_sec", "repeats"}, "measurement");
    settings.settle_sec = yamlNumber(measurement["settle_sec"], "measurement.settle_sec", 0.1, 30);
    settings.record_sec = yamlNumber(measurement["record_sec"], "measurement.record_sec", 1.2, 60);
    settings.repeats = yamlInteger(measurement["repeats"], "measurement.repeats", 1, 100);
    const auto identify = root["identify"];
    yamlKeys(
      identify, {"torque_constants_nm_per_a", "torque_signs", "joint_signs", "reference_deg",
        "min_duration_sec", "max_span_deg"}, "identify");
    for (const auto gain :
      yamlVector<4>(
        identify["torque_constants_nm_per_a"],
        "identify.torque_constants_nm_per_a"))
    {
      if (gain <= 0) {
        throw std::runtime_error("identify.torque_constants_nm_per_a must be positive");
      }
    }
    for (const auto * key : {"torque_signs", "joint_signs"}) {
      for (const auto sign : yamlVector<4>(identify[key], key)) {
        if (sign != -1 && sign != 1) {
          throw std::runtime_error(std::string(key) + " must contain only -1 or 1");
        }
      }
    }
    yamlVector<4>(identify["reference_deg"], "identify.reference_deg");
    const auto minimum = yamlNumber(
      identify["min_duration_sec"], "identify.min_duration_sec",
      0.001, 60);
    yamlNumber(identify["max_span_deg"], "identify.max_span_deg", 0.001, 180);
    if (settings.record_sec < minimum + 0.1) {
      throw std::runtime_error(
              "measurement.record_sec must exceed identify.min_duration_sec by 0.1s");
    }
    const auto poses = root["poses"];
    if (!poses.IsSequence() || poses.size() == 0 || poses.size() > 1000) {
      throw std::runtime_error("poses must contain 1 to 1000 entries");
    }
    for (const auto & row : poses) {
      yamlKeys(row, {"kind", "joints_deg"}, "pose");
      Pose pose;
      pose.kind = QString::fromStdString(row["kind"].as<std::string>());
      if (!kKinds.contains(pose.kind)) {
        throw std::runtime_error("pose.kind must be base, train, check, or move");
      }
      pose.radians = yamlVector<4>(row["joints_deg"], "pose.joints_deg");
      for (auto & angle : pose.radians) {
        angle = mg400_interface::degree2rad(angle);
      }
      if (!validAngles(pose.radians)) {
        throw std::runtime_error("Pose outside MG400 joint limits (including J2/J3 coupling)");
      }
      settings.poses.push_back(pose);
    }
    // Apply only after the entire file has passed validation. Never modify it during a run.
    settings_ = settings;
    settings_loaded_ = true;
    payload_confirmed_ = false;
    config_path_ = QFileInfo(path).absoluteFilePath();
    config_yaml_ = bytes;
    config_status_->setText(config_path_);
    for (size_t i = 0; i < values.size(); ++i) {
      payload_values_[i]->setText(QString::number(values[i], 'g', 10));
    }
    settings_summary_->setText(
      QString("Speed: %1%   Acceleration: %2%   CP: 0\nSettle: %3 s   Record: %4 s   Repeats: %5")
      .arg(settings.speed_percent).arg(settings.acceleration_percent).arg(settings.settle_sec)
      .arg(settings.record_sec).arg(settings.repeats));
    poses_->setRowCount(0);
    for (const auto & pose : settings.poses) {
      appendPoseRow(pose);
    }
    run_status_->setText(
      QString("Loaded %1 poses. Review settings, then Enable and Run.")
      .arg(settings.poses.size()));
    refreshControls();
    return true;
  } catch (const std::exception & error) {
    run_status_->setText(QString("Invalid experiment YAML: %1").arg(error.what()));
    return false;
  }
}

bool IdentifyPanel::robotReady() const
{
  return Clock::now() - mode_received_ < std::chrono::seconds(1) &&
         mode_ == RobotMode::ENABLE && payload_confirmed_ && !service_pending_ &&
         motion_ && motion_->action_server_is_ready() &&
         disable_client_ && disable_client_->service_is_ready();
}

bool IdentifyPanel::telemetryReady() const
{
  return Clock::now() - joints_received_ < std::chrono::seconds(1) &&
         Clock::now() - currents_received_ < std::chrono::seconds(1);
}

bool IdentifyPanel::atTarget() const
{
  for (size_t i = 0; i < joint_values_.size(); ++i) {
    if (std::abs(joint_values_[i] - commanded_pose_.radians[i]) > kPositionTolerance) {
      return false;
    }
  }
  return true;
}

bool IdentifyPanel::startSequence(const QString & path)
{
  if (phase_ != Phase::Idle || recording_ || !settings_loaded_ ||
    !robotReady() || !telemetryReady())
  {
    run_status_->setText(
      "Run requires experiment YAML, fresh telemetry, and Enable with its payload.");
    return false;
  }
  if (!path.endsWith(".mcap", Qt::CaseInsensitive)) {
    run_status_->setText("Run not started: choose an .mcap filename.");
    return false;
  }
  QStringList existing;
  for (const auto & filename : {path, snapshotPath(path)}) {
    const QFileInfo output(filename);
    if (output.exists() || output.isSymLink()) {
      existing << filename;
    }
  }
  const bool overwrite = !existing.isEmpty();
  if (overwrite) {
    const auto answer = QMessageBox::question(
      this, "Replace recording?",
      "Replace the existing MCAP / YAML files and start the robot sequence?\n\n" +
      existing.join('\n'), QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel);
    if (answer != QMessageBox::Yes) {
      run_status_->setText("Run canceled; existing files kept.");
      return false;
    }
    // Robot state can change while the confirmation dialog is open.
    if (!robotReady() || !telemetryReady()) {
      run_status_->setText("Run not started: check robot state and telemetry, then try again.");
      return false;
    }
  }
  if (!startRecording(path, overwrite)) {
    run_status_->setText("Run not started: " + recording_status_->text());
    return false;
  }
  plan_.clear();
  for (int repeat = 0; repeat < settings_.repeats; ++repeat) {
    plan_.insert(plan_.end(), settings_.poses.begin(), settings_.poses.end());
  }
  segment_ = 0;
  sendPose(plan_[segment_]);
  return true;
}

void IdentifyPanel::sendPose(const Pose & pose)
{
  commanded_pose_ = pose;
  phase_ = Phase::Moving;
  motion_deadline_ = Clock::now() + std::chrono::seconds(15);
  const auto generation = ++generation_;
  Action::Goal goal;
  goal.joint_angles = pose.radians;
  goal.set_speed_j = true;
  goal.speed_j = settings_.speed_percent;
  goal.set_acc_j = true;
  goal.acc_j = settings_.acceleration_percent;
  goal.set_cp = true;
  goal.cp = 0;
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.goal_response_callback = [this, generation](const GoalHandle::SharedPtr & handle) {
      if (generation != generation_) {
        if (handle) {
          disableRobot();  // A stop may have preceded acceptance of this request.
        }
        return;
      }
      if (!handle) {
        endRun("Motion rejected.", false);
      }
    };
  options.result_callback = [this, generation](const GoalHandle::WrappedResult & result) {
      if (generation != generation_) {
        return;
      }
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
        !result.result->result)
      {
        endRun("Motion failed; run aborted.", true);
        return;
      }
      phase_ = Phase::Settling;
      phase_started_ = Clock::now();
      stable_since_ = phase_started_;
      stable_anchor_ = joint_values_;
      motion_deadline_ = phase_started_ + std::chrono::seconds(60);
      run_status_->setText(
        QString("Settling at pose %1 / %2.").arg(segment_ + 1).arg(plan_.size()));
    };
  run_status_->setText(
    QString("Moving to pose %1 / %2 (%3).")
    .arg(segment_ + 1).arg(plan_.size()).arg(pose.kind));
  try {
    motion_->async_send_goal(goal, options);
  } catch (const std::exception & e) {
    endRun(QString("Motion request failed: %1").arg(e.what()), true);
  }
  refreshControls();
}

void IdentifyPanel::advanceSequence()
{
  if (++segment_ == plan_.size()) {
    endRun("Measurement sequence completed.", false);
  } else {
    sendPose(plan_[segment_]);
  }
}

void IdentifyPanel::endRun(const QString & reason, bool disable)
{
  ++generation_;
  phase_ = Phase::Idle;
  stopRecording(reason);
  run_status_->setText(reason);
  if (disable) {
    disableRobot();
  }
  refreshControls();
}

void IdentifyPanel::disableRobot()
{
  payload_confirmed_ = false;
  requestService<mg400_msgs::srv::DisableRobot>(
    disable_client_,
    std::make_shared<mg400_msgs::srv::DisableRobot::Request>(), "Disable");
}

template<typename Service>
void IdentifyPanel::requestService(
  const typename rclcpp::Client<Service>::SharedPtr & client,
  const typename Service::Request::SharedPtr & request, const QString & name,
  std::function<void(bool)> completed)
{
  if (service_pending_) {
    return;
  }
  if (!client || !client->service_is_ready()) {
    service_status_->setText(name + " unavailable.");
    return;
  }
  service_pending_ = true;
  service_deadline_ = Clock::now() + std::chrono::seconds(5);
  service_status_->setText(name + " requested...");
  try {
    const auto future = client->async_send_request(
      request,
      [this, name, completed](typename rclcpp::Client<Service>::SharedFuture response) {
        service_pending_ = false;
        remove_request_ = {};
        bool ok = false;
        try {
          const auto result = response.get();
          ok = result->result;
          service_status_->setText(
            ok ? name + " succeeded." :
            QString("%1 failed (error ID: %2).").arg(name).arg(result->error_id));
        } catch (const std::exception & e) {
          service_status_->setText(QString("%1 failed: %2").arg(name).arg(e.what()));
        }
        if (completed) {
          completed(ok);
        }
        refreshControls();
      });
    remove_request_ = [client, id = future.request_id]() {client->remove_pending_request(id);};
  } catch (const std::exception & e) {
    service_pending_ = false;
    service_status_->setText(QString("%1 failed: %2").arg(name).arg(e.what()));
    if (completed) {
      completed(false);
    }
  }
  refreshControls();
}

void IdentifyPanel::tick()
{
  if (node_) {
    // Drain bursts between GUI ticks; spin_some consumes at most one per subscription.
    executor_.spin_all(std::chrono::milliseconds(5));
  }
  const auto now = Clock::now();
  if (service_pending_ && now >= service_deadline_) {
    remove_request_();
    remove_request_ = {};
    service_pending_ = false;
    payload_confirmed_ = false;
    service_status_->setText("Robot service timed out; robot state is unconfirmed.");
  }
  if (phase_ != Phase::Idle) {
    if (node_->now().nanoseconds() < last_record_ns_) {
      endRun("ROS clock went backwards; run aborted.", true);
    } else if (now - mode_received_ >= std::chrono::seconds(1) ||
      (mode_ != RobotMode::ENABLE && mode_ != RobotMode::RUNNING))
    {
      endRun("Robot state lost or not enabled; run aborted.", true);
    } else if (now >= motion_deadline_) {
      endRun("Motion or settling timed out; run aborted.", true);
    } else if (!telemetryReady()) {
      endRun("Telemetry lost; run aborted.", true);
    } else if (phase_ == Phase::Settling) {
      if (mode_ != RobotMode::ENABLE || !atTarget()) {
        stable_since_ = now;
      } else if (std::chrono::duration<double>(now - stable_since_).count() >=
        settings_.settle_sec)
      {
        if (commanded_pose_.kind == "move") {
          advanceSequence();
        } else {
          phase_ = Phase::Sampling;
          phase_started_ = now;
          motion_deadline_ = now + std::chrono::milliseconds(
            static_cast<int64_t>((settings_.record_sec + 5.0) * 1000));
          sample_start_ns_ = node_->now().nanoseconds();
          measurement_windows_.push_back({segment_, commanded_pose_, sample_start_ns_, 0, false});
          sample_joints_ = sample_currents_ = 0;
          run_status_->setText(
            QString("Recording pose %1 / %2 (%3).")
            .arg(segment_ + 1).arg(plan_.size()).arg(commanded_pose_.kind));
        }
      }
    } else if (phase_ == Phase::Sampling) {
      if (mode_ != RobotMode::ENABLE || !atTarget() || stable_since_ > phase_started_) {
        endRun("Motion detected during sampling; segment rejected.", true);
      } else if (std::chrono::duration<double>(now - phase_started_).count() >=
        settings_.record_sec)
      {
        if (sample_joints_ < 5 || sample_currents_ < 5) {
          endRun("Insufficient samples; segment rejected.", true);
        } else {
          measurement_windows_.back().end_ns = node_->now().nanoseconds();
          measurement_windows_.back().complete = true;
          advanceSequence();
        }
      }
    }
  }
  refreshControls();
}

void IdentifyPanel::onJoints(const JointState::ConstSharedPtr msg)
{
  const std::array<const char *, 4> names = {mg400_interface::J1_NAME, mg400_interface::J2_1_NAME,
    mg400_interface::J4_2_NAME, mg400_interface::J5_NAME};
  std::array<double, 4> angles;
  for (size_t i = 0; i < names.size(); ++i) {
    const auto it = std::find_if(
      msg->name.begin(), msg->name.end(),
      [&](const std::string & name) {return QString::fromStdString(name).endsWith(names[i]);});
    const auto index = static_cast<size_t>(std::distance(msg->name.begin(), it));
    if (it == msg->name.end() || index >= msg->position.size() ||
      !std::isfinite(msg->position[index]))
    {
      return;
    }
    angles[i] = msg->position[index];
  }
  if (phase_ != Phase::Idle &&
    std::abs((node_->now() - rclcpp::Time(msg->header.stamp)).seconds()) > 1.0)
  {
    return;
  }
  joints_received_ = Clock::now();
  for (size_t i = 0; i < angles.size(); ++i) {
    if (std::abs(angles[i] - stable_anchor_[i]) > kStableTolerance) {
      stable_anchor_ = angles;
      stable_since_ = joints_received_;
      break;
    }
  }
  joint_values_ = angles;
  for (size_t i = 0; i < angles.size(); ++i) {
    angles_[i]->setText(QString::number(mg400_interface::rad2degree(angles[i]), 'f', 3));
  }
  if (recordMessage(*msg, joints_sub_->get_topic_name())) {
    ++joint_rows_;
    if (phase_ == Phase::Sampling &&
      rclcpp::Time(msg->header.stamp).nanoseconds() >= sample_start_ns_)
    {
      ++sample_joints_;
    }
  }
}

void IdentifyPanel::onCurrents(const JointCurrents::ConstSharedPtr msg)
{
  for (size_t i = 0; i < msg->actual.size(); ++i) {
    if (!std::isfinite(msg->actual[i]) || !std::isfinite(msg->target[i])) {
      return;
    }
  }
  if (phase_ != Phase::Idle &&
    std::abs((node_->now() - rclcpp::Time(msg->header.stamp)).seconds()) > 1.0)
  {
    return;
  }
  currents_received_ = Clock::now();
  for (size_t i = 0; i < msg->actual.size(); ++i) {
    actual_[i]->setText(QString::number(msg->actual[i], 'f', 4));
    target_[i]->setText(QString::number(msg->target[i], 'f', 4));
  }
  if (recordMessage(*msg, currents_sub_->get_topic_name())) {
    ++current_rows_;
    if (phase_ == Phase::Sampling &&
      rclcpp::Time(msg->header.stamp).nanoseconds() >= sample_start_ns_)
    {
      ++sample_currents_;
    }
  }
}

bool IdentifyPanel::startRecording(const QString & path, bool overwrite)
{
  if (recording_) {
    return false;
  }
  const QFileInfo output(path);
  for (const auto & filename : {path, snapshotPath(path)}) {
    const QFileInfo file(filename);
    if (file.isSymLink() || (file.exists() && (!file.isFile() || !overwrite))) {
      recording_status_->setText("Cannot replace output: " + filename);
      return false;
    }
  }
  // Prepare both files before moving; keep any previous pair until the run ends.
  recording_directory_ = std::make_unique<QTemporaryDir>(
    output.absoluteDir().filePath(".param_identify-XXXXXX"));
  if (!recording_directory_->isValid()) {
    recording_status_->setText("Cannot create MCAP output: " + path);
    recording_directory_.reset();
    return false;
  }
  recording_path_ = output.absoluteFilePath();
  try {
    measurement_windows_.clear();
    recording_started_ns_ = node_->now().nanoseconds();
    last_record_ns_ = recording_started_ns_;
    saveSnapshot("Recording started.");
    auto writer = std::make_unique<rosbag2_cpp::Writer>();
    rosbag2_storage::StorageOptions options;
    options.uri = recording_directory_->filePath("bag").toStdString();
    options.storage_id = "mcap";
    writer->open(options);
    RobotMode mode;
    mode.robot_mode = mode_;
    const auto stamp = node_->now();
    writer->write(mode, mode_sub_->get_topic_name(), stamp);
    last_record_ns_ = stamp.nanoseconds();
    recording_ = std::move(writer);
  } catch (const std::exception & e) {
    recording_directory_.reset();
    recording_status_->setText(QString("Cannot record MCAP: %1").arg(e.what()));
    return false;
  }
  joint_rows_ = current_rows_ = 0;
  refreshControls();
  return true;
}

template<typename Message>
bool IdentifyPanel::recordMessage(const Message & message, const std::string & topic)
{
  if (!recording_) {
    return false;
  }
  try {
    // Bag time is receipt time. Original source time remains in message.header.
    const auto stamp = node_->now();
    if (stamp.nanoseconds() < last_record_ns_) {
      throw std::runtime_error("ROS clock went backwards during recording");
    }
    recording_->write(message, topic, stamp);
    last_record_ns_ = stamp.nanoseconds();
    return true;
  } catch (const std::exception & e) {
    recordingFailed(QString::fromUtf8(e.what()));
    return false;
  }
}

void IdentifyPanel::recordingFailed(const QString & error)
{
  const auto reason = "MCAP write failed: " + error;
  endRun(reason, phase_ != Phase::Idle);
}

void IdentifyPanel::saveSnapshot(const QString & reason)
{
  auto root = YAML::Load(config_yaml_.constData());
  root.remove("recording");
  auto recording = root["recording"];
  recording["started_at"] = yamlTime(recording_started_ns_);
  recording["ended_at"] = yamlTime(std::max(last_record_ns_, node_->now().nanoseconds()));
  recording["result"] = reason.toStdString();
  recording["topics"]["joint_states"] = joints_sub_->get_topic_name();
  recording["topics"]["joint_currents"] = currents_sub_->get_topic_name();
  recording["topics"]["robot_mode"] = mode_sub_->get_topic_name();
  recording["enabled_payload"]["load_kg"] = enabled_payload_.load;
  recording["enabled_payload"]["center_x_mm"] = enabled_payload_.center_x;
  recording["enabled_payload"]["center_y_mm"] = enabled_payload_.center_y;
  recording["enabled_payload"]["center_z_mm"] = enabled_payload_.center_z;
  recording["windows"] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto & window : measurement_windows_) {
    YAML::Node entry;
    entry["segment_id"] = window.segment;
    entry["kind"] = window.pose.kind.toStdString();
    for (const auto angle : window.pose.radians) {
      entry["goal_rad"].push_back(angle);
    }
    entry["start"] = yamlTime(window.start_ns);
    entry["end"] = yamlTime(window.end_ns);
    entry["complete"] = window.complete;
    recording["windows"].push_back(entry);
  }
  YAML::Emitter output;
  output << root;
  QSaveFile file(recording_directory_->filePath(QFileInfo(snapshotPath(recording_path_)).fileName()));
  if (!output.good() || !file.open(QIODevice::WriteOnly) ||
    file.write(
      output.c_str(),
      output.size()) != static_cast<qint64>(output.size()) || !file.commit())
  {
    throw std::runtime_error("cannot save recording YAML");
  }
}

void IdentifyPanel::stopRecording(const QString & reason)
{
  if (!recording_) {
    return;
  }
  // Move ownership first: failures here must never recursively stop the writer.
  auto writer = std::move(recording_);
  try {
    if (!measurement_windows_.empty() && !measurement_windows_.back().complete) {
      measurement_windows_.back().end_ns = std::max(
        measurement_windows_.back().start_ns,
        std::max(last_record_ns_, node_->now().nanoseconds()));
    }
    writer->close();
    writer.reset();
    saveSnapshot(reason);
    const QDir bag(recording_directory_->filePath("bag"));
    const auto files = bag.entryList({"*.mcap"}, QDir::Files);
    if (files.size() != 1) {
      throw std::runtime_error("expected one MCAP file");
    }
    renameFile(
      bag.filePath(files[0]),
      recording_directory_->filePath(QFileInfo(recording_path_).fileName()));
    saveRecordingPair(recording_directory_->path(), recording_path_);
    recording_directory_.reset();
    recording_status_->setText(
      QString("Saved %1 joint messages and %2 current messages to %3 and %4.")
      .arg(joint_rows_).arg(current_rows_).arg(recording_path_).arg(snapshotPath(recording_path_)));
  } catch (const std::exception & e) {
    recording_directory_->setAutoRemove(false);
    recording_status_->setText(
      QString("MCAP save failed: %1. Recording retained at %2")
      .arg(e.what()).arg(recording_directory_->path()));
    recording_directory_.reset();
  }
  refreshControls();
}

void IdentifyPanel::refreshControls()
{
  QString mode;
  switch (mode_) {
    case RobotMode::INIT: mode = "INIT"; break;
    case RobotMode::DISABLED: mode = "DISABLED"; break;
    case RobotMode::ENABLE: mode = "ENABLE"; break;
    case RobotMode::RUNNING: mode = "RUNNING"; break;
    case RobotMode::ERROR: mode = "ERROR"; break;
    default: mode = QString::number(mode_); break;
  }
  const bool fresh_mode = Clock::now() - mode_received_ < std::chrono::seconds(1);
  mode_label_->setText("Robot Mode: " + mode + (node_ && !fresh_mode ? " (stale)" : ""));
  const bool idle = phase_ == Phase::Idle && !recording_ && !service_pending_;
  enable_->setEnabled(
    idle && fresh_mode && mode_ == RobotMode::DISABLED && settings_loaded_ &&
    enable_client_ && enable_client_->service_is_ready());
  disable_->setEnabled(
    !service_pending_ && disable_client_ && disable_client_->service_is_ready() &&
    (phase_ != Phase::Idle || mode_ == RobotMode::ENABLE || mode_ == RobotMode::RUNNING));
  clear_->setEnabled(
    idle && fresh_mode && mode_ != RobotMode::RUNNING &&
    clear_client_ && clear_client_->service_is_ready());
  const bool have_joints = Clock::now() - joints_received_ < std::chrono::seconds(1);
  const bool have_currents = Clock::now() - currents_received_ < std::chrono::seconds(1);
  telemetry_status_->setText(
    QString("Joints: %1. Currents: %2.")
    .arg(have_joints ? "receiving" : "missing / stale")
    .arg(have_currents ? "receiving" : "missing / stale (publish_joint_currents:=true)"));
  for (size_t i = 0; i < angles_.size(); ++i) {
    if (!have_joints) {
      angles_[i]->setText("--");
    }
    if (!have_currents) {
      actual_[i]->setText("--");
      target_[i]->setText("--");
    }
  }
  load_config_->setEnabled(
    idle && (mode_ == RobotMode::INIT || mode_ == RobotMode::DISABLED));
  run_->setEnabled(idle && settings_loaded_ && robotReady() && telemetryReady());
  run_->setToolTip("Load experiment YAML, Enable with its payload, then run the reviewed poses.");
  if (recording_) {
    recording_status_->setText(
      QString("Recording: %1 joint messages, %2 current messages.")
      .arg(joint_rows_).arg(current_rows_));
  }
}
}  // namespace mg400_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mg400_rviz_plugin::IdentifyPanel, rviz_common::Panel)
