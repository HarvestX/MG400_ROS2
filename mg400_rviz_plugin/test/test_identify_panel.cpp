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

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <thread>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>
#include <mg400_interface/joint_handler.hpp>
#include <pluginlib/class_loader.hpp>
#include <QTemporaryDir>
#include <rosbag2_cpp/reader.hpp>
#include <yaml-cpp/yaml.h>

#include "mg400_rviz_plugin/panel_identify.hpp"
#include "mg400_rviz_plugin/panel_mg400_controller.hpp"

using namespace std::chrono_literals;  // NOLINT
using Currents = mg400_msgs::msg::JointCurrents;
using JointState = sensor_msgs::msg::JointState;
using Enable = mg400_msgs::srv::EnableRobot;
using Disable = mg400_msgs::srv::DisableRobot;
using RobotMode = mg400_msgs::msg::RobotMode;
using Action = mg400_msgs::action::JointMovJ;
using ServerGoal = rclcpp_action::ServerGoalHandle<Action>;

static_assert(
  !std::is_base_of<mg400_rviz_plugin::Mg400ControllerPanel,
  mg400_rviz_plugin::IdentifyPanel>::value, "Identify must be independent of Controller");

class IdentifyTestPanel : public mg400_rviz_plugin::IdentifyPanel
{
public:
  using IdentifyPanel::initializeRos;
  using IdentifyPanel::startSequence;
  using IdentifyPanel::loadConfig;
};

class IdentifyPanelTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("identify_panel_test");
    executor_.add_node(node_);
    panel_ = std::make_unique<IdentifyTestPanel>();
    panel_->initializeRos(node_);
    joints_ = node_->create_publisher<JointState>("/mg400/joint_states", 10);
    currents_ = node_->create_publisher<Currents>("/mg400/joint_currents", 10);
    mode_ = node_->create_publisher<RobotMode>("/mg400/robot_mode", 1);
    enable_ = node_->create_service<Enable>(
      "/mg400/enable_robot",
      [this](Enable::Request::SharedPtr request, Enable::Response::SharedPtr response) {
        last_request_ = *request;
        ++enable_count_;
        response->result = true;
        mode_value_ = RobotMode::ENABLE;
      });
    disable_ = node_->create_service<Disable>(
      "/mg400/disable_robot",
      [this](Disable::Request::SharedPtr, Disable::Response::SharedPtr response) {
        ++disable_count_;
        response->result = true;
        mode_value_ = RobotMode::DISABLED;
      });
    server_ = rclcpp_action::create_server<Action>(
      node_, "/mg400/joint_mov_j",
      [this](const rclcpp_action::GoalUUID &, Action::Goal::ConstSharedPtr goal) {
        goals_.push_back(*goal);
        return reject_ ? rclcpp_action::GoalResponse::REJECT :
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<ServerGoal>) {return rclcpp_action::CancelResponse::REJECT;},
      [this](std::shared_ptr<ServerGoal> handle) {
        handle_ = handle;
        angles_ = handle->get_goal()->joint_angles;
      });
    ASSERT_TRUE(
      waitFor(
        [this]() {
          return joints_->get_subscription_count() == 1 &&
          currents_->get_subscription_count() == 1 && mode_->get_subscription_count() == 1;
        }));
  }

  void TearDown() override
  {
    panel_.reset();
    if (node_) {
      executor_.remove_node(node_);
    }
  }

  bool waitFor(const std::function<bool()> & predicate)
  {
    const auto deadline = std::chrono::steady_clock::now() + 10s;
    auto published = std::chrono::steady_clock::time_point{};
    do {
      if (stream_ && std::chrono::steady_clock::now() - published >= 10ms) {
        publishTelemetry();
        RobotMode mode;
        mode.robot_mode = mode_value_;
        mode_->publish(mode);
        published = std::chrono::steady_clock::now();
      }
      executor_.spin_some();
      if (handle_ && !hold_) {
        auto result = std::make_shared<Action::Result>();
        result->result = !fail_;
        handle_->succeed(result);
        handle_.reset();
      }
      QApplication::processEvents();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
  }

  void enableForRun()
  {
    if (panel_->findChild<QTableWidget *>("identify_poses")->rowCount() == 0) {
      ASSERT_TRUE(panel_->loadConfig(configFile()));
    }
    stream_ = true;
    auto * enable = panel_->findChild<QPushButton *>("enable_robot");
    ASSERT_TRUE(waitFor([&]() {return enable->isEnabled();}));
    enable->click();
    ASSERT_TRUE(
      waitFor(
        [this]() {
          return panel_->findChild<QPushButton *>("run_identify")->isEnabled();
        }));
  }

  void publishTelemetry()
  {
    auto joints = mg400_interface::JointHandler::getJointState(angles_, "test_");
    joints->header.stamp = node_->now();
    last_joint_stamp_ = joints->header.stamp;
    std::reverse(joints->name.begin(), joints->name.end());
    std::reverse(joints->position.begin(), joints->position.end());
    joints_->publish(*joints);
    if (!publish_currents_) {
      return;
    }
    Currents currents;
    currents.header.stamp = node_->now();
    last_current_stamp_ = currents.header.stamp;
    currents.actual = {0.5, -0.6, 0.7, -0.8};
    currents.target = {0.1, -0.2, 0.3, -0.4};
    currents_->publish(currents);
  }

  template<typename Message>
  std::vector<Message> readMessages(const QString & path, const std::string & topic)
  {
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions options;
    options.uri = path.toStdString();
    options.storage_id = "mcap";
    reader.open(options);
    std::vector<Message> messages;
    while (reader.has_next()) {
      const auto record = reader.read_next();
      if (record->topic_name == topic) {
        rclcpp::SerializedMessage data(*record->serialized_data);
        Message msg;
        rclcpp::Serialization<Message>().deserialize_message(&data, &msg);
        messages.push_back(msg);
      }
    }
    return messages;
  }

  YAML::Node readSnapshot(const QString & path)
  {
    return YAML::LoadFile(yamlPath(path).toStdString());
  }

  YAML::Node readWindows(const QString & path)
  {
    return readSnapshot(path)["recording"]["windows"];
  }

  std::string recordedSettings(const QString & path)
  {
    auto root = readSnapshot(path);
    root.remove("recording");
    return YAML::Dump(root);
  }

  QString yamlPath(const QString & path)
  {
    const QFileInfo file(path);
    return file.absoluteDir().filePath(file.completeBaseName() + ".yaml");
  }

  YAML::Node settings(const std::string & poses = "[{kind: base, joints_deg: [0,45,45,0]}]")
  {
    auto root = YAML::Load(
      R"(
schema_version: 1
payload: {load_kg: 0.2, center_x_mm: 10, center_y_mm: -20, center_z_mm: 30}
motion: {speed_percent: 7, acceleration_percent: 9}
measurement: {settle_sec: 0.1, record_sec: 1.2, repeats: 1}
identify:
  torque_constants_nm_per_a: [3.4, 3.3, 5.6, 0.75]
  torque_signs: [-1, 1, 1, 1]
  joint_signs: [1, 1, 1, 1]
  reference_deg: [0, 45, 45, 0]
  min_duration_sec: 1.0
  max_span_deg: 0.5
)");
    root["poses"] = YAML::Load(poses);
    return root;
  }

  QString configFile(const YAML::Node & root = YAML::Node())
  {
    YAML::Emitter output;
    output << (root.IsNull() ? settings() : root);
    const auto path = directory_.filePath("experiment.yaml");
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
      return "";
    }
    file.write(output.c_str());
    return path;
  }

  QByteArray readBytes(const QString & path)
  {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
      return {};
    }
    return file.readAll();
  }

  bool startWithOverwriteAnswer(const QString & path, QMessageBox::StandardButton answer)
  {
    bool prompted = false;
    QTimer reply;
    QObject::connect(
      &reply, &QTimer::timeout, [&]() {
        auto * question = qobject_cast<QMessageBox *>(QApplication::activeModalWidget());
        if (question) {
          prompted = true;
          EXPECT_EQ("Replace recording?", question->windowTitle());
          EXPECT_TRUE(question->text().contains(path) || question->text().contains(yamlPath(path)));
          EXPECT_EQ(question->button(QMessageBox::Cancel), question->defaultButton());
          reply.stop();
          question->button(answer)->click();
        }
      });
    reply.start(10);
    const bool started = panel_->startSequence(path);
    EXPECT_TRUE(prompted);
    return started;
  }

  QTemporaryDir directory_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::unique_ptr<IdentifyTestPanel> panel_;
  rclcpp::Publisher<JointState>::SharedPtr joints_;
  rclcpp::Publisher<Currents>::SharedPtr currents_;
  rclcpp::Publisher<RobotMode>::SharedPtr mode_;
  rclcpp::Service<Enable>::SharedPtr enable_;
  rclcpp::Service<Disable>::SharedPtr disable_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  std::shared_ptr<ServerGoal> handle_;
  std::vector<Action::Goal> goals_;
  Enable::Request last_request_;
  builtin_interfaces::msg::Time last_joint_stamp_;
  builtin_interfaces::msg::Time last_current_stamp_;
  size_t enable_count_ = 0;
  size_t disable_count_ = 0;
  bool stream_ = false;
  bool hold_ = false;
  bool reject_ = false;
  bool fail_ = false;
  bool publish_currents_ = true;
  RobotMode::_robot_mode_type mode_value_ = RobotMode::DISABLED;
  std::array<double, 4> angles_ = {0.1, 0.2, 0.3, 0.4};
};

TEST_F(IdentifyPanelTest, RequiresAllPayloadFieldsAndOwnEnable)
{
  stream_ = true;
  rviz_common::Config legacy;
  legacy.mapSetValue("Set Enable Payload", false);
  panel_->load(legacy);
  EXPECT_EQ(nullptr, panel_->findChild<QCheckBox *>("set_enable_payload"));
  ASSERT_TRUE(
    waitFor(
      [this]() {
        return panel_->findChild<QLabel *>("robot_mode")->text() == "Robot Mode: DISABLED";
      }));
  auto * enable = panel_->findChild<QPushButton *>("enable_robot");
  EXPECT_FALSE(enable->isEnabled());
  auto invalid = settings();
  invalid["payload"]["load_kg"] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(panel_->loadConfig(configFile(invalid)));
  EXPECT_FALSE(panel_->startSequence(directory_.filePath("bad.mcap")));
  EXPECT_EQ(0u, goals_.size());
  ASSERT_TRUE(panel_->loadConfig(configFile()));
  mode_value_ = RobotMode::ENABLE;  // An external Enable cannot confirm this panel's payload.
  ASSERT_TRUE(
    waitFor(
      [this]() {
        return panel_->findChild<QLabel *>("robot_mode")->text() == "Robot Mode: ENABLE";
      }));
  EXPECT_FALSE(panel_->findChild<QPushButton *>("run_identify")->isEnabled());
  mode_value_ = RobotMode::DISABLED;
  enableForRun();
  EXPECT_EQ(Enable::Request::FOUR_PARAM, last_request_.num_of_params);
  EXPECT_DOUBLE_EQ(0.2, last_request_.load);
  EXPECT_DOUBLE_EQ(10.0, last_request_.center_x);
  EXPECT_DOUBLE_EQ(-20.0, last_request_.center_y);
  EXPECT_DOUBLE_EQ(30.0, last_request_.center_z);
  EXPECT_EQ(1u, enable_count_);
}

TEST_F(IdentifyPanelTest, RecordsIndependentTimestampsAndRawPhysicalAxes)
{
  enableForRun();
  stream_ = false;
  hold_ = true;
  const auto path = directory_.filePath("measurement.mcap");
  ASSERT_TRUE(panel_->startSequence(path));
  EXPECT_FALSE(panel_->startSequence(directory_.filePath("second.mcap")));
  publishTelemetry();
  const auto processed_after = std::chrono::steady_clock::now() + 60ms;
  ASSERT_TRUE(waitFor([&]() {return std::chrono::steady_clock::now() >= processed_after;}));
  EXPECT_EQ("17.189", panel_->findChild<QLabel *>("identify_angle_j3")->text());
  EXPECT_EQ("-0.8000", panel_->findChild<QLabel *>("identify_actual_j4")->text());
  panel_->findChild<QPushButton *>("disable_robot")->click();
  EXPECT_TRUE(readBytes(path).startsWith(QByteArray::fromHex("894d434150300d0a")));
  const auto recording = readSnapshot(path)["recording"];
  EXPECT_EQ(0u, recording["windows"].size());
  const auto payload = recording["enabled_payload"];
  EXPECT_DOUBLE_EQ(0.2, payload["load_kg"].as<double>());
  EXPECT_DOUBLE_EQ(10, payload["center_x_mm"].as<double>());
  EXPECT_DOUBLE_EQ(-20, payload["center_y_mm"].as<double>());
  EXPECT_DOUBLE_EQ(30, payload["center_z_mm"].as<double>());
  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions options;
  options.uri = path.toStdString();
  options.storage_id = "mcap";
  reader.open(options);
  EXPECT_EQ(3u, reader.get_all_topics_and_types().size());
  EXPECT_EQ("/mg400/joint_states", recording["topics"]["joint_states"].as<std::string>());
  const auto joints = readMessages<JointState>(path, "/mg400/joint_states");
  ASSERT_FALSE(joints.empty());
  EXPECT_EQ(last_joint_stamp_, joints.back().header.stamp);
  const auto expected = mg400_interface::JointHandler::getJointState({0.1, 0.2, 0.3, 0.4}, "test_");
  std::reverse(expected->name.begin(), expected->name.end());
  std::reverse(expected->position.begin(), expected->position.end());
  EXPECT_EQ(expected->name, joints.back().name);
  EXPECT_EQ(expected->position, joints.back().position);
  const auto currents = readMessages<Currents>(path, "/mg400/joint_currents");
  ASSERT_FALSE(currents.empty());
  EXPECT_EQ(last_current_stamp_, currents.back().header.stamp);
  EXPECT_DOUBLE_EQ(-0.6, currents.back().actual[1]);
  EXPECT_DOUBLE_EQ(-0.4, currents.back().target[3]);
  EXPECT_FALSE(readMessages<RobotMode>(path, "/mg400/robot_mode").empty());
}

TEST_F(IdentifyPanelTest, RunsPosesWithSettlingSamplingTransitAndRepeats)
{
  auto root = settings(
    "[{kind: base, joints_deg: [0,45,45,0]}, "
    "{kind: move, joints_deg: [5,40,40,0]}, {kind: train, joints_deg: [10,40,50,15]}]");
  root["measurement"]["repeats"] = 2;
  const auto yaml_path = configFile(root);
  ASSERT_TRUE(panel_->loadConfig(yaml_path));
  const auto original_yaml = readBytes(yaml_path);
  enableForRun();
  // Editing the source file cannot change an already loaded experiment.
  root["motion"]["speed_percent"] = 99;
  configFile(root);
  const auto path = directory_.filePath("run.mcap");
  ASSERT_TRUE(panel_->startSequence(path));
  EXPECT_TRUE(panel_->findChild<QTableWidget *>("identify_poses")->isEnabled());
  EXPECT_FALSE(panel_->findChild<QPushButton *>("load_config")->isEnabled());
  EXPECT_EQ(nullptr, panel_->findChild<QSpinBox *>("identify_speed"));
  EXPECT_TRUE(panel_->findChild<QPushButton *>("disable_robot")->isEnabled());
  EXPECT_FALSE(panel_->loadConfig(yaml_path));
  ASSERT_TRUE(
    waitFor(
      [this]() {
        return panel_->findChild<QLabel *>("run_status")->text() ==
        "Measurement sequence completed.";
      })) << panel_->findChild<QLabel *>("run_status")->text().toStdString() << " / " <<
    panel_->findChild<QLabel *>("telemetry_status")->text().toStdString() << " / goals " <<
    goals_.size() << " / " << panel_->findChild<QLabel *>("recording_status")->text().toStdString();
  ASSERT_EQ(6u, goals_.size());
  EXPECT_EQ(0u, disable_count_);
  for (const auto & goal : goals_) {
    EXPECT_TRUE(goal.set_speed_j);
    EXPECT_EQ(7, goal.speed_j);
    EXPECT_TRUE(goal.set_acc_j);
    EXPECT_EQ(9, goal.acc_j);
    EXPECT_TRUE(goal.set_cp);
    EXPECT_EQ(0, goal.cp);
  }
  EXPECT_NEAR(M_PI / 18, goals_[2].joint_angles[0], 1e-10);
  EXPECT_EQ(YAML::Dump(YAML::Load(original_yaml.constData())), recordedSettings(path));
  const auto windows = readWindows(path);
  ASSERT_EQ(4u, windows.size());
  const std::array<int, 4> segments = {0, 2, 3, 5};
  for (size_t i = 0; i < windows.size(); ++i) {
    const auto window = windows[i];
    EXPECT_TRUE(window["complete"].as<bool>());
    EXPECT_EQ(segments[i], window["segment_id"].as<int>());
    EXPECT_NE("move", window["kind"].as<std::string>());
    const auto stamp = [](const YAML::Node & node) {
        return node["sec"].as<int64_t>() * 1000000000 + node["nanosec"].as<int64_t>();
      };
    EXPECT_GT(stamp(window["end"]), stamp(window["start"]));
  }
  QProcess reader;
  reader.start(
    "python3", {"-c",
      "import sys; sys.path.insert(0, sys.argv[1]); "
      "from parameter_identifier import read_recording; "
      "samples, skipped = read_recording(sys.argv[2], [1]*4); "
      "assert len(samples) == 4 and not skipped, (samples, skipped)",
      IDENTIFIER_PATH, path});
  ASSERT_TRUE(reader.waitForFinished(10000));
  EXPECT_EQ(0, reader.exitCode()) << reader.readAllStandardError().toStdString();
}

TEST_F(IdentifyPanelTest, StopDisablesAndLateResultDoesNotAdvance)
{
  ASSERT_TRUE(
    panel_->loadConfig(
      configFile(
        settings(
          "[{kind: base, joints_deg: [0,45,45,0]}, {kind: train, joints_deg: [10,40,50,15]}]"))));
  enableForRun();
  hold_ = true;
  const auto path = directory_.filePath("stopped.mcap");
  ASSERT_TRUE(panel_->startSequence(path));
  ASSERT_TRUE(waitFor([this]() {return handle_ != nullptr;}));
  panel_->findChild<QPushButton *>("disable_robot")->click();
  ASSERT_TRUE(waitFor([this]() {return disable_count_ >= 1;}));
  hold_ = false;  // Deliver the old success after Stop.
  ASSERT_TRUE(
    waitFor(
      [this]() {
        return panel_->findChild<QLabel *>("service_status")->text() == "Disable succeeded.";
      }));
  EXPECT_EQ(1u, goals_.size());
  EXPECT_FALSE(panel_->findChild<QPushButton *>("run_identify")->isEnabled());
  for (const auto & window : readWindows(path)) {
    EXPECT_FALSE(window["complete"].as<bool>());
  }
}

TEST_F(IdentifyPanelTest, RejectsMotionAndAbortsOnActionFailure)
{
  enableForRun();
  reject_ = true;
  ASSERT_TRUE(panel_->startSequence(directory_.filePath("rejected.mcap")));
  ASSERT_TRUE(
    waitFor(
      [this]() {
        return panel_->findChild<QLabel *>("run_status")->text() == "Motion rejected.";
      }));
  EXPECT_EQ(1u, goals_.size());
  EXPECT_EQ(0u, disable_count_);
  reject_ = false;
  fail_ = true;
  ASSERT_TRUE(panel_->startSequence(directory_.filePath("failed.mcap")));
  ASSERT_TRUE(waitFor([this]() {return disable_count_ == 1;}));
  EXPECT_EQ(2u, goals_.size());
  EXPECT_TRUE(panel_->findChild<QLabel *>("run_status")->text().contains("Motion failed"));
}

TEST_F(IdentifyPanelTest, TelemetryLossAbortsAndDisables)
{
  enableForRun();
  hold_ = true;
  ASSERT_TRUE(panel_->startSequence(directory_.filePath("lost.mcap")));
  ASSERT_TRUE(waitFor([this]() {return handle_ != nullptr;}));
  publish_currents_ = false;
  ASSERT_TRUE(waitFor([this]() {return disable_count_ == 1;}));
  EXPECT_TRUE(panel_->findChild<QLabel *>("run_status")->text().contains("Telemetry lost"));
  EXPECT_EQ(1u, goals_.size());
}

TEST_F(IdentifyPanelTest, MovementDuringSamplingExcludesIncompleteSegment)
{
  enableForRun();
  const auto path = directory_.filePath("moved.mcap");
  ASSERT_TRUE(panel_->startSequence(path));
  ASSERT_TRUE(
    waitFor(
      [this]() {
        return panel_->findChild<QLabel *>("run_status")->text().startsWith("Recording pose");
      }));
  angles_[0] += 0.1;
  ASSERT_TRUE(waitFor([this]() {return disable_count_ == 1;}));
  EXPECT_TRUE(panel_->findChild<QLabel *>("run_status")->text().contains("Motion detected"));
  for (const auto & window : readWindows(path)) {
    EXPECT_FALSE(window["complete"].as<bool>());
  }
}

TEST_F(IdentifyPanelTest, InvalidPlanAndOutputNeverSendMotion)
{
  for (const auto * pose : {"{kind: train, joints_deg: [.nan,45,45,0]}",
      "{kind: train, joints_deg: [180,45,45,0]}", "{kind: train, joints_deg: [0,-10,100,0]}",
      "{kind: unknown, joints_deg: [0,45,45,0]}", "{kind: train, joints_deg: [0,45,45]}"})
  {
    EXPECT_FALSE(panel_->loadConfig(configFile(settings(std::string("[") + pose + "]"))));
    EXPECT_EQ(0, panel_->findChild<QTableWidget *>("identify_poses")->rowCount());
  }
  enableForRun();
  EXPECT_FALSE(panel_->startSequence(directory_.filePath("missing/run.mcap")));
  EXPECT_TRUE(panel_->findChild<QLabel *>("run_status")->text().startsWith("Run not started:"));
  const auto path = directory_.filePath("existing.mcap");
  QFile existing(path);
  ASSERT_TRUE(existing.open(QIODevice::WriteOnly));
  existing.write("preserve this");
  existing.close();
  EXPECT_FALSE(startWithOverwriteAnswer(path, QMessageBox::Cancel));
  EXPECT_EQ(QByteArray("preserve this"), readBytes(path));
  const auto link = directory_.filePath("symlink.mcap");
  ASSERT_TRUE(QFile::link(path, link));
  EXPECT_FALSE(startWithOverwriteAnswer(link, QMessageBox::Yes));
  EXPECT_EQ(QByteArray("preserve this"), readBytes(path));
  EXPECT_EQ(0u, goals_.size());
}

TEST_F(IdentifyPanelTest, ExistingYamlRequiresOverwriteConfirmation)
{
  enableForRun();
  const auto path = directory_.filePath("yaml_only.mcap");
  QFile snapshot(yamlPath(path));
  ASSERT_TRUE(snapshot.open(QIODevice::WriteOnly));
  snapshot.write("previous settings");
  snapshot.close();
  EXPECT_FALSE(startWithOverwriteAnswer(path, QMessageBox::Cancel));
  EXPECT_TRUE(goals_.empty());
  EXPECT_FALSE(QFileInfo::exists(path));
  EXPECT_EQ(QByteArray("previous settings"), readBytes(yamlPath(path)));
  const auto link = directory_.filePath("linked.yaml");
  ASSERT_TRUE(QFile::link(yamlPath(path), link));
  EXPECT_FALSE(startWithOverwriteAnswer(directory_.filePath("linked.mcap"), QMessageBox::Yes));
  EXPECT_TRUE(goals_.empty());
  EXPECT_EQ(QByteArray("previous settings"), readBytes(yamlPath(path)));
}

TEST_F(IdentifyPanelTest, ConfirmedOverwritePreservesAndReplacesMcapAndYaml)
{
  enableForRun();
  const auto expected_yaml = readBytes(directory_.filePath("experiment.yaml"));
  const auto path = directory_.filePath("replace.mcap");
  QFile file(path);
  ASSERT_TRUE(file.open(QIODevice::WriteOnly));
  file.write("previous recording");
  file.close();
  QFile snapshot(yamlPath(path));
  ASSERT_TRUE(snapshot.open(QIODevice::WriteOnly));
  snapshot.write("previous settings");
  snapshot.close();
  EXPECT_FALSE(startWithOverwriteAnswer(path, QMessageBox::Cancel));
  EXPECT_EQ(QByteArray("previous recording"), readBytes(path));
  EXPECT_EQ(QByteArray("previous settings"), readBytes(yamlPath(path)));
  ASSERT_TRUE(startWithOverwriteAnswer(path, QMessageBox::Yes));
  EXPECT_EQ(QByteArray("previous recording"), readBytes(path));
  EXPECT_EQ(QByteArray("previous settings"), readBytes(yamlPath(path)));
  ASSERT_TRUE(
    waitFor(
      [this]() {
        return panel_->findChild<QLabel *>("run_status")->text() ==
        "Measurement sequence completed.";
      }));
  EXPECT_EQ(YAML::Dump(YAML::Load(expected_yaml.constData())), recordedSettings(path));
  const auto windows = readWindows(path);
  ASSERT_EQ(1u, windows.size());
  EXPECT_TRUE(windows[0]["complete"].as<bool>());
  EXPECT_EQ("base", windows[0]["kind"].as<std::string>());
  EXPECT_EQ(1u, goals_.size());
}

TEST_F(IdentifyPanelTest, FinalizationFailureRetainsRecordingForRecovery)
{
  enableForRun();
  hold_ = true;
  const auto path = directory_.filePath("blocked.mcap");
  ASSERT_TRUE(panel_->startSequence(path));
  ASSERT_TRUE(waitFor([this]() {return handle_ != nullptr;}));
  ASSERT_TRUE(QDir().mkdir(path));  // Simulate the destination changing during the run.
  panel_->findChild<QPushButton *>("disable_robot")->click();
  ASSERT_TRUE(waitFor([this]() {return disable_count_ == 1;}));
  const auto status = panel_->findChild<QLabel *>("recording_status")->text();
  EXPECT_TRUE(status.startsWith("MCAP save failed:"));
  const auto staged = QDir(directory_.path()).entryList(
    {".identify-*"}, QDir::Dirs | QDir::Hidden | QDir::NoDotAndDotDot);
  ASSERT_EQ(1, staged.size());
  const QDir bag(directory_.filePath(staged[0]));
  const auto files = bag.entryList({"*.mcap"}, QDir::Files);
  ASSERT_EQ(1, files.size());
  EXPECT_FALSE(readMessages<RobotMode>(bag.filePath(files[0]), "/mg400/robot_mode").empty());
  EXPECT_EQ(
    YAML::Dump(YAML::Load(readBytes(directory_.filePath("experiment.yaml")).constData())),
    recordedSettings(bag.filePath(files[0])));
  EXPECT_TRUE(status.contains(directory_.filePath(staged[0])));
}

TEST_F(IdentifyPanelTest, ValidatesAllSettingsAndKeepsPreviousValidConfig)
{
  auto root = settings();
  ASSERT_TRUE(panel_->loadConfig(configFile(root)));
  for (const auto * payload : {"{load_kg: 0.2, center_x: 10, center_y: -20, center_z: 30}",
      "{load_kg: 0.2, center_x_mm: 10, center_y_m: -0.02, center_z_mm: 30}",
      "{load_kg: 0.751, center_x_mm: 10, center_y_mm: -20, center_z_mm: 30}",
      "{load_kg: 0.2, center_x_mm: 10, center_y_mm: -20, center_z_mm: 501}",
      "{load_kg: 0.20001, center_x_mm: 10, center_y_mm: -20, center_z_mm: 30}"})
  {
    auto bad = settings();
    bad["payload"] = YAML::Load(payload);
    EXPECT_FALSE(panel_->loadConfig(configFile(bad)));
  }
  for (const auto * value : {"101", "0", "2.5", ".nan"}) {
    auto bad = settings();
    bad["motion"]["speed_percent"] = YAML::Load(value);
    EXPECT_FALSE(panel_->loadConfig(configFile(bad)));
  }
  auto bad = settings();
  bad["measurement"]["record_sec"] = 0.5;
  EXPECT_FALSE(panel_->loadConfig(configFile(bad)));
  bad = settings();
  bad["identify"]["min_duration_sec"] = 1.2;
  EXPECT_FALSE(panel_->loadConfig(configFile(bad)));
  bad = settings();
  bad["identify"]["torque_constants_nm_per_a"][0] = 0;
  EXPECT_FALSE(panel_->loadConfig(configFile(bad)));
  const auto path = configFile(root);
  QFile file(path);
  ASSERT_TRUE(file.open(QIODevice::Append));
  file.write("\nschema_version: 1\n");
  file.close();
  EXPECT_FALSE(panel_->loadConfig(path));
  EXPECT_EQ("0.2", panel_->findChild<QLabel *>("enable_load")->text());
  EXPECT_EQ(1, panel_->findChild<QTableWidget *>("identify_poses")->rowCount());
  EXPECT_TRUE(goals_.empty());
  EXPECT_EQ(0u, enable_count_);
}

TEST_F(IdentifyPanelTest, ConvertsExplicitMetersToEnableMillimetersAndRequiresReloadWhileDisabled)
{
  auto root = settings();
  root["payload"] = YAML::Load(
    "{load_kg: 0.1, center_x_m: 0.045, center_y_m: 0.012, center_z_m: 0.01}");
  ASSERT_TRUE(panel_->loadConfig(configFile(root)));
  enableForRun();
  EXPECT_DOUBLE_EQ(0.1, last_request_.load);
  EXPECT_DOUBLE_EQ(45, last_request_.center_x);
  EXPECT_DOUBLE_EQ(12, last_request_.center_y);
  EXPECT_DOUBLE_EQ(10, last_request_.center_z);
  EXPECT_FALSE(panel_->loadConfig(configFile()));
  panel_->findChild<QPushButton *>("disable_robot")->click();
  ASSERT_TRUE(
    waitFor(
      [this]() {
        return panel_->findChild<QPushButton *>("load_config")->isEnabled();
      }));
  ASSERT_TRUE(panel_->loadConfig(configFile()));
  EXPECT_FALSE(panel_->findChild<QPushButton *>("run_identify")->isEnabled());
  enableForRun();
  EXPECT_DOUBLE_EQ(0.2, last_request_.load);
  EXPECT_DOUBLE_EQ(10, last_request_.center_x);
  EXPECT_EQ(2u, enable_count_);
}

TEST_F(IdentifyPanelTest, LoadsReadOnlySettingsExplicitlyWithoutPersistingInRviz)
{
  auto root = settings(
    "[{kind: base, joints_deg: [0,45,45,0]}, {kind: check, joints_deg: [-10,40,50,15]}]");
  root["measurement"]["record_sec"] = 2.5;
  const auto path = configFile(root);
  ASSERT_TRUE(panel_->loadConfig(path));
  rviz_common::Config config;
  panel_->save(config);
  QString saved_path;
  EXPECT_FALSE(config.mapGetString("Identify config", &saved_path));
  EXPECT_FALSE(config.mapGetChild("poses").isValid());
  IdentifyTestPanel restored;
  restored.load(config);
  EXPECT_EQ(0, restored.findChild<QTableWidget *>("identify_poses")->rowCount());
  EXPECT_EQ("--", restored.findChild<QLabel *>("enable_load")->text());
  ASSERT_TRUE(restored.loadConfig(path));
  EXPECT_EQ("0.2", restored.findChild<QLabel *>("enable_load")->text());
  EXPECT_TRUE(restored.findChild<QLabel *>("settings_summary")->text().contains("Record: 2.5 s"));
  EXPECT_TRUE(restored.findChild<QLabel *>("settings_summary")->text().contains("Speed: 7%"));
  EXPECT_TRUE(restored.findChildren<QLineEdit *>().isEmpty());
  EXPECT_TRUE(restored.findChildren<QAbstractSpinBox *>().isEmpty());
  auto * table = restored.findChild<QTableWidget *>("identify_poses");
  ASSERT_EQ(2, table->rowCount());
  EXPECT_EQ(QAbstractItemView::NoEditTriggers, table->editTriggers());
  EXPECT_EQ(nullptr, table->cellWidget(1, 0));
  EXPECT_EQ("check", table->item(1, 0)->text());
  EXPECT_DOUBLE_EQ(-10, table->item(1, 1)->text().toDouble());
  EXPECT_DOUBLE_EQ(40, table->item(1, 2)->text().toDouble());
  EXPECT_DOUBLE_EQ(50, table->item(1, 3)->text().toDouble());
  EXPECT_NEAR(15, table->item(1, 4)->text().toDouble(), 1e-12);
}

TEST_F(IdentifyPanelTest, IgnoresLegacyYamlPathsAtStartupAndRvizRestore)
{
  const auto path = configFile();
  panel_.reset();
  node_->declare_parameter<std::string>("identify_config", path.toStdString());
  panel_ = std::make_unique<IdentifyTestPanel>();
  panel_->initializeRos(node_);
  rviz_common::Config config;
  config.mapSetValue("Identify config", path);
  panel_->load(config);
  stream_ = true;
  ASSERT_TRUE(
    waitFor(
      [this]() {
        return panel_->findChild<QLabel *>("robot_mode")->text() == "Robot Mode: DISABLED";
      }));
  EXPECT_EQ(0, panel_->findChild<QTableWidget *>("identify_poses")->rowCount());
  EXPECT_EQ("--", panel_->findChild<QLabel *>("enable_load")->text());
  EXPECT_EQ("No experiment YAML loaded.", panel_->findChild<QLabel *>("config_status")->text());
  EXPECT_FALSE(panel_->findChild<QPushButton *>("enable_robot")->isEnabled());
  EXPECT_FALSE(panel_->findChild<QPushButton *>("run_identify")->isEnabled());
  EXPECT_EQ(0u, enable_count_);
  EXPECT_TRUE(goals_.empty());
}

TEST_F(IdentifyPanelTest, CanRepeatSameExperimentWithSavedSettings)
{
  enableForRun();
  for (int run = 0; run < 2; ++run) {
    ASSERT_TRUE(panel_->startSequence(directory_.filePath(QString("run%1.mcap").arg(run))));
    ASSERT_TRUE(
      waitFor(
        [this]() {
          return panel_->findChild<QLabel *>("run_status")->text() ==
          "Measurement sequence completed.";
        }));
  }
  EXPECT_EQ(
    recordedSettings(directory_.filePath("run0.mcap")),
    recordedSettings(directory_.filePath("run1.mcap")));
  ASSERT_EQ(2u, goals_.size());
  EXPECT_EQ(goals_[0].joint_angles, goals_[1].joint_angles);
  EXPECT_EQ(goals_[0].speed_j, goals_[1].speed_j);
  EXPECT_EQ(1u, enable_count_);
}

TEST_F(IdentifyPanelTest, RejectsMalformedTelemetryAndExpiresDisplay)
{
  auto bad_joints = mg400_interface::JointHandler::getJointState({0.1, 0.2, 0.3, 0.4}, "");
  bad_joints->position.resize(2);
  joints_->publish(*bad_joints);
  Currents bad_currents;
  bad_currents.actual[1] = std::numeric_limits<double>::quiet_NaN();
  currents_->publish(bad_currents);
  ASSERT_TRUE(
    waitFor(
      [this]() {
        return panel_->findChild<QLabel *>("identify_actual_j1")->text() == "--";
      }));
  publishTelemetry();
  ASSERT_TRUE(
    waitFor(
      [this]() {
        return panel_->findChild<QLabel *>("identify_actual_j1")->text() == "0.5000";
      }));
  ASSERT_TRUE(
    waitFor(
      [this]() {
        return panel_->findChild<QLabel *>("identify_actual_j1")->text() == "--";
      }));
  EXPECT_EQ("--", panel_->findChild<QLabel *>("identify_angle_j1")->text());
}

TEST_F(IdentifyPanelTest, ClosingActiveRunSavesAndRequestsDisable)
{
  enableForRun();
  hold_ = true;
  const auto path = directory_.filePath("closed.mcap");
  ASSERT_TRUE(panel_->startSequence(path));
  ASSERT_TRUE(
    waitFor(
      [this]() {
        const auto status = panel_->findChild<QLabel *>("recording_status")->text();
        return handle_ && status.startsWith("Recording:") &&
        !status.contains("Recording: 0 joint messages") && !status.contains(", 0 current messages");
      }));
  panel_.reset();
  stream_ = false;
  ASSERT_TRUE(waitFor([this]() {return disable_count_ == 1;}));
  EXPECT_FALSE(readMessages<JointState>(path, "/mg400/joint_states").empty());
  EXPECT_FALSE(readMessages<Currents>(path, "/mg400/joint_currents").empty());
  EXPECT_EQ("Panel closed.", readSnapshot(path)["recording"]["result"].as<std::string>());
  for (const auto & window : readWindows(path)) {
    EXPECT_FALSE(window["complete"].as<bool>());
  }
}

TEST_F(IdentifyPanelTest, RequiresYamlAndLoadingDoesNotMove)
{
  EXPECT_EQ(0, panel_->findChild<QTableWidget *>("identify_poses")->rowCount());
  EXPECT_FALSE(panel_->startSequence(directory_.filePath("empty.mcap")));
  ASSERT_TRUE(panel_->loadConfig(configFile()));
  EXPECT_TRUE(goals_.empty());
  EXPECT_EQ(0u, enable_count_);
  EXPECT_FALSE(panel_->findChild<QPushButton *>("run_identify")->isEnabled());
}

TEST(IdentifyPluginTest, LoadsIndependentPanelThroughPluginlib)
{
  pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common", "rviz_common::Panel");
  auto panel = loader.createSharedInstance("mg400_rviz_plugin/Identify");
  EXPECT_NE(nullptr, panel->findChild<QPushButton *>("run_identify"));
  EXPECT_NE(nullptr, panel->findChild<QTableWidget *>("identify_poses"));
  EXPECT_EQ(nullptr, panel->findChild<QTabWidget *>("motion_tabs"));
  EXPECT_EQ(nullptr, panel->findChild<QPushButton *>("send_mov_j"));
  for (const auto * name : {"add_pose", "move_selected", "start_recording", "stop_recording"}) {
    EXPECT_EQ(nullptr, panel->findChild<QPushButton *>(name));
  }
  EXPECT_EQ(
    QAbstractItemView::NoEditTriggers,
    panel->findChild<QTableWidget *>("identify_poses")->editTriggers());
  const QStringList expected = {"Enable", "Stop / Disable", "Clear Error", "Load YAML...",
    "Run and record..."};
  const auto buttons = panel->findChildren<QPushButton *>();
  ASSERT_EQ(expected.size(), buttons.size());
  for (const auto * button : buttons) {
    EXPECT_TRUE(expected.contains(button->text()));
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  QApplication application(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
