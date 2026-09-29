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

#ifndef MG400_RVIZ_PLUGIN__PANEL_IDENTIFY_HPP_
#define MG400_RVIZ_PLUGIN__PANEL_IDENTIFY_HPP_

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#ifndef Q_MOC_RUN
#include <mg400_msgs/action/joint_mov_j.hpp>
#include <mg400_msgs/msg/joint_currents.hpp>
#include <mg400_msgs/msg/robot_mode.hpp>
#include <mg400_msgs/srv/clear_error.hpp>
#include <mg400_msgs/srv/disable_robot.hpp>
#include <mg400_msgs/srv/enable_robot.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <QtWidgets>
#endif

namespace mg400_rviz_plugin
{
class IdentifyPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit IdentifyPanel(QWidget * parent = nullptr);
  ~IdentifyPanel() override;
  void onInitialize() override;

protected:
  void initializeRos(const rclcpp::Node::SharedPtr & node);
  bool startSequence(const QString & path);
  bool loadConfig(const QString & path);

private:
  using JointCurrents = mg400_msgs::msg::JointCurrents;
  using JointState = sensor_msgs::msg::JointState;
  using RobotMode = mg400_msgs::msg::RobotMode;
  using Action = mg400_msgs::action::JointMovJ;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
  using Enable = mg400_msgs::srv::EnableRobot;
  using Clock = std::chrono::steady_clock;
  enum class Phase {Idle, Moving, Settling, Sampling};
  struct Pose
  {
    QString kind;
    std::array<double, 4> radians;
  };

  struct MeasurementWindow
  {
    size_t segment;
    Pose pose;
    int64_t start_ns;
    int64_t end_ns = 0;
    bool complete = false;
  };

  struct Settings
  {
    Enable::Request payload;
    int speed_percent = 0;
    int acceleration_percent = 0;
    int repeats = 0;
    double settle_sec = 0.0;
    double record_sec = 0.0;
    std::vector<Pose> poses;
  };

  void tick();
  void refreshControls();
  void onJoints(const JointState::ConstSharedPtr msg);
  void onCurrents(const JointCurrents::ConstSharedPtr msg);
  void appendPoseRow(const Pose & pose);
  bool startRecording(const QString & path, bool overwrite);
  void stopRecording(const QString & reason = "Panel closed.");
  bool robotReady() const;
  bool telemetryReady() const;
  bool atTarget() const;
  void sendPose(const Pose & pose);
  void advanceSequence();
  void endRun(const QString & reason, bool disable);
  void disableRobot();
  void saveSnapshot(const QString & reason);
  void recordingFailed(const QString & error);
  template<typename Message>
  bool recordMessage(const Message & message, const std::string & topic);

  template<typename Service>
  void requestService(
    const typename rclcpp::Client<Service>::SharedPtr & client,
    const typename Service::Request::SharedPtr & request, const QString & name,
    std::function<void(bool)> completed = {});

  std::array<QLabel *, 4> payload_values_;
  QLabel * config_status_;
  QLabel * settings_summary_;
  QPushButton * enable_;
  QPushButton * disable_;
  QPushButton * clear_;
  QLabel * mode_label_;
  QLabel * service_status_;
  QTableWidget * poses_;
  QPushButton * run_;
  QPushButton * load_config_;
  QLabel * run_status_;
  std::array<QLabel *, 4> angles_;
  std::array<QLabel *, 4> actual_;
  std::array<QLabel *, 4> target_;
  QLabel * telemetry_status_;
  QLabel * recording_status_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<JointState>::SharedPtr joints_sub_;
  rclcpp::Subscription<JointCurrents>::SharedPtr currents_sub_;
  rclcpp::Subscription<RobotMode>::SharedPtr mode_sub_;
  rclcpp_action::Client<Action>::SharedPtr motion_;
  rclcpp::Client<Enable>::SharedPtr enable_client_;
  rclcpp::Client<mg400_msgs::srv::DisableRobot>::SharedPtr disable_client_;
  rclcpp::Client<mg400_msgs::srv::ClearError>::SharedPtr clear_client_;
  bool service_pending_ = false;
  Clock::time_point service_deadline_{};
  std::function<void()> remove_request_;
  bool payload_confirmed_ = false;
  Enable::Request enabled_payload_;
  RobotMode::_robot_mode_type mode_ = RobotMode::INIT;
  Clock::time_point mode_received_{};
  Clock::time_point joints_received_{};
  Clock::time_point currents_received_{};
  Clock::time_point stable_since_{};
  std::array<double, 4> joint_values_{};
  std::array<double, 4> stable_anchor_{};

  Settings settings_;
  bool settings_loaded_ = false;
  QString config_path_;
  QByteArray config_yaml_;
  Phase phase_ = Phase::Idle;
  uint64_t generation_ = 0;
  std::vector<Pose> plan_;
  size_t segment_ = 0;
  Pose commanded_pose_;
  Clock::time_point phase_started_{};
  Clock::time_point motion_deadline_{};
  int64_t sample_start_ns_ = 0;
  size_t sample_joints_ = 0;
  size_t sample_currents_ = 0;
  std::unique_ptr<rosbag2_cpp::Writer> recording_;
  std::unique_ptr<QTemporaryDir> recording_directory_;
  QString recording_path_;
  std::vector<MeasurementWindow> measurement_windows_;
  int64_t recording_started_ns_ = 0;
  int64_t last_record_ns_ = 0;
  size_t joint_rows_ = 0;
  size_t current_rows_ = 0;
};
}  // namespace mg400_rviz_plugin

#endif  // MG400_RVIZ_PLUGIN__PANEL_IDENTIFY_HPP_
