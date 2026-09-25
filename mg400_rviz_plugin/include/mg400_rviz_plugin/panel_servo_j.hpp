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

#ifndef MG400_RVIZ_PLUGIN__PANEL_SERVO_J_HPP_
#define MG400_RVIZ_PLUGIN__PANEL_SERVO_J_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include <QtWidgets>

#ifndef Q_MOC_RUN
#include <mg400_common/mg400_ik_util.hpp>
#include <mg400_msgs/msg/robot_state.hpp>
#include <mg400_msgs/msg/servo_j.hpp>
#include <mg400_msgs/srv/disable_robot.hpp>
#include <mg400_msgs/srv/enable_robot.hpp>
#include <mg400_msgs/srv/servo_j_session.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "mg400_rviz_plugin/servo_j_panel_model.hpp"
#endif

namespace mg400_rviz_plugin
{

/** RViz panel for cautiously exercising the four-joint ServoJ stream.
 *
 * ROS callbacks and the publish scheduler run in a dedicated callback group
 * so GUI rendering cannot directly delay the 30 Hz setpoint stream.
 */
class ServoJPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  /** Build the controls without starting ROS communication. */
  explicit ServoJPanel(QWidget * parent = nullptr);

  /** Stop publication and the ROS worker before deleting Qt controls. */
  ~ServoJPanel() override;

  /** Bind the panel to RViz's ROS node and start the worker executor. */
  void onInitialize() override;

  /** Restore the requested publish frequency from an RViz layout. */
  void load(const rviz_common::Config & config) override;

  /** Save the requested publish frequency in an RViz layout. */
  void save(rviz_common::Config config) const override;

private:
  using Clock = std::chrono::steady_clock;
  using RobotState = mg400_msgs::msg::RobotState;
  using ServoJ = mg400_msgs::msg::ServoJ;
  using Session = mg400_msgs::srv::ServoJSession;
  using Enable = mg400_msgs::srv::EnableRobot;
  using Disable = mg400_msgs::srv::DisableRobot;

  enum class Phase {IDLE, STARTING, STREAMING, STOPPING, DISABLING};

  struct SharedState
  {
    RobotState robot_state{};
    bool state_seen{false};
    Clock::time_point state_received{};
    JointArray feedback{};
    bool feedback_seen{false};
    Clock::time_point feedback_received{};
    JointArray target{};
    JointArray last_command{};
    uint64_t session_id{0};
    uint64_t anchor_version{0};
    Phase phase{Phase::IDLE};
    bool stop_in_flight{false};
    bool enable_in_flight{false};
    bool disable_in_flight{false};
    bool first_point_pending{false};
    bool limits_enabled{true};
    int publish_hz{30};
    double measured_publish_hz{0.0};
    Clock::time_point last_publish{};
    Clock::time_point next_publish{};
    Clock::time_point last_stop_attempt{};
    Clock::time_point session_started{};
    std::string detail{"Waiting for robot feedback"};
  };

  void onEnableClicked();
  void onDisableClicked();
  void onStartClicked();
  void onStopClicked();
  void onSliderChanged(size_t joint, int value);
  void refreshUi();
  void onRobotState(const RobotState::ConstSharedPtr msg);
  void onJointState(const sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void onStreamTimer();
  void requestStop();
  bool feedbackReady(const SharedState & state, Clock::time_point now) const;
  bool robotReady(const SharedState & state, Clock::time_point now) const;
  bool targetInRange(const JointArray & target);
  void updateSliders(const JointArray & joints);
  static const char * phaseText(Phase phase);
  static const char * stateText(uint8_t state);

  std::array<QSlider *, 4> sliders_{};
  std::array<QLabel *, 4> feedback_labels_{};
  std::array<QLabel *, 4> target_labels_{};
  QLabel * robot_status_label_{nullptr};
  QLabel * session_status_label_{nullptr};
  QLabel * stream_status_label_{nullptr};
  QLabel * detail_label_{nullptr};
  QPushButton * enable_button_{nullptr};
  QPushButton * disable_button_{nullptr};
  QPushButton * start_button_{nullptr};
  QPushButton * stop_button_{nullptr};
  QSpinBox * frequency_spin_{nullptr};
  QCheckBox * limits_checkbox_{nullptr};
  QTimer * ui_timer_{nullptr};
  uint64_t displayed_anchor_version_{0};

  std::mutex mutex_;
  SharedState state_;
  ServoJRateLimiter limiter_;
  mg400_common::MG400IKUtil ik_util_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread executor_thread_;
  rclcpp::Subscription<RobotState>::SharedPtr robot_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<ServoJ>::SharedPtr servo_pub_;
  rclcpp::Client<Enable>::SharedPtr enable_client_;
  rclcpp::Client<Disable>::SharedPtr disable_client_;
  rclcpp::Client<Session>::SharedPtr session_client_;
  rclcpp::TimerBase::SharedPtr stream_timer_;
};

}  // namespace mg400_rviz_plugin

#endif  // MG400_RVIZ_PLUGIN__PANEL_SERVO_J_HPP_
