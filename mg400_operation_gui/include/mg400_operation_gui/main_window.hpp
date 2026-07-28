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

#ifndef MG400_OPERATION_GUI__MAIN_WINDOW_HPP_
#define MG400_OPERATION_GUI__MAIN_WINDOW_HPP_

#include <QMainWindow>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <mg400_msgs/msg/control_state.hpp>
#include <mg400_msgs/msg/realtime_feedback.hpp>
#include <mg400_msgs/msg/robot_mode.hpp>
#include <mg400_msgs/msg/servo_j.hpp>
#include <mg400_msgs/msg/servo_p.hpp>
#include <mg400_msgs/srv/change_control_state.hpp>
#include <mg400_msgs/srv/clear_error.hpp>
#include <mg400_msgs/srv/disable_robot.hpp>
#include <mg400_msgs/srv/enable_robot.hpp>
#include <rclcpp/rclcpp.hpp>

class QCloseEvent;
class QDoubleSpinBox;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QRadioButton;
class QSlider;
class QStackedWidget;
class QTimer;
class QVBoxLayout;
class QWidget;

namespace mg400_operation_gui
{

class MainWindow : public QMainWindow, public rclcpp::Node
{
  Q_OBJECT

public:
  explicit MainWindow(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MainWindow() override = default;

protected:
  void closeEvent(QCloseEvent * event) override;

private:
  enum class ServoMode
  {
    SERVO_J,
    SERVO_P,
  };

  struct AxisWidgets
  {
    QSlider * slider;
    QDoubleSpinBox * spin_box;
  };

  using ChangeControlState = mg400_msgs::srv::ChangeControlState;
  using ClearError = mg400_msgs::srv::ClearError;
  using DisableRobot = mg400_msgs::srv::DisableRobot;
  using EnableRobot = mg400_msgs::srv::EnableRobot;

  std::array<AxisWidgets, 4> servo_j_axes_{};
  std::array<AxisWidgets, 4> servo_p_axes_{};
  std::array<double, 4> current_joint_angles_{};
  std::array<double, 4> current_pose_{};
  std::array<double, 4> servo_j_filtered_target_{};
  std::array<double, 4> servo_p_filtered_target_{};
  std::array<QLabel *, 4> current_joint_value_labels_{};
  std::array<QLabel *, 4> current_pose_value_labels_{};

  QLabel * robot_mode_value_{nullptr};
  QLabel * control_state_value_{nullptr};
  QLabel * lease_value_{nullptr};
  QLabel * command_value_{nullptr};
  QLabel * target_help_{nullptr};
  QPushButton * enable_button_{nullptr};
  QPushButton * disable_button_{nullptr};
  QPushButton * clear_error_button_{nullptr};
  QPushButton * load_target_button_{nullptr};
  QPushButton * mode_start_button_{nullptr};
  QPushButton * mode_stop_button_{nullptr};
  QPushButton * publish_start_button_{nullptr};
  QPushButton * publish_stop_button_{nullptr};
  QDoubleSpinBox * filter_cutoff_spin_box_{nullptr};
  QRadioButton * servo_j_radio_{nullptr};
  QRadioButton * servo_p_radio_{nullptr};
  QStackedWidget * target_stack_{nullptr};
  QPlainTextEdit * log_view_{nullptr};
  QTimer * close_timeout_{nullptr};

  rclcpp::Client<EnableRobot>::SharedPtr enable_client_;
  rclcpp::Client<DisableRobot>::SharedPtr disable_client_;
  rclcpp::Client<ClearError>::SharedPtr clear_error_client_;
  rclcpp::Client<ChangeControlState>::SharedPtr control_state_client_;
  rclcpp::Publisher<mg400_msgs::msg::ServoJ>::SharedPtr servo_j_publisher_;
  rclcpp::Publisher<mg400_msgs::msg::ServoP>::SharedPtr servo_p_publisher_;
  rclcpp::Subscription<mg400_msgs::msg::RealtimeFeedback>::SharedPtr
    realtime_feedback_subscription_;
  rclcpp::Subscription<mg400_msgs::msg::RobotMode>::SharedPtr robot_mode_subscription_;
  rclcpp::Subscription<mg400_msgs::msg::ControlState>::SharedPtr
    control_state_subscription_;
  rclcpp::TimerBase::SharedPtr command_timer_;

  const std::string servo_p_frame_id_;
  std::uint64_t lease_id_{0};
  std::uint64_t robot_mode_{mg400_msgs::msg::RobotMode::INVALID};
  std::uint8_t control_state_{mg400_msgs::msg::ControlState::UNAVAILABLE};
  ServoMode active_mode_{ServoMode::SERVO_J};
  bool robot_mode_received_{false};
  bool control_state_received_{false};
  bool realtime_feedback_received_{false};
  bool servo_j_target_initialized_{false};
  bool servo_p_target_initialized_{false};
  bool servo_j_filter_initialized_{false};
  bool servo_p_filter_initialized_{false};
  bool command_active_{false};
  bool service_busy_{false};
  bool pending_servo_start_{false};
  bool stop_request_in_flight_{false};
  bool closing_{false};
  bool close_allowed_{false};
  std::chrono::steady_clock::time_point filter_updated_at_{};

  void setupUi();
  QWidget * createAxisPanel(
    const std::array<QString, 4> & labels,
    const std::array<double, 4> & minimums,
    const std::array<double, 4> & maximums,
    const std::array<QString, 4> & suffixes,
    std::array<AxisWidgets, 4> & axes);
  void createRosInterfaces(std::chrono::milliseconds command_period);
  void updateUiState();
  void setServiceBusy(bool busy);
  void setAxesEnabled(std::array<AxisWidgets, 4> & axes, bool enabled);
  bool setAxisValues(std::array<AxisWidgets, 4> & axes, const std::array<double, 4> & values);
  std::array<double, 4> axisValues(const std::array<AxisWidgets, 4> & axes) const;
  std::array<double, 4> filteredTargetValues();
  ServoMode selectedMode() const;
  bool selectedTargetInitialized() const;

  void callEnableRobot();
  void callDisableRobot();
  void callClearError();
  void loadCurrentTarget();
  void beginServoStart();
  void startPublishing();
  void stopPublishing();
  void requestServoStop();
  void publishServoTarget();

  void handleRealtimeFeedback(
    const mg400_msgs::msg::RealtimeFeedback::ConstSharedPtr message);
  void handleRobotMode(const mg400_msgs::msg::RobotMode::ConstSharedPtr message);
  void handleControlState(const mg400_msgs::msg::ControlState::ConstSharedPtr message);
  void appendLog(const QString & message, bool error = false);
  void continueCloseIfNeeded();
  void finishClose();

  static QString robotModeName(std::uint64_t mode);
  static QString controlStateName(std::uint8_t state);
};

}  // namespace mg400_operation_gui

#endif  // MG400_OPERATION_GUI__MAIN_WINDOW_HPP_
