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

#include "mg400_rviz_plugin/panel_servo_j.hpp"

#include <QPointer>
#include <QFocusEvent>
#include <QKeyEvent>
#include <QMouseEvent>
#include <cmath>
#include <vector>

#include <mg400_common/kinematics.hpp>
#include <mg400_msgs/msg/robot_mode.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace mg400_rviz_plugin
{
namespace
{

using namespace std::chrono_literals;  // NOLINT

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadiansToDegrees = 180.0 / kPi;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kMaximumSpeed = 10.0 * kDegreesToRadians;
constexpr double kMaximumAcceleration = 50.0 * kDegreesToRadians;
constexpr double kSliderScale = 10.0;  // One tick is 0.1 degree.
constexpr char kFocusFrameName[] = "servoJKeyboardFrame";
constexpr char kFocusFrameInactiveStyle[] =
  "QFrame#servoJKeyboardFrame { border: 2px solid transparent; border-radius: 4px; }";
constexpr char kFocusFrameActiveStyle[] =
  "QFrame#servoJKeyboardFrame { border: 2px solid #4dabf7; border-radius: 4px; }";

// Preserve arrow-key control on the clicked slider inside RViz.
class KeyboardSlider : public QSlider
{
public:
  KeyboardSlider(Qt::Orientation orientation, QFrame * frame, QLabel * label)
  : QSlider(orientation, frame), focus_frame_(frame), focus_label_(label),
    normal_label_(label->text())
  {
    setFocusPolicy(Qt::StrongFocus);
    setSingleStep(1);
    updateFocusIndicator(false);
  }

protected:
  void focusInEvent(QFocusEvent * event) override
  {
    QSlider::focusInEvent(event);
    updateFocusIndicator(true);
  }

  void focusOutEvent(QFocusEvent * event) override
  {
    QSlider::focusOutEvent(event);
    updateFocusIndicator(false);
  }

  void mousePressEvent(QMouseEvent * event) override
  {
    setFocus(Qt::MouseFocusReason);
    QSlider::mousePressEvent(event);
  }

  bool event(QEvent * event) override
  {
    if (event->type() == QEvent::ShortcutOverride && hasFocus()) {
      const auto * key_event = static_cast<QKeyEvent *>(event);
      if (key_event->key() == Qt::Key_Left || key_event->key() == Qt::Key_Right) {
        event->accept();
        return true;
      }
    }
    return QSlider::event(event);
  }

  void keyPressEvent(QKeyEvent * event) override
  {
    if (event->key() == Qt::Key_Left) {
      triggerAction(QAbstractSlider::SliderSingleStepSub);
    } else if (event->key() == Qt::Key_Right) {
      triggerAction(QAbstractSlider::SliderSingleStepAdd);
    } else {
      QSlider::keyPressEvent(event);
      return;
    }
    event->accept();
  }

private:
  void updateFocusIndicator(bool focused)
  {
    if (focus_frame_) {
      focus_frame_->setStyleSheet(
        focused ? kFocusFrameActiveStyle : kFocusFrameInactiveStyle);
    }
    if (focus_label_) {
      focus_label_->setText(focused ? normal_label_ + QStringLiteral(" [KEY]") : normal_label_);
      focus_label_->setStyleSheet(focused ? "color: #4dabf7; font-weight: bold;" : "");
    }
  }

  QPointer<QFrame> focus_frame_;
  QPointer<QLabel> focus_label_;
  QString normal_label_;
};

QString ageText(bool seen, std::chrono::steady_clock::time_point received)
{
  if (!seen) {
    return "not received";
  }
  const auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() - received);
  return QString::number(age.count() * 1000.0, 'f', 0) + " ms";
}

}  // namespace

ServoJPanel::ServoJPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout;
  auto * robot_buttons = new QHBoxLayout;
  enable_button_ = new QPushButton("Enable");
  disable_button_ = new QPushButton("Disable");
  robot_buttons->addWidget(enable_button_);
  robot_buttons->addWidget(disable_button_);
  layout->addLayout(robot_buttons);

  auto * servo_buttons = new QHBoxLayout;
  start_button_ = new QPushButton("Start ServoJ");
  stop_button_ = new QPushButton("Stop ServoJ");
  servo_buttons->addWidget(start_button_);
  servo_buttons->addWidget(stop_button_);
  layout->addLayout(servo_buttons);

  auto * rate_row = new QHBoxLayout;
  rate_row->addWidget(new QLabel("Publish frequency:"));
  frequency_spin_ = new QSpinBox;
  frequency_spin_->setRange(10, 33);
  frequency_spin_->setValue(30);
  frequency_spin_->setSuffix(" Hz");
  frequency_spin_->setToolTip(
    "ServoJ topic publish rate. The driver checks for new points every 30 ms, "
    "so the actual TCP command rate cannot exceed about 33 Hz.");
  rate_row->addWidget(frequency_spin_);
  layout->addLayout(rate_row);
  limits_checkbox_ = new QCheckBox("Limit speed and acceleration");
  limits_checkbox_->setChecked(true);
  limits_checkbox_->setToolTip(
    "Limit each joint to 10 deg/s and 50 deg/s^2. Uncheck only while the "
    "session is stopped; subsequent slider targets will be sent directly.");
  layout->addWidget(limits_checkbox_);

  auto * joints = new QGridLayout;
  joints->addWidget(new QLabel("Joint"), 0, 0);
  joints->addWidget(new QLabel("Target slider"), 0, 1);
  joints->addWidget(new QLabel("FB [deg]"), 0, 2);
  joints->addWidget(new QLabel("Target [deg]"), 0, 3);
  const std::array<double, 4> lower = {
    mg400_common::kinematics::J1_MIN, mg400_common::kinematics::J2_MIN,
    mg400_common::kinematics::J3_MIN, mg400_common::kinematics::J4_MIN};
  const std::array<double, 4> upper = {
    mg400_common::kinematics::J1_MAX, mg400_common::kinematics::J2_MAX,
    mg400_common::kinematics::J3_MAX, mg400_common::kinematics::J4_MAX};
  for (size_t joint = 0; joint < 4; ++joint) {
    auto * joint_label = new QLabel(QString("J%1").arg(joint + 1));
    joints->addWidget(joint_label, joint + 1, 0);
    auto * focus_frame = new QFrame;
    focus_frame->setObjectName(kFocusFrameName);
    auto * slider_layout = new QHBoxLayout(focus_frame);
    slider_layout->setContentsMargins(4, 1, 4, 1);
    sliders_[joint] = new KeyboardSlider(Qt::Horizontal, focus_frame, joint_label);
    slider_layout->addWidget(sliders_[joint]);
    sliders_[joint]->setRange(
      static_cast<int>(std::ceil(lower[joint] * kRadiansToDegrees * kSliderScale)),
      static_cast<int>(std::floor(upper[joint] * kRadiansToDegrees * kSliderScale)));
    sliders_[joint]->setEnabled(false);
    joints->addWidget(focus_frame, joint + 1, 1);
    feedback_labels_[joint] = new QLabel("--");
    target_labels_[joint] = new QLabel("--");
    joints->addWidget(feedback_labels_[joint], joint + 1, 2);
    joints->addWidget(target_labels_[joint], joint + 1, 3);
    connect(
      sliders_[joint], &QSlider::valueChanged, this,
      [this, joint](int value) {onSliderChanged(joint, value);});
  }
  layout->addLayout(joints);

  robot_status_label_ = new QLabel("Robot: waiting for feedback");
  session_status_label_ = new QLabel("Session: idle");
  stream_status_label_ = new QLabel("Stream: 0 Hz");
  detail_label_ = new QLabel("Waiting for robot feedback");
  detail_label_->setWordWrap(true);
  layout->addWidget(robot_status_label_);
  layout->addWidget(session_status_label_);
  layout->addWidget(stream_status_label_);
  layout->addWidget(detail_label_);
  setLayout(layout);

  connect(enable_button_, &QPushButton::clicked, this, &ServoJPanel::onEnableClicked);
  connect(disable_button_, &QPushButton::clicked, this, &ServoJPanel::onDisableClicked);
  connect(start_button_, &QPushButton::clicked, this, &ServoJPanel::onStartClicked);
  connect(stop_button_, &QPushButton::clicked, this, &ServoJPanel::onStopClicked);
  connect(
    frequency_spin_, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
    this, [this](int value) {
      std::lock_guard<std::mutex> lock(mutex_);
      state_.publish_hz = value;
    });

  ui_timer_ = new QTimer(this);
  connect(limits_checkbox_, &QCheckBox::toggled, this, [this](bool checked) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.phase != Phase::IDLE) {
      const QSignalBlocker blocker(limits_checkbox_);
      limits_checkbox_->setChecked(state_.limits_enabled);
      return;
    }
    state_.limits_enabled = checked;
    state_.detail = checked ?
      "Speed and acceleration limits enabled" :
      "Speed and acceleration limits disabled; targets will be sent directly";
  });
  connect(ui_timer_, &QTimer::timeout, this, &ServoJPanel::refreshUi);
  ui_timer_->start(50);
  refreshUi();
}

ServoJPanel::~ServoJPanel()
{
  ui_timer_->stop();
  uint64_t session_id = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    session_id = state_.session_id;
    state_.phase = Phase::STOPPING;
  }
  // A bounded best-effort release prevents a closed panel from publishing
  // further points. The driver retains the session if hardware is not idle.
  if (session_id != 0 && session_client_ && session_client_->service_is_ready()) {
    auto request = std::make_shared<Session::Request>();
    request->start = false;
    request->session_id = session_id;
    try {
      auto future = session_client_->async_send_request(request);
      future.wait_for(500ms);
    } catch (const std::exception & error) {
      if (node_) {
        RCLCPP_WARN(node_->get_logger(), "ServoJ panel shutdown: %s", error.what());
      }
    }
  }
  executor_.cancel();
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  stream_timer_.reset();
  session_client_.reset();
  enable_client_.reset();
  disable_client_.reset();
  joint_state_sub_.reset();
  robot_state_sub_.reset();
  servo_pub_.reset();
}

void ServoJPanel::onInitialize()
{
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    detail_label_->setText("RViz ROS node unavailable");
    return;
  }
  node_ = abstraction->get_raw_node();
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  robot_state_sub_ = node_->create_subscription<RobotState>(
    "/mg400/robot_state", rclcpp::QoS(1).reliable().transient_local(),
    [this](RobotState::ConstSharedPtr msg) {onRobotState(msg);}, options);
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/mg400/joint_states", rclcpp::SensorDataQoS().keep_last(1),
    [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {onJointState(msg);}, options);
  servo_pub_ = node_->create_publisher<ServoJ>(
    "/mg400/servo_j", rclcpp::QoS(1).best_effort().durability_volatile());
  enable_client_ = node_->create_client<Enable>(
    "/mg400/enable_robot", rmw_qos_profile_services_default, callback_group_);
  disable_client_ = node_->create_client<Disable>(
    "/mg400/disable_robot", rmw_qos_profile_services_default, callback_group_);
  session_client_ = node_->create_client<Session>(
    "/mg400/servo_j_session", rmw_qos_profile_services_default, callback_group_);
  stream_timer_ = node_->create_wall_timer(5ms, [this]() {onStreamTimer();}, callback_group_);
  executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  executor_thread_ = std::thread([this]() {executor_.spin();});
}

void ServoJPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  int frequency = 30;
  if (config.mapGetInt("ServoJ Publish Hz", &frequency)) {
    frequency_spin_->setValue(frequency);
  }
}

void ServoJPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("ServoJ Publish Hz", frequency_spin_->value());
}

bool ServoJPanel::feedbackReady(const SharedState & state, Clock::time_point now) const
{
  return state.feedback_seen && now - state.feedback_received <= 200ms;
}

bool ServoJPanel::robotReady(const SharedState & state, Clock::time_point now) const
{
  return state.state_seen && state.robot_state.feedback_fresh &&
         now - state.state_received <= 500ms;
}

bool ServoJPanel::targetInRange(const JointArray & target)
{
  return ik_util_.InMG400Range(std::vector<double>(target.begin(), target.end()));
}

void ServoJPanel::onEnableClicked()
{
  if (!enable_client_ || !enable_client_->service_is_ready()) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.detail = "EnableRobot service unavailable";
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!robotReady(state_, Clock::now()) || state_.robot_state.state != RobotState::DISABLED ||
      state_.enable_in_flight || state_.phase != Phase::IDLE)
    {
      return;
    }
    state_.enable_in_flight = true;
    state_.detail = "EnableRobot requested";
  }
  auto request = std::make_shared<Enable::Request>();
  request->num_of_params = Enable::Request::NO_PARAM;
  try {
    enable_client_->async_send_request(
      request, [this](rclcpp::Client<Enable>::SharedFuture future) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.enable_in_flight = false;
        try {
          const auto response = future.get();
          state_.detail = response->result ?
            "EnableRobot accepted; waiting for ENABLE feedback" :
            "EnableRobot failed, error_id=" + std::to_string(response->error_id);
        } catch (const std::exception & error) {
          state_.detail = std::string("EnableRobot failed: ") + error.what();
        }
      });
  } catch (const std::exception & error) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.enable_in_flight = false;
    state_.detail = std::string("EnableRobot request failed: ") + error.what();
  }
}

void ServoJPanel::onDisableClicked()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.phase != Phase::IDLE) {
      state_.phase = Phase::DISABLING;  // Publication stops before the service request.
    }
    state_.detail = "DisableRobot requested; ServoJ publication stopped";
  }
  if (!disable_client_ || !disable_client_->service_is_ready()) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.session_id != 0) {
      state_.phase = Phase::STOPPING;
    }
    state_.detail = "DisableRobot service unavailable; trying ServoJ stop";
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.disable_in_flight) {
      return;
    }
    state_.disable_in_flight = true;
  }
  try {
    disable_client_->async_send_request(
      std::make_shared<Disable::Request>(),
      [this](rclcpp::Client<Disable>::SharedFuture future) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.disable_in_flight = false;
        try {
          const auto response = future.get();
          if (response->result) {
            state_.detail = "DisableRobot accepted; waiting for DISABLED feedback";
          } else {
            state_.detail = "DisableRobot failed, error_id=" +
              std::to_string(response->error_id);
            if (state_.session_id != 0) {
              state_.phase = Phase::STOPPING;
            }
          }
        } catch (const std::exception & error) {
          state_.detail = std::string("DisableRobot failed: ") + error.what();
          if (state_.session_id != 0) {
            state_.phase = Phase::STOPPING;
          }
        }
      });
  } catch (const std::exception & error) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.disable_in_flight = false;
    state_.detail = std::string("DisableRobot request failed: ") + error.what();
    if (state_.session_id != 0) {
      state_.phase = Phase::STOPPING;
    }
  }
}

void ServoJPanel::onStartClicked()
{
  if (!session_client_ || !session_client_->service_is_ready()) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.detail = "ServoJ session service unavailable";
    return;
  }
  JointArray anchor{};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = Clock::now();
    if (state_.phase != Phase::IDLE || !robotReady(state_, now) ||
      !feedbackReady(state_, now) || state_.robot_state.state != RobotState::ENABLED ||
      !targetInRange(state_.feedback))
    {
      state_.detail = "Start requires fresh ENABLE and valid joint feedback";
      return;
    }
    anchor = state_.feedback;
    state_.target = anchor;
    state_.phase = Phase::STARTING;
    state_.detail = "Requesting ServoJ session";
  }
  updateSliders(anchor);
  auto request = std::make_shared<Session::Request>();
  request->start = true;
  request->session_id = 0;
  try {
    session_client_->async_send_request(
      request, [this](rclcpp::Client<Session>::SharedFuture future) {
        std::lock_guard<std::mutex> lock(mutex_);
        try {
          const auto response = future.get();
          if (!response->success || response->session_id == 0) {
            if (state_.phase == Phase::STARTING ||
              (state_.phase == Phase::STOPPING && state_.session_id == 0)) {
              state_.phase = Phase::IDLE;
            }
            state_.detail = "ServoJ start rejected: " + response->message;
            return;
          }
          state_.session_id = response->session_id;
          const auto now = Clock::now();
          if (state_.phase != Phase::STARTING || !robotReady(state_, now) ||
            !feedbackReady(state_, now) || !targetInRange(state_.feedback))
          {
            if (state_.phase != Phase::DISABLING) {
              state_.phase = Phase::STOPPING;
            }
            state_.detail = "Start response arrived after state changed; stopping session";
            return;
          }
          // The first setpoint uses full-precision feedback, not slider ticks.
          limiter_.reset(state_.feedback);
          state_.target = state_.feedback;
          state_.last_command = state_.feedback;
          state_.anchor_version++;
          state_.session_started = now;
          state_.last_publish = Clock::time_point{};
          state_.next_publish = now;
          state_.measured_publish_hz = 0.0;
          state_.first_point_pending = true;
          state_.phase = Phase::STREAMING;
          state_.detail = "ServoJ streaming from current joint feedback";
        } catch (const std::exception & error) {
          if (state_.phase == Phase::STARTING ||
            (state_.phase == Phase::STOPPING && state_.session_id == 0)) {
            state_.phase = Phase::IDLE;
          }
          state_.detail = std::string("ServoJ start request failed: ") + error.what();
        }
      });
  } catch (const std::exception & error) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.phase = Phase::IDLE;
    state_.detail = std::string("ServoJ start request failed: ") + error.what();
  }
}

void ServoJPanel::onStopClicked()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_.phase == Phase::STARTING || state_.phase == Phase::STREAMING) {
    state_.phase = Phase::STOPPING;
    state_.detail = "ServoJ publication stopped; requesting session release";
  }
}

void ServoJPanel::onSliderChanged(size_t joint, int value)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_.phase == Phase::STREAMING) {
    state_.target[joint] = static_cast<double>(value) / kSliderScale * kDegreesToRadians;
  }
}

void ServoJPanel::onRobotState(const RobotState::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.robot_state = *msg;
  state_.state_seen = true;
  state_.state_received = Clock::now();
  if (state_.phase == Phase::DISABLING && msg->state == RobotState::DISABLED) {
    state_.phase = Phase::IDLE;
    state_.session_id = 0;
    state_.detail = "Robot disabled; ServoJ session invalidated";
  } else if (state_.phase == Phase::STREAMING &&
    (msg->state == RobotState::ERROR || msg->state == RobotState::DISABLED ||
    msg->state == RobotState::MANUAL || msg->state == RobotState::UNKNOWN))
  {
    state_.phase = Phase::STOPPING;
    state_.detail = "Robot left ServoJ state; publication stopped";
  }
}

void ServoJPanel::onJointState(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  JointArray joints{};
  if (!extractServoJoints(*msg, joints)) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  state_.feedback = joints;
  state_.feedback_seen = true;
  state_.feedback_received = Clock::now();
}

void ServoJPanel::requestStop()
{
  uint64_t session_id = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.phase != Phase::STOPPING || state_.session_id == 0 || state_.stop_in_flight ||
      Clock::now() - state_.last_stop_attempt < 250ms)
    {
      return;
    }
    state_.stop_in_flight = true;
    state_.last_stop_attempt = Clock::now();
    session_id = state_.session_id;
  }
  if (!session_client_ || !session_client_->service_is_ready()) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.stop_in_flight = false;
    state_.detail = "ServoJ stop service unavailable; no points are being published";
    return;
  }
  auto request = std::make_shared<Session::Request>();
  request->start = false;
  request->session_id = session_id;
  try {
    session_client_->async_send_request(
      request, [this, session_id](rclcpp::Client<Session>::SharedFuture future) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.stop_in_flight = false;
        if (state_.session_id != session_id) {
          return;
        }
        try {
          const auto response = future.get();
          if (response->success ||
            response->message == "ServoJ session ID does not match")
          {
            state_.session_id = 0;
            state_.phase = Phase::IDLE;
            state_.detail = response->success ?
              "ServoJ session stopped" : "ServoJ session no longer owned by this panel";
          } else {
            state_.detail = "ServoJ stop pending: " + response->message;
          }
        } catch (const std::exception & error) {
          state_.detail = std::string("ServoJ stop request failed: ") + error.what();
        }
      });
  } catch (const std::exception & error) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.stop_in_flight = false;
    state_.detail = std::string("ServoJ stop request failed: ") + error.what();
  }
}

void ServoJPanel::onStreamTimer()
{
  const auto now = Clock::now();
  ServoJ message;
  bool publish = false;
  bool stop = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.phase == Phase::STREAMING) {
      if (!robotReady(state_, now) || !feedbackReady(state_, now) ||
        (state_.last_publish != Clock::time_point{} && now - state_.last_publish > 120ms) ||
        (state_.robot_state.state != RobotState::SERVO &&
        (state_.robot_state.state != RobotState::ENABLED ||
        now - state_.session_started > 500ms)))
      {
        state_.phase = Phase::STOPPING;
        state_.detail = "Feedback or ServoJ publish deadline lost; publication stopped";
      } else if (now >= state_.next_publish) {
        const double dt = state_.last_publish == Clock::time_point{} ?
          1.0 / state_.publish_hz :
          std::chrono::duration<double>(now - state_.last_publish).count();
        JointArray next = state_.last_command;
        if (!state_.first_point_pending && targetInRange(state_.target)) {
          if (state_.limits_enabled) {
            next = limiter_.step(state_.target, std::min(dt, 0.05),
                kMaximumSpeed, kMaximumAcceleration);
          } else {
            next = state_.target;
          }
        } else if (!state_.first_point_pending) {
          state_.detail = "Target outside MG400 joint constraints; holding last command";
        }
        if (!targetInRange(next)) {
          state_.phase = Phase::STOPPING;
          state_.detail = "Interpolated target outside MG400 joint constraints";
        } else {
          message.session_id = state_.session_id;
          message.joint_positions = next;
          message.t = 0.0;  // Use the driver's configured ServoJ t.
          state_.last_command = next;
          if (state_.last_publish != Clock::time_point{}) {
            const double rate = 1.0 / dt;
            state_.measured_publish_hz = state_.measured_publish_hz == 0.0 ?
              rate : 0.9 * state_.measured_publish_hz + 0.1 * rate;
          }
          state_.last_publish = now;
          const auto period = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(1.0 / state_.publish_hz));
          do {
            state_.next_publish += period;
          } while (state_.next_publish <= now);
          publish = true;
          state_.first_point_pending = false;
        }
      }
    }
    if (publish) {
      try {
        servo_pub_->publish(message);
      } catch (const std::exception & error) {
        state_.phase = Phase::STOPPING;
        state_.detail = std::string("ServoJ topic publish failed: ") + error.what();
      }
    }
    stop = state_.phase == Phase::STOPPING;
  }
  if (stop) {
    requestStop();
  }
}

void ServoJPanel::updateSliders(const JointArray & joints)
{
  for (size_t joint = 0; joint < 4; ++joint) {
    const QSignalBlocker blocker(sliders_[joint]);
    sliders_[joint]->setValue(static_cast<int>(std::lround(
      joints[joint] * kRadiansToDegrees * kSliderScale)));
  }
}

void ServoJPanel::refreshUi()
{
  SharedState snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot = state_;
  }
  const auto now = Clock::now();
  if (feedbackReady(snapshot, now) &&
    (snapshot.phase == Phase::IDLE || snapshot.anchor_version != displayed_anchor_version_))
  {
    updateSliders(snapshot.phase == Phase::IDLE ? snapshot.feedback : snapshot.target);
    displayed_anchor_version_ = snapshot.anchor_version;
  }
  for (size_t joint = 0; joint < 4; ++joint) {
    feedback_labels_[joint]->setText(snapshot.feedback_seen ?
      QString::number(snapshot.feedback[joint] * kRadiansToDegrees, 'f', 2) : "--");
    const auto target = snapshot.phase == Phase::IDLE ? snapshot.feedback : snapshot.target;
    target_labels_[joint]->setText(snapshot.feedback_seen ?
      QString::number(target[joint] * kRadiansToDegrees, 'f', 2) : "--");
    if (snapshot.phase != Phase::STREAMING) {
      sliders_[joint]->clearFocus();
    }
    sliders_[joint]->setEnabled(snapshot.phase == Phase::STREAMING);
  }
  const bool robot_ready = robotReady(snapshot, now);
  const bool feedback_ready = feedbackReady(snapshot, now);
  enable_button_->setEnabled(
    robot_ready && snapshot.robot_state.state == RobotState::DISABLED &&
    snapshot.phase == Phase::IDLE && !snapshot.enable_in_flight);
  disable_button_->setEnabled(
    robot_ready && snapshot.robot_state.state != RobotState::DISABLED &&
    !snapshot.disable_in_flight);
  start_button_->setEnabled(
    robot_ready && feedback_ready && snapshot.phase == Phase::IDLE &&
    snapshot.robot_state.state == RobotState::ENABLED &&
    !snapshot.enable_in_flight && !snapshot.disable_in_flight &&
    session_client_ && session_client_->service_is_ready());
  stop_button_->setEnabled(
    snapshot.phase == Phase::STARTING || snapshot.phase == Phase::STREAMING ||
    snapshot.phase == Phase::STOPPING);
  frequency_spin_->setEnabled(snapshot.phase == Phase::IDLE);
  limits_checkbox_->setEnabled(snapshot.phase == Phase::IDLE);
  robot_status_label_->setText(
    QString("Robot: %1 (raw=%2, fresh=%3, state age=%4, joint age=%5)")
    .arg(stateText(snapshot.robot_state.state))
    .arg(snapshot.robot_state.raw_robot_mode)
    .arg(robot_ready ? "yes" : "no")
    .arg(ageText(snapshot.state_seen, snapshot.state_received))
    .arg(ageText(snapshot.feedback_seen, snapshot.feedback_received)));
  session_status_label_->setText(
    QString("Session: %1, ID=%2")
    .arg(phaseText(snapshot.phase)).arg(snapshot.session_id));
  stream_status_label_->setText(
    QString("Publish: requested %1 Hz, measured %2 Hz, last point %3, limits %4")
    .arg(snapshot.publish_hz)
    .arg(snapshot.measured_publish_hz, 0, 'f', 1)
    .arg(ageText(snapshot.last_publish != Clock::time_point{}, snapshot.last_publish))
    .arg(snapshot.limits_enabled ? "ON" : "OFF"));
  detail_label_->setText(QString::fromStdString(snapshot.detail));
}

const char * ServoJPanel::phaseText(Phase phase)
{
  switch (phase) {
    case Phase::IDLE: return "IDLE";
    case Phase::STARTING: return "STARTING";
    case Phase::STREAMING: return "STREAMING";
    case Phase::STOPPING: return "STOPPING";
    case Phase::DISABLING: return "DISABLING";
  }
  return "UNKNOWN";
}

const char * ServoJPanel::stateText(uint8_t state)
{
  switch (state) {
    case RobotState::UNKNOWN: return "UNKNOWN";
    case RobotState::NOT_READY: return "NOT_READY";
    case RobotState::MANUAL: return "MANUAL";
    case RobotState::DISABLED: return "DISABLED";
    case RobotState::ENABLED: return "ENABLED";
    case RobotState::RUNNING: return "RUNNING";
    case RobotState::PAUSED_OR_JOG: return "PAUSED_OR_JOG";
    case RobotState::ERROR: return "ERROR";
    case RobotState::SERVO: return "SERVO";
  }
  return "INVALID";
}

}  // namespace mg400_rviz_plugin

PLUGINLIB_EXPORT_CLASS(mg400_rviz_plugin::ServoJPanel, rviz_common::Panel)
