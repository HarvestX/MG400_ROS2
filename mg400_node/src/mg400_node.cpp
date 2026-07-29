// Copyright 2022 HarvestX Inc.
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

#include "mg400_node/mg400_node.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include "mg400_interface/tcp_interface/realtime_data_converter.hpp"

namespace mg400_node
{
using namespace std::chrono_literals;   // NOLINT

MG400Node::MG400Node(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("mg400_node", options)
{
  this->declare_parameter<bool>("auto_configure", true);
  this->declare_parameter<bool>("auto_connect", true);
  this->declare_parameter<int>("auto_configure_start_delay_msec", 100);
  this->declare_parameter<std::string>("ip_address", "192.168.1.6");
  this->declare_parameter<std::vector<std::string>>(
    "dashboard_api_plugins", this->default_dashboard_api_plugins_);
  this->declare_parameter<std::vector<std::string>>(
    "motion_api_plugins", this->default_motion_api_plugins_);
  this->declare_parameter<std::string>("prefix", "");
  this->declare_parameter<int>("servo.send_period_ms", 30);
  this->declare_parameter<int>("servo.target_watchdog_ms", 30000);
  this->declare_parameter<int>("servo.stop_timeout_ms", 2000);
  this->declare_parameter<int>("servo.stop_confirmation_poll_ms", 10);
  this->declare_parameter<int>("servo.safety.feedback_timeout_ms", 100);
  this->declare_parameter<double>("servo.safety.max_initial_joint_distance_rad", 0.0872665);
  this->declare_parameter<double>("servo.safety.max_joint_step_rad", 0.0174533);

  if (this->get_parameter("auto_configure").as_bool()) {
    RCLCPP_INFO(
      this->get_logger(), "Auto configure is enabled. Delaying for %ld msec.",
      this->get_parameter("auto_configure_start_delay_msec").as_int());
  }

  if (this->get_parameter("auto_connect").as_bool()) {
    RCLCPP_INFO(this->get_logger(), "Auto connect is enabled.");
  }

  autoconfigure_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(
      this->get_parameter("auto_configure_start_delay_msec").as_int()),
    std::bind(&MG400Node::handleAutoConfigure, this));
}

MG400Node::~MG400Node()
{
  this->cancelTimer();
  if (this->servo_control_ros_interface_) {
    const auto stopped = this->servo_control_ros_interface_->stopForLifecycle(
      "MG400Node destructor");
    if (!stopped.success) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Servo safe stop failed during destruction; lease remains fail-closed: %s",
        stopped.message.c_str());
    }
    // Destruction is the final fallback: even after a failed best-effort stop,
    // the Session must still be destroyed before its raw TCP dependencies.
    this->servo_control_ros_interface_->clearSession();
    this->servo_control_ros_interface_.reset();
  }
  if (this->interface_) {
    this->interface_->deactivate();
    this->interface_active_ = false;
  }
}

void MG400Node::handleAutoConfigure()
{
  if (autoconfigure_executed_.exchange(true)) {
    return;
  }

  if (autoconfigure_timer_) {
    autoconfigure_timer_->cancel();
  }

  if (this->get_parameter("auto_configure").as_bool()) {
    RCLCPP_INFO(this->get_logger(), "Attempting auto-configure...");
    try {
      auto state = this->configure();
      if (state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        RCLCPP_ERROR(
          this->get_logger(), "Auto-configure failed (State ID: %d)",
          state.id());
        return;
      } else {
        RCLCPP_INFO(this->get_logger(), "Auto-configure successful");
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        this->get_logger(), "Auto-configure exception: %s",
        e.what());
      return;
    }
  } else {
    return;
  }
}

CallbackReturn MG400Node::on_configure(const State &)
{
  if (!this->loadAndValidateServoParameters()) {
    return CallbackReturn::FAILURE;
  }

  this->mg400_connected_pub_ =
    this->create_publisher<std_msgs::msg::Bool>(
    "mg400_connected", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  this->mg400_connected_pub_->publish(std_msgs::msg::Bool().set__data(false));

  this->ip_address_ = this->get_parameter("ip_address").as_string();
  this->interface_ =
    std::make_shared<mg400_interface::MG400Interface>(this->ip_address_);
  if (!this->interface_->configure(this->get_parameter("prefix").as_string())) {
    RCLCPP_ERROR(this->get_logger(), "Failed to configure MG400Interface.");
    return CallbackReturn::FAILURE;
  }

  try {
    this->servo_control_ros_interface_ = std::make_unique<ServoControlRosInterface>(
      *this, this->interface_->getControlStateManagerShared(),
      this->servo_safety_violation_state_, this->servo_operational_error_state_);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to configure Servo ROS interface: %s", error.what());
    return CallbackReturn::FAILURE;
  }

  this->dashboard_api_loader_ =
    std::make_shared<mg400_plugin_base::DashboardApiLoader>();
  this->dashboard_api_loader_->loadPlugins(
    this->get_parameter("dashboard_api_plugins").as_string_array());
  this->dashboard_api_loader_->configure(
    this->interface_->dashboard_commander,
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_services_interface(),
    this->get_node_waitables_interface(),
    this->interface_);
  this->dashboard_api_loader_->showPluginInfo(
    this->get_node_logging_interface());

  this->motion_api_loader_ =
    std::make_shared<mg400_plugin_base::MotionApiLoader>();
  this->motion_api_loader_->loadPlugins(
    this->get_parameter("motion_api_plugins").as_string_array());
  this->motion_api_loader_->configure(
    this->interface_->motion_commander,
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_services_interface(),
    this->get_node_waitables_interface(),
    this->interface_);
  this->motion_api_loader_->showPluginInfo(
    this->get_node_logging_interface());

  this->joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::SystemDefaultsQoS());
  this->realtime_feedback_pub_ =
    this->create_publisher<mg400_msgs::msg::RealtimeFeedback>(
    "realtime_feedback", rclcpp::SensorDataQoS());
  this->robot_mode_pub_ = this->create_publisher<mg400_msgs::msg::RobotMode>(
    "robot_mode", rclcpp::SensorDataQoS());
  this->error_id_pub_ = this->create_publisher<mg400_msgs::msg::ErrorID>(
    "error_id", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
  this->interface_->realtime_tcp_interface->setRealtimeDataCallback(
    [this](const mg400_interface::RealTimeData & data) {this->onRealtimeData(data);});

  this->connection_interrupted_ = false;
  this->interface_active_ = false;

  if (this->get_parameter("auto_connect").as_bool()) {
    RCLCPP_INFO(this->get_logger(), "Try connecting to MG400 at %s ...", this->ip_address_.c_str());
    this->connect_timer_ = this->create_wall_timer(0s, [this]() {this->activate();});
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn MG400Node::on_activate(const State &)
{
  this->connect_timer_.reset();

  if (!this->interface_->activate()) {
    this->interface_active_ = false;
    RCLCPP_WARN(this->get_logger(), "Failed to connect to MG400 at %s", this->ip_address_.c_str());
    if (this->get_parameter("auto_connect").as_bool() && this->connection_interrupted_) {
      RCLCPP_INFO(this->get_logger(), "Try reconnecting in 5 seconds ...");
      this->connect_timer_ = this->create_wall_timer(5s, [this]() {this->activate();});
    }
    return CallbackReturn::FAILURE;
  }

  this->interface_active_ = true;
  if (!this->createServoSession()) {
    this->interface_->deactivate();
    this->interface_active_ = false;
    return CallbackReturn::FAILURE;
  }

  this->runTimer();
  this->connection_interrupted_ = false;

  RCLCPP_INFO(this->get_logger(), "Connected to MG400 at %s", this->ip_address_.c_str());
  this->mg400_connected_pub_->publish(std_msgs::msg::Bool().set__data(true));
  this->servo_control_ros_interface_->publishControlState(true);
  return CallbackReturn::SUCCESS;
}

CallbackReturn MG400Node::on_deactivate(const State &)
{
  if (!this->stopAndDestroyServoSession("Lifecycle deactivate")) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Refusing to disconnect MG400 because Servo safe stop was not confirmed");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_WARN(this->get_logger(), "Disconnected from MG400 at %s", this->ip_address_.c_str());
  this->mg400_connected_pub_->publish(std_msgs::msg::Bool().set__data(false));

  this->cancelTimer();
  if (this->interface_active_) {
    this->interface_->deactivate();
    this->interface_active_ = false;
  }
  this->servo_control_ros_interface_->publishControlState(true);

  if (this->get_parameter("auto_connect").as_bool() && this->connection_interrupted_) {
    RCLCPP_INFO(this->get_logger(), "Try reconnecting in 5 seconds ...");
    this->connect_timer_ = this->create_wall_timer(5s, [this]() {this->activate();});
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn MG400Node::on_cleanup(const State &)
{
  if (!this->stopAndDestroyServoSession("Lifecycle cleanup")) {
    return CallbackReturn::FAILURE;
  }
  this->cancelTimer();
  if (this->interface_active_) {
    this->interface_->deactivate();
    this->interface_active_ = false;
  }
  this->destroyRosEntities();
  this->interface_.reset();
  this->connect_timer_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn MG400Node::on_shutdown(const State &)
{
  if (!this->stopAndDestroyServoSession("Lifecycle shutdown")) {
    return CallbackReturn::FAILURE;
  }
  this->cancelTimer();
  if (this->interface_active_) {
    this->interface_->deactivate();
    this->interface_active_ = false;
  }
  this->destroyRosEntities();
  this->interface_.reset();
  this->connect_timer_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn MG400Node::on_error(const State &)
{
  if (!this->stopAndDestroyServoSession("Lifecycle error")) {
    return CallbackReturn::FAILURE;
  }
  this->cancelTimer();
  if (this->interface_active_) {
    this->interface_->deactivate();
    this->interface_active_ = false;
  }
  this->destroyRosEntities();
  this->interface_.reset();
  this->connect_timer_.reset();
  return CallbackReturn::SUCCESS;
}

void MG400Node::onRealtimeData(const mg400_interface::RealTimeData & data)
{
  if (!this->realtime_feedback_pub_) {
    return;
  }

  auto message = std::make_unique<mg400_msgs::msg::RealtimeFeedback>(
    mg400_interface::toRealtimeFeedbackMessage(data));
  message->header.stamp = this->now();
  this->realtime_feedback_pub_->publish(std::move(message));
}

void MG400Node::onJointStateTimer()
{
  if (!this->interface_->ok()) {
    return;
  }

  static std::array<double, 4> joint_states;
  this->interface_->realtime_tcp_interface->getCurrentJointStates(joint_states);

  this->joint_state_pub_->publish(
    mg400_interface::JointHandler::getJointState(
      joint_states,
      this->interface_->realtime_tcp_interface->frame_id_prefix));
}

void MG400Node::onRobotModeTimer()
{
  if (!this->interface_->ok()) {
    return;
  }

  auto msg = std::make_unique<mg400_msgs::msg::RobotMode>();
  uint64_t mode;
  if (this->interface_->realtime_tcp_interface->getRobotMode(mode)) {
    msg->robot_mode = mode;
    this->robot_mode_pub_->publish(std::move(msg));
  }
}

void MG400Node::onErrorTimer()
{
  if (!this->interface_->ok()) {
    return;
  }

  if (!this->interface_->realtime_tcp_interface->isRobotMode(
      mg400_msgs::msg::RobotMode::ERROR))
  {
    return;
  }

  auto msg = std::make_unique<mg400_msgs::msg::ErrorID>();
  try {
    std::stringstream ss;
    const auto error_ids =
      this->interface_->dashboard_commander->getErrorId();
    if (!error_ids.at(0).empty()) {
      msg->controller.ids = error_ids.at(0);
      ss << "Controller and Algorithm:" << std::endl;
      for (auto error_id : error_ids.at(0)) {
        const auto message =
          this->interface_->controller_error_msg_generator->get(error_id);
        ss << "\t" << message << std::endl;
      }
    }
    for (size_t i = 1; i <= msg->servo.size(); ++i) {
      if (error_ids.at(i).empty()) {
        continue;
      }
      msg->servo.at(i).ids = error_ids.at(i);
      ss << "Servo" << i << ":" << std::endl;
      for (auto error_id : error_ids.at(i)) {
        const auto message =
          this->interface_->servo_error_msg_generator->get(error_id);
        ss << "\t" << message << std::endl;
      }
    }
    RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
    this->error_id_pub_->publish(std::move(msg));
  } catch (const std::runtime_error & ex) {
    RCLCPP_ERROR(this->get_logger(), ex.what());
    msg->controller.ids.emplace_back(-1);
    this->error_id_pub_->publish(std::move(msg));
  } catch (const std::out_of_range & ex) {
    RCLCPP_ERROR(this->get_logger(), "Out of range %s", ex.what());
    msg->controller.ids.emplace_back(-1);
    this->error_id_pub_->publish(std::move(msg));
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Cannot parse response from MG400");
    msg->controller.ids.emplace_back(-1);
    this->error_id_pub_->publish(std::move(msg));
  }
}

void MG400Node::onInterfaceCheckTimer()
{
  const auto previous_manager_snapshot =
    this->interface_->getControlStateManager().getSnapshot();
  const bool servo_was_active =
    previous_manager_snapshot.control_state ==
    mg400_interface::ControlStateManager::State::SERVO_J ||
    (this->servo_control_ros_interface_ &&
    this->servo_control_ros_interface_->hasActiveServoLease());
  if (!this->interface_->ok()) {
    RCLCPP_ERROR(this->get_logger(), "Connection to MG400 was interrupted");
    this->connection_interrupted_ = true;
    if (servo_was_active && this->servo_operational_error_state_->reportError(
        mg400_interface::ServoOperationalErrorCode::REALTIME_CONNECTION_LOST,
        "The MG400 connection was lost during Servo control"))
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "REALTIME_CONNECTION_LOST: MG400 connection was lost during Servo control; "
        "lease_id=%lu",
        previous_manager_snapshot.lease_id);
    }
    if (this->servo_control_ros_interface_) {
      this->servo_control_ros_interface_->retireSessionIfLeaseLost(
        "Connection interruption revoked the Servo lease");
    }
    this->deactivate();
    return;
  }

  if (this->servo_control_ros_interface_ &&
    this->servo_control_ros_interface_->retireSessionIfLeaseLost(
      "RobotMode or connection epoch revoked the Servo lease"))
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Retired a stale Servo Session without issuing ResetRobot");
  }

  if (this->servo_control_ros_interface_ &&
    !this->servo_control_ros_interface_->hasSession() &&
    this->interface_->getControlStateManager().getState() ==
    mg400_interface::ControlStateManager::State::IDLE)
  {
    if (!this->createServoSession()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create a fresh Servo connection epoch");
    }
  }
  if (this->servo_control_ros_interface_) {
    this->servo_control_ros_interface_->publishControlState();
  }
}

void MG400Node::runTimer()
{
  this->joint_state_timer_ = this->create_wall_timer(
    10ms, std::bind(&MG400Node::onJointStateTimer, this));
  this->robot_mode_timer_ = this->create_wall_timer(
    100ms, std::bind(&MG400Node::onRobotModeTimer, this));
  this->error_timer_ = this->create_wall_timer(
    500ms, std::bind(&MG400Node::onErrorTimer, this));
  this->interface_check_timer_ = this->create_wall_timer(
    100ms, std::bind(&MG400Node::onInterfaceCheckTimer, this));
}

void MG400Node::cancelTimer()
{
  this->joint_state_timer_.reset();
  this->robot_mode_timer_.reset();
  this->error_timer_.reset();
  this->interface_check_timer_.reset();
}

bool MG400Node::loadAndValidateServoParameters()
{
  const auto send_period_ms = this->get_parameter("servo.send_period_ms").as_int();
  const auto target_watchdog_ms =
    this->get_parameter("servo.target_watchdog_ms").as_int();
  const auto stop_timeout_ms = this->get_parameter("servo.stop_timeout_ms").as_int();
  const auto stop_confirmation_poll_ms =
    this->get_parameter("servo.stop_confirmation_poll_ms").as_int();
  const auto feedback_timeout_ms =
    this->get_parameter("servo.safety.feedback_timeout_ms").as_int();

  const std::array<std::pair<const char *, std::int64_t>, 5> values{{
    {"servo.send_period_ms", send_period_ms},
    {"servo.target_watchdog_ms", target_watchdog_ms},
    {"servo.stop_timeout_ms", stop_timeout_ms},
    {"servo.stop_confirmation_poll_ms", stop_confirmation_poll_ms},
    {"servo.safety.feedback_timeout_ms", feedback_timeout_ms}}};
  for (const auto & parameter : values) {
    if (parameter.second <= 0) {
      RCLCPP_ERROR(
        this->get_logger(), "Parameter %s must be greater than zero", parameter.first);
      return false;
    }
  }

  const std::array<std::pair<const char *, double>, 2> safety_thresholds{{
    {"servo.safety.max_initial_joint_distance_rad",
      this->get_parameter("servo.safety.max_initial_joint_distance_rad").as_double()},
    {"servo.safety.max_joint_step_rad",
      this->get_parameter("servo.safety.max_joint_step_rad").as_double()}}};
  for (const auto & parameter : safety_thresholds) {
    if (!std::isfinite(parameter.second) || parameter.second <= 0.0) {
      RCLCPP_ERROR(
        this->get_logger(), "Parameter %s must be positive and finite", parameter.first);
      return false;
    }
  }

  this->servo_session_options_.send_period = std::chrono::milliseconds(send_period_ms);
  this->servo_session_options_.target_watchdog_timeout =
    std::chrono::milliseconds(target_watchdog_ms);
  this->servo_session_options_.feedback_timeout =
    std::chrono::milliseconds(feedback_timeout_ms);
  this->servo_session_options_.max_initial_joint_distance_rad = safety_thresholds[0].second;
  this->servo_session_options_.max_joint_step_rad = safety_thresholds[1].second;
  const auto logger = this->get_logger();
  this->servo_session_options_.safety_log_callback =
    [logger](const std::string & message) {RCLCPP_ERROR(logger, "%s", message.c_str());};
  const auto clock = this->get_clock();
  this->servo_session_options_.operational_log_callback =
    [logger, clock](
    const mg400_interface::ServoOperationalErrorCode code,
    const std::string & message)
    {
      if (code == mg400_interface::ServoOperationalErrorCode::WATCHDOG_TIMEOUT) {
        RCLCPP_WARN_THROTTLE(logger, *clock, 5000, "%s", message.c_str());
      } else {
        RCLCPP_ERROR_THROTTLE(logger, *clock, 5000, "%s", message.c_str());
      }
    };
  this->servo_session_options_.stop_confirmation_timeout =
    std::chrono::milliseconds(stop_timeout_ms);
  this->servo_stop_options_.confirmation_poll_period =
    std::chrono::milliseconds(stop_confirmation_poll_ms);
  return true;
}

bool MG400Node::createServoSession()
{
  if (!this->interface_ || !this->servo_control_ros_interface_) {
    RCLCPP_ERROR(this->get_logger(), "Servo dependencies are not configured");
    return false;
  }
  if (this->servo_control_ros_interface_->hasSession()) {
    this->servo_control_ros_interface_->setLifecycleActive(true);
    return true;
  }

  try {
    std::weak_ptr<mg400_interface::MG400Interface> weak_interface(this->interface_);
    auto stop_strategy = std::make_shared<mg400_interface::ResetRobotStopStrategy>(
      [weak_interface]() {
        const auto interface = weak_interface.lock();
        if (!interface || !interface->dashboard_commander) {
          throw std::runtime_error("MG400 dashboard connection epoch is no longer available");
        }
        interface->dashboard_commander->resetRobot();
      },
      [weak_interface](std::uint64_t & robot_mode) {
        const auto interface = weak_interface.lock();
        return interface && interface->realtime_tcp_interface &&
        interface->realtime_tcp_interface->getRobotMode(robot_mode);
      },
      this->servo_stop_options_);
    auto session = std::make_shared<mg400_interface::ServoControlSession>(
      this->interface_->getControlStateManagerShared(),
      this->interface_->motion_commander,
      std::move(stop_strategy),
      this->servo_control_ros_interface_->getSafetyViolationStateShared(),
      this->servo_control_ros_interface_->getOperationalErrorStateShared(),
      this->interface_->realtime_tcp_interface->getServoFeedbackStateShared(),
      this->servo_session_options_);
    this->servo_control_ros_interface_->installSession(std::move(session));
    return true;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create Servo Session: %s", error.what());
    return false;
  }
}

bool MG400Node::stopAndDestroyServoSession(const std::string & reason)
{
  if (!this->servo_control_ros_interface_) {
    return true;
  }

  const auto stopped = this->servo_control_ros_interface_->stopForLifecycle(reason);
  if (!stopped.success) {
    // Refreshing status is safe and distinguishes a fail-closed live lease
    // from an epoch already revoked by communication loss.
    if (this->interface_ && this->interface_active_) {
      static_cast<void>(this->interface_->ok());
    }
    if (this->servo_control_ros_interface_->retireSessionIfLeaseLost(
        reason + ": stale Session retired after stop failure"))
    {
      return true;
    }
    this->servo_control_ros_interface_->publishControlState(true);
    RCLCPP_ERROR(
      this->get_logger(), "Servo safe stop failed and the live Lease is retained: %s",
      stopped.message.c_str());
    return false;
  }

  if (this->servo_control_ros_interface_->hasSession()) {
    this->servo_control_ros_interface_->clearSession();
  }
  return true;
}

void MG400Node::destroyRosEntities()
{
  // Servo callbacks and Session references are destroyed before the
  // MG400Interface that owns the TCP objects referenced by MotionCommander.
  this->servo_control_ros_interface_.reset();
  this->dashboard_api_loader_.reset();
  this->motion_api_loader_.reset();
  if (this->interface_ && this->interface_->realtime_tcp_interface) {
    this->interface_->realtime_tcp_interface->setRealtimeDataCallback({});
  }
  this->joint_state_pub_.reset();
  this->realtime_feedback_pub_.reset();
  this->robot_mode_pub_.reset();
  this->error_id_pub_.reset();
  this->mg400_connected_pub_.reset();
}

}  // namespace mg400_node
