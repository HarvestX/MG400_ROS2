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

#include <cmath>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace mg400_node
{
using namespace std::chrono_literals;   // NOLINT

namespace
{
constexpr int DEFAULT_SERVO_COMMAND_TIMEOUT_MS = 100;
constexpr double QUATERNION_NORM_MIN = 1.0e-12;

bool validateFinite(
  const double value, const char * name, std::string & reason)
{
  if (!std::isfinite(value)) {
    reason = std::string(name) + " must be finite";
    return false;
  }
  return true;
}

bool validateServoJCommand(
  const mg400_msgs::msg::ServoJ & msg, std::string & reason)
{
  for (size_t i = 0; i < msg.joint_angles.size(); ++i) {
    std::ostringstream name;
    name << "joint_angles[" << i << "]";
    if (!validateFinite(msg.joint_angles[i], name.str().c_str(), reason)) {
      return false;
    }
  }
  return true;
}

bool validateServoPCommand(
  const mg400_msgs::msg::ServoP & msg, const std::string & expected_frame_id,
  double & tool_rotation, std::string & reason)
{
  if (msg.pose.header.frame_id != expected_frame_id) {
    reason = "pose.header.frame_id must be " + expected_frame_id;
    return false;
  }

  const auto & position = msg.pose.pose.position;
  if (!validateFinite(position.x, "pose.position.x", reason) ||
    !validateFinite(position.y, "pose.position.y", reason) ||
    !validateFinite(position.z, "pose.position.z", reason))
  {
    return false;
  }

  const auto & orientation = msg.pose.pose.orientation;
  if (!validateFinite(orientation.x, "pose.orientation.x", reason) ||
    !validateFinite(orientation.y, "pose.orientation.y", reason) ||
    !validateFinite(orientation.z, "pose.orientation.z", reason) ||
    !validateFinite(orientation.w, "pose.orientation.w", reason))
  {
    return false;
  }

  const double norm =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  if (norm < QUATERNION_NORM_MIN) {
    reason = "pose.orientation quaternion norm is too small";
    return false;
  }

  tf2::Quaternion quaternion(
    orientation.x, orientation.y, orientation.z, orientation.w);
  quaternion.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

  if (!validateFinite(roll, "roll", reason) ||
    !validateFinite(pitch, "pitch", reason) ||
    !validateFinite(yaw, "yaw", reason))
  {
    return false;
  }

  tool_rotation = yaw;
  return true;
}
}  // namespace

MG400Node::MG400Node(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("mg400_node", options)
  , connection_interrupted_(false)
  , servo_command_timeout_ms_(DEFAULT_SERVO_COMMAND_TIMEOUT_MS)
  , servo_mode_active_(false)
  , servo_command_type_(ServoCommandType::NONE)
  , last_valid_servo_command_stamp_(0, 0, RCL_ROS_TIME)
  , has_realtime_feedback_(false)
  , latest_robot_mode_(mg400_msgs::msg::RobotMode::INVALID)
  , latest_enable_status_(0)
  , latest_error_status_(0)
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
  this->declare_parameter<int>("servo_command_timeout_ms", DEFAULT_SERVO_COMMAND_TIMEOUT_MS);
  this->declare_parameter<int>("servo_expected_period_ms", 30);
  this->declare_parameter<bool>("servo_qos_reliable", false);
  this->declare_parameter<bool>("servo_exit_on_dashboard_stop_command", true);

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
  if (this->interface_) {
    this->interface_->deactivate();
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

CallbackReturn MG400Node::on_configure(const State & /*previous_state*/)
{
  const auto servo_command_timeout_ms =
    this->get_parameter("servo_command_timeout_ms").as_int();
  if (servo_command_timeout_ms <= 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "servo_command_timeout_ms must be positive. Using default %d ms.",
      DEFAULT_SERVO_COMMAND_TIMEOUT_MS);
    this->servo_command_timeout_ms_ = DEFAULT_SERVO_COMMAND_TIMEOUT_MS;
  } else {
    this->servo_command_timeout_ms_ =
      static_cast<int>(servo_command_timeout_ms);
  }

  {
    std::lock_guard<std::mutex> lock(this->servo_mode_mutex_);
    this->servo_mode_active_ = false;
    this->servo_command_type_ = ServoCommandType::NONE;
    this->has_realtime_feedback_ = false;
    this->latest_robot_mode_ = mg400_msgs::msg::RobotMode::INVALID;
    this->latest_enable_status_ = 0;
    this->latest_error_status_ = 0;
  }

  this->mg400_connected_pub_ =
    this->create_publisher<std_msgs::msg::Bool>(
    "mg400_connected", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  this->mg400_connected_pub_->publish(std_msgs::msg::Bool().set__data(false));
  this->servo_mode_enabled_pub_ =
    this->create_publisher<std_msgs::msg::Bool>(
    "servo_mode_enabled", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  this->publishServoModeEnabled(false);

  this->ip_address_ = this->get_parameter("ip_address").as_string();
  this->interface_ =
    std::make_shared<mg400_interface::MG400Interface>(this->ip_address_);
  if (!this->interface_->configure(this->get_parameter("prefix").as_string())) {
    RCLCPP_ERROR(this->get_logger(), "Failed to configure MG400Interface.");
    return CallbackReturn::FAILURE;
  }
  this->interface_->setServoModeActive(false);
  this->interface_->setServoExitOnDashboardStopCommand(
    this->get_parameter("servo_exit_on_dashboard_stop_command").as_bool());
  this->interface_->setServoModeExitCallback(
    [this](const std::string & reason, const bool warn) {
      this->exitServoMode(reason, warn);
    });

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

  auto servo_command_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile();
  if (this->get_parameter("servo_qos_reliable").as_bool()) {
    servo_command_qos.reliable();
  } else {
    servo_command_qos.best_effort();
  }

  this->servo_j_sub_ =
    this->create_subscription<mg400_msgs::msg::ServoJ>(
    "servo_j", servo_command_qos,
    std::bind(&MG400Node::onServoJ, this, std::placeholders::_1));
  this->servo_p_sub_ =
    this->create_subscription<mg400_msgs::msg::ServoP>(
    "servo_p", servo_command_qos,
    std::bind(&MG400Node::onServoP, this, std::placeholders::_1));
  this->servo_mode_srv_ =
    this->create_service<mg400_msgs::srv::ServoMode>(
    "servo_mode",
    std::bind(
      &MG400Node::onServoModeService, this,
      std::placeholders::_1, std::placeholders::_2));

  this->connection_interrupted_ = false;

  if (this->get_parameter("auto_connect").as_bool()) {
    RCLCPP_INFO(this->get_logger(), "Try connecting to MG400 at %s ...", this->ip_address_.c_str());
    this->connect_timer_ = this->create_wall_timer(0s, [this]() {this->activate();});
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn MG400Node::on_activate(const State & /*previous_state*/)
{
  this->connect_timer_.reset();

  if (!this->interface_->activate()) {
    RCLCPP_WARN(this->get_logger(), "Failed to connect to MG400 at %s", this->ip_address_.c_str());
    if (this->get_parameter("auto_connect").as_bool() && this->connection_interrupted_) {
      RCLCPP_INFO(this->get_logger(), "Try reconnecting in 5 seconds ...");
      this->connect_timer_ = this->create_wall_timer(5s, [this]() {this->activate();});
    }
    return CallbackReturn::FAILURE;
  }

  this->runTimer();
  this->connection_interrupted_ = false;

  RCLCPP_INFO(this->get_logger(), "Connected to MG400 at %s", this->ip_address_.c_str());
  this->mg400_connected_pub_->publish(std_msgs::msg::Bool().set__data(true));
  this->publishServoModeEnabled(false);
  return CallbackReturn::SUCCESS;
}

CallbackReturn MG400Node::on_deactivate(const State & /*previous_state*/)
{
  this->exitServoMode("Lifecycle deactivate");
  RCLCPP_WARN(this->get_logger(), "Disconnected from MG400 at %s", this->ip_address_.c_str());
  this->mg400_connected_pub_->publish(std_msgs::msg::Bool().set__data(false));

  this->cancelTimer();
  this->interface_->deactivate();

  if (this->get_parameter("auto_connect").as_bool() && this->connection_interrupted_) {
    RCLCPP_INFO(this->get_logger(), "Try reconnecting in 5 seconds ...");
    this->connect_timer_ = this->create_wall_timer(5s, [this]() {this->activate();});
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn MG400Node::on_cleanup(const State & /*previous_state*/)
{
  this->exitServoMode("Lifecycle cleanup");
  this->mg400_connected_pub_.reset();
  this->servo_mode_enabled_pub_.reset();
  this->servo_j_sub_.reset();
  this->servo_p_sub_.reset();
  this->servo_mode_srv_.reset();
  this->dashboard_api_loader_.reset();
  this->motion_api_loader_.reset();
  this->joint_state_pub_.reset();
  this->realtime_feedback_pub_.reset();
  this->robot_mode_pub_.reset();
  this->error_id_pub_.reset();
  this->interface_.reset();
  this->connect_timer_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn MG400Node::on_shutdown(const State & /*previous_state*/)
{
  this->exitServoMode("Lifecycle shutdown");
  this->cancelTimer();
  this->dashboard_api_loader_.reset();
  this->motion_api_loader_.reset();
  this->joint_state_pub_.reset();
  this->realtime_feedback_pub_.reset();
  this->robot_mode_pub_.reset();
  this->error_id_pub_.reset();
  this->mg400_connected_pub_.reset();
  this->servo_mode_enabled_pub_.reset();
  this->servo_j_sub_.reset();
  this->servo_p_sub_.reset();
  this->servo_mode_srv_.reset();
  this->interface_.reset();
  this->connect_timer_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn MG400Node::on_error(const State & /*previous_state*/)
{
  this->exitServoMode("Lifecycle error", true);
  this->cancelTimer();
  this->dashboard_api_loader_.reset();
  this->motion_api_loader_.reset();
  this->joint_state_pub_.reset();
  this->realtime_feedback_pub_.reset();
  this->robot_mode_pub_.reset();
  this->error_id_pub_.reset();
  this->mg400_connected_pub_.reset();
  this->servo_mode_enabled_pub_.reset();
  this->servo_j_sub_.reset();
  this->servo_p_sub_.reset();
  this->servo_mode_srv_.reset();
  this->interface_.reset();
  this->connect_timer_.reset();
  return CallbackReturn::SUCCESS;
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

void MG400Node::onRealtimeFeedbackTimer()
{
  if (!this->interface_->ok()) {
    return;
  }

  auto data = mg400_interface::RealTimeData();
  if (this->interface_->realtime_tcp_interface->getRealtimeData(data)) {
    this->updateServoModeRealtimeFeedback(data);
    this->publishRealtimeFeedback(data);
  }
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
  this->exitServoMode("Robot mode ERROR", true);

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
    RCLCPP_ERROR(this->get_logger(), "%s", ss.str().c_str());
    this->error_id_pub_->publish(std::move(msg));
  } catch (const std::runtime_error & ex) {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
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
  if (!this->interface_->ok()) {
    RCLCPP_ERROR(this->get_logger(), "Connection to MG400 was interrupted");
    this->exitServoMode("MG400 connection interrupted", true);
    this->connection_interrupted_ = true;
    this->deactivate();
  }
}

void MG400Node::onServoCommandTimeoutTimer()
{
  std::string reason;
  bool should_exit = false;

  {
    std::lock_guard<std::mutex> lock(this->servo_mode_mutex_);
    if (!this->servo_mode_active_) {
      return;
    }

    const auto elapsed =
      this->get_clock()->now() - this->last_valid_servo_command_stamp_;
    const auto timeout_ns =
      static_cast<int64_t>(this->servo_command_timeout_ms_) * 1000000LL;
    if (elapsed.nanoseconds() > timeout_ns) {
      std::ostringstream ss;
      ss << "No valid servo command received for "
         << this->servo_command_timeout_ms_ << " ms";
      reason = ss.str();
      should_exit = true;
    }
  }

  if (should_exit) {
    this->exitServoMode(reason, true);
  }
}

void MG400Node::runTimer()
{
  this->joint_state_timer_ = this->create_wall_timer(
    10ms, std::bind(&MG400Node::onJointStateTimer, this));
  this->realtime_feedback_timer_ = this->create_wall_timer(
    10ms, std::bind(&MG400Node::onRealtimeFeedbackTimer, this));
  this->robot_mode_timer_ = this->create_wall_timer(
    100ms, std::bind(&MG400Node::onRobotModeTimer, this));
  this->error_timer_ = this->create_wall_timer(
    500ms, std::bind(&MG400Node::onErrorTimer, this));
  this->interface_check_timer_ = this->create_wall_timer(
    100ms, std::bind(&MG400Node::onInterfaceCheckTimer, this));
  this->servo_command_timeout_timer_ = this->create_wall_timer(
    10ms, std::bind(&MG400Node::onServoCommandTimeoutTimer, this));
}

void MG400Node::cancelTimer()
{
  this->joint_state_timer_.reset();
  this->realtime_feedback_timer_.reset();
  this->robot_mode_timer_.reset();
  this->error_timer_.reset();
  this->interface_check_timer_.reset();
  this->servo_command_timeout_timer_.reset();
}

void MG400Node::updateServoModeRealtimeFeedback(
  const mg400_interface::RealTimeData & data)
{
  std::string reason;
  bool should_exit = false;

  {
    std::lock_guard<std::mutex> lock(this->servo_mode_mutex_);
    this->has_realtime_feedback_ = true;
    this->latest_robot_mode_ = data.robot_mode;
    this->latest_enable_status_ = data.enable_status;
    this->latest_error_status_ = data.error_status;

    if (!this->servo_mode_active_) {
      return;
    }

    if (data.robot_mode == mg400_msgs::msg::RobotMode::ERROR) {
      reason = "Robot mode ERROR";
      should_exit = true;
    } else if (data.error_status != 0) {
      reason = "Realtime feedback error_status is non-zero";
      should_exit = true;
    } else if (data.enable_status == 0) {
      reason = "Realtime feedback enable_status is disabled";
      should_exit = true;
    }
  }

  if (should_exit) {
    this->exitServoMode(reason, true);
  }
}

void MG400Node::onServoModeService(
  const std::shared_ptr<mg400_msgs::srv::ServoMode::Request> request,
  std::shared_ptr<mg400_msgs::srv::ServoMode::Response> response)
{
  if (request->enable) {
    int32_t error_id = 0;
    std::string message;
    if (this->enterServoMode(message, error_id)) {
      response->error_id = 0;
      response->message = message;
      return;
    }

    response->error_id = error_id;
    response->message = message;
    RCLCPP_WARN(this->get_logger(), "ServoMode enter rejected: %s", message.c_str());
    return;
  }

  this->exitServoMode("ServoMode service exit request");
  response->error_id = 0;
  response->message = "ServoMode inactive";
}

void MG400Node::onServoJ(const mg400_msgs::msg::ServoJ::SharedPtr msg)
{
  this->warnUnusedServoOptions("servo_j", msg->t, msg->aheadtime, msg->gain);

  {
    std::lock_guard<std::mutex> lock(this->servo_mode_mutex_);
    if (!this->servo_mode_active_) {
      return;
    }
  }

  std::string reason;
  if (!validateServoJCommand(*msg, reason)) {
    this->warnInvalidServoCommand("servo_j", reason);
    return;
  }

  if (!this->interface_ || !this->interface_->ok()) {
    this->exitServoMode("MG400 interface is not connected", true);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(this->servo_mode_mutex_);
    if (!this->servo_mode_active_) {
      return;
    }
    if (this->servo_command_type_ == ServoCommandType::NONE) {
      this->servo_command_type_ = ServoCommandType::SERVO_J;
    } else if (this->servo_command_type_ != ServoCommandType::SERVO_J) {
      this->warnInvalidServoCommand(
        "servo_j", "ServoMode session already uses servo_p");
      return;
    }
  }

  try {
    this->interface_->motion_commander->servoJ(
      msg->joint_angles[0], msg->joint_angles[1],
      msg->joint_angles[2], msg->joint_angles[3],
      msg->t, msg->aheadtime, msg->gain);
  } catch (const std::exception & ex) {
    this->warnInvalidServoCommand("servo_j", ex.what());
    return;
  } catch (...) {
    this->warnInvalidServoCommand("servo_j", "Unknown ServoJ send failure");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(this->servo_mode_mutex_);
    if (this->servo_mode_active_ &&
      this->servo_command_type_ == ServoCommandType::SERVO_J)
    {
      this->last_valid_servo_command_stamp_ = this->get_clock()->now();
    }
  }
}

void MG400Node::onServoP(const mg400_msgs::msg::ServoP::SharedPtr msg)
{
  this->warnUnusedServoOptions("servo_p", msg->t, msg->aheadtime, msg->gain);

  {
    std::lock_guard<std::mutex> lock(this->servo_mode_mutex_);
    if (!this->servo_mode_active_) {
      return;
    }
  }

  double tool_rotation = 0.0;
  std::string reason;
  if (!validateServoPCommand(*msg, this->getServoOriginFrame(), tool_rotation, reason)) {
    this->warnInvalidServoCommand("servo_p", reason);
    return;
  }

  if (!this->interface_ || !this->interface_->ok()) {
    this->exitServoMode("MG400 interface is not connected", true);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(this->servo_mode_mutex_);
    if (!this->servo_mode_active_) {
      return;
    }
    if (this->servo_command_type_ == ServoCommandType::NONE) {
      this->servo_command_type_ = ServoCommandType::SERVO_P;
    } else if (this->servo_command_type_ != ServoCommandType::SERVO_P) {
      this->warnInvalidServoCommand(
        "servo_p", "ServoMode session already uses servo_j");
      return;
    }
  }

  const auto & position = msg->pose.pose.position;
  try {
    // MG400's single 4-axis tool angle is carried as yaw in ROS pose,
    // but Dobot TCP ServoP expects it in the Rx slot.
    this->interface_->motion_commander->servoP(
      position.x, position.y, position.z,
      tool_rotation, 0.0, 0.0,
      msg->t, msg->aheadtime, msg->gain);
  } catch (const std::exception & ex) {
    this->warnInvalidServoCommand("servo_p", ex.what());
    return;
  } catch (...) {
    this->warnInvalidServoCommand("servo_p", "Unknown ServoP send failure");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(this->servo_mode_mutex_);
    if (this->servo_mode_active_ &&
      this->servo_command_type_ == ServoCommandType::SERVO_P)
    {
      this->last_valid_servo_command_stamp_ = this->get_clock()->now();
    }
  }
}

bool MG400Node::enterServoMode(std::string & message, int32_t & error_id)
{
  {
    std::lock_guard<std::mutex> lock(this->servo_mode_mutex_);
    if (this->servo_mode_active_) {
      if (this->interface_) {
        this->interface_->setServoModeActive(true);
      }
      message = "ServoMode already active";
      error_id = 0;
      return true;
    }
  }

  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    message = "Lifecycle node is not active";
    error_id = -1;
    return false;
  }

  if (!this->interface_ || !this->interface_->ok()) {
    message = "MG400Interface is not connected";
    error_id = -2;
    return false;
  }

  bool has_realtime_feedback = false;
  uint64_t robot_mode = mg400_msgs::msg::RobotMode::INVALID;
  uint8_t enable_status = 0;
  uint8_t error_status = 0;
  {
    std::lock_guard<std::mutex> lock(this->servo_mode_mutex_);
    has_realtime_feedback = this->has_realtime_feedback_;
    robot_mode = this->latest_robot_mode_;
    enable_status = this->latest_enable_status_;
    error_status = this->latest_error_status_;
  }

  if (!has_realtime_feedback) {
    message = "Realtime feedback has not been received";
    error_id = -3;
    return false;
  }
  if (robot_mode == mg400_msgs::msg::RobotMode::ERROR) {
    message = "Robot mode is ERROR";
    error_id = -4;
    return false;
  }
  if (robot_mode != mg400_msgs::msg::RobotMode::ENABLE) {
    std::ostringstream ss;
    ss << "Robot mode must be ENABLE, current mode is " << robot_mode;
    message = ss.str();
    error_id = -5;
    return false;
  }
  if (enable_status == 0) {
    message = "Realtime feedback enable_status is disabled";
    error_id = -6;
    return false;
  }
  if (error_status != 0) {
    message = "Realtime feedback error_status is non-zero";
    error_id = -7;
    return false;
  }
  if (this->checkDashboardErrors(message)) {
    error_id = -8;
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(this->servo_mode_mutex_);
    this->servo_mode_active_ = true;
    this->servo_command_type_ = ServoCommandType::NONE;
    this->last_valid_servo_command_stamp_ = this->get_clock()->now();
    this->interface_->setServoModeActive(true);
  }

  this->publishServoModeEnabled(true);
  message = "ServoMode active";
  RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
  error_id = 0;
  return true;
}

void MG400Node::exitServoMode(const std::string & reason, bool warn)
{
  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(this->servo_mode_mutex_);
    if (!this->servo_mode_active_) {
      this->servo_command_type_ = ServoCommandType::NONE;
    } else {
      this->servo_mode_active_ = false;
      this->servo_command_type_ = ServoCommandType::NONE;
      changed = true;
    }
  }

  if (this->interface_) {
    this->interface_->setServoModeActive(false);
  }

  if (!changed) {
    return;
  }

  this->publishServoModeEnabled(false);
  if (warn) {
    RCLCPP_WARN(this->get_logger(), "ServoMode exited: %s", reason.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "ServoMode exited: %s", reason.c_str());
  }
}

bool MG400Node::checkDashboardErrors(std::string & message)
{
  try {
    const auto error_ids = this->interface_->dashboard_commander->getErrorId();
    std::ostringstream ss;
    bool has_error = false;
    for (size_t i = 0; i < error_ids.size(); ++i) {
      if (error_ids.at(i).empty()) {
        continue;
      }
      has_error = true;
      if (i == 0) {
        ss << "controller";
      } else {
        ss << "servo" << i;
      }
      ss << "=[";
      for (size_t j = 0; j < error_ids.at(i).size(); ++j) {
        if (j > 0) {
          ss << ",";
        }
        ss << error_ids.at(i).at(j);
      }
      ss << "] ";
    }

    if (has_error) {
      message = "Dobot reports active errors: " + ss.str();
      return true;
    }
  } catch (const std::exception & ex) {
    message = std::string("Failed to query GetErrorID: ") + ex.what();
    return true;
  } catch (...) {
    message = "Failed to query GetErrorID";
    return true;
  }

  return false;
}

void MG400Node::publishServoModeEnabled(const bool enabled)
{
  if (!this->servo_mode_enabled_pub_) {
    return;
  }

  auto msg = std_msgs::msg::Bool();
  msg.data = enabled;
  this->servo_mode_enabled_pub_->publish(msg);
}

void MG400Node::warnInvalidServoCommand(
  const std::string & topic, const std::string & reason)
{
  RCLCPP_WARN_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "Ignoring %s command: %s", topic.c_str(), reason.c_str());
}

void MG400Node::warnUnusedServoOptions(
  const std::string & topic, const double t,
  const double aheadtime, const double gain)
{
  if (t == 0.0 && aheadtime == 0.0 && gain == 0.0) {
    return;
  }

  RCLCPP_WARN_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "%s command ignores t/aheadtime/gain fields; received t=%g, aheadtime=%g, gain=%g",
    topic.c_str(), t, aheadtime, gain);
}

std::string MG400Node::getServoOriginFrame() const
{
  if (this->interface_ && this->interface_->realtime_tcp_interface) {
    return this->interface_->realtime_tcp_interface->frame_id_prefix +
           "mg400_origin_link";
  }
  return this->get_parameter("prefix").as_string() + "mg400_origin_link";
}

void MG400Node::publishRealtimeFeedback(const mg400_interface::RealTimeData & data)
{
  auto msg = std::make_unique<mg400_msgs::msg::RealtimeFeedback>();
  msg->stamp = this->get_clock()->now();

  msg->message_size = data.message_size;
  msg->digital_inputs = data.digital_inputs;
  msg->digital_outputs = data.digital_outputs;
  msg->robot_mode = data.robot_mode;
  msg->time_stamp = data.time_stamp;
  msg->run_time = data.run_time;
  msg->test_value = data.test_value;
  msg->speed_scaling = data.speed_scaling;
  msg->v_robot = data.v_robot;
  msg->i_robot = data.i_robot;
  msg->program_state = data.program_state;

  for (int i = 0; i < 2; ++i) {
    msg->safety_io_in[i] = data.safety_io_in[i];
    msg->safety_io_out[i] = data.safety_io_out[i];
  }

  for (int i = 0; i < 6; ++i) {
    msg->q_target[i] = data.q_target[i];
    msg->qd_target[i] = data.qd_target[i];
    msg->qdd_target[i] = data.qdd_target[i];
    msg->i_target[i] = data.i_target[i];
    msg->m_target[i] = data.m_target[i];

    msg->q_actual[i] = data.q_actual[i];
    msg->qd_actual[i] = data.qd_actual[i];
    msg->i_actual[i] = data.i_actual[i];

    msg->actual_tcp_force[i] = data.actual_tcp_force[i];
    msg->tool_vector_actual[i] = data.tool_vector_actual[i];
    msg->tcp_speed_actual[i] = data.tcp_speed_actual[i];
    msg->tcp_force[i] = data.tcp_force[i];
    msg->tool_vector_target[i] = data.tool_vector_target[i];
    msg->tcp_speed_target[i] = data.tcp_speed_target[i];
    msg->motor_temperatures[i] = data.motor_temperatures[i];
    msg->joint_modes[i] = data.joint_modes[i];
    msg->v_actual[i] = data.v_actual[i];
    msg->m_actual[i] = data.m_actual[i];
    msg->user_coordinates[i] = data.user_coordinates[i];
    msg->tool_coordinates[i] = data.tool_coordinates[i];
    msg->six_force_value[i] = data.six_force_value[i];
  }

  for (int i = 0; i < 4; ++i) {
    msg->hand_type[i] = data.hand_type[i];
    msg->target_quaternion[i] = data.target_quaternion[i];
    msg->actual_quaternion[i] = data.actual_quaternion[i];
  }

  msg->user = data.user;
  msg->tool = data.tool;
  msg->run_queued_cmd = data.run_queued_cmd;
  msg->pause_cmd_flag = data.pause_cmd_flag;
  msg->velocity_ratio = data.velocity_ratio;
  msg->acceleration_ratio = data.acceleration_ratio;
  msg->xyz_velocity_ratio = data.xyz_velocity_ratio;
  msg->r_velocity_ratio = data.r_velocity_ratio;
  msg->xyz_acceleration_ratio = data.xyz_acceleration_ratio;
  msg->r_acceleration_ratio = data.r_acceleration_ratio;
  msg->brake_status = data.brake_status;
  msg->enable_status = data.enable_status;
  msg->drag_status = data.drag_status;
  msg->running_status = data.running_status;
  msg->error_status = data.error_status;
  msg->jog_status_cr = data.jog_status_cr;
  msg->cr_robot_type = data.cr_robot_type;
  msg->drag_button_signal = data.drag_button_signal;
  msg->enable_button_signal = data.enable_button_signal;
  msg->record_button_signal = data.record_button_signal;
  msg->reappear_button_signal = data.reappear_button_signal;
  msg->jaw_button_signal = data.jaw_button_signal;
  msg->six_force_online = data.six_force_online;
  msg->collision_state = data.collision_state;
  msg->arm_approach_state = data.arm_approach_state;
  msg->j4_approach_state = data.j4_approach_state;
  msg->j5_approach_state = data.j5_approach_state;
  msg->j6_approach_state = data.j6_approach_state;

  msg->vibration_dis_z = data.vibration_dis_z;
  msg->current_command_id = data.current_command_id;
  msg->load = data.load;
  msg->center_x = data.center_x;
  msg->center_y = data.center_y;
  msg->center_z = data.center_z;
  msg->auto_manual_mode = data.auto_manual_mode;
  msg->export_status = data.export_status;
  msg->safety_state = data.safety_state;
  msg->safe_state = data.safe_state;

  this->realtime_feedback_pub_->publish(std::move(msg));
}

}  // namespace mg400_node
