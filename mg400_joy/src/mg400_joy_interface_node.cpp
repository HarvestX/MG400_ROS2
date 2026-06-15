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

#include "mg400_joy/mg400_joy_interface_node.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace mg400_joy
{
namespace
{
using namespace std::chrono_literals;  // NOLINT

constexpr double kMillimetersToMeters = 1.0e-3;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kMaxServoTargetDtSec = 0.1;
constexpr double kControllerPromptPeriodSec = 2.0;
constexpr double kUnusedServoOption = 0.0;
constexpr auto kJoyControlActivePeriod = 1s;

bool timeIsUnset(const rclcpp::Time & time)
{
  return time.nanoseconds() == 0;
}

bool elapsed(const rclcpp::Time & now, const rclcpp::Time & stamp, const double seconds)
{
  if (timeIsUnset(stamp)) {
    return true;
  }
  return (now - stamp).seconds() >= seconds;
}

double clampValue(const double value, const double min_value, const double max_value)
{
  return std::min(std::max(value, min_value), max_value);
}

std::string normalizeControlType(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](const unsigned char c) {return static_cast<char>(std::tolower(c));});
  value.erase(
    std::remove_if(
      value.begin(), value.end(), [](const char c) {
        return c == '_' || c == '-' || c == ' ';
      }),
    value.end());
  return value;
}
}  // namespace

MG400JoyInterfaceNode::MG400JoyInterfaceNode(const rclcpp::NodeOptions & node_options)
: Node("mg400_joy", node_options),
  current_robot_mode_(RobotMode::INVALID),
  servo_mode_enabled_(false),
  has_actual_state_(false),
  current_pose_yaw_(0.0),
  current_joint_state_({0.0, 0.0, 0.0, 0.0}),
  servo_control_type_(ServoControlType::SERVO_J),
  servo_pose_target_initialized_(false),
  servo_joint_target_initialized_(false),
  servo_target_yaw_(0.0),
  servo_joint_target_({0.0, 0.0, 0.0, 0.0}),
  servo_pose_axes_({0.0, 0.0, 0.0, 0.0}),
  servo_joint_axes_({0.0, 0.0, 0.0, 0.0}),
  last_servo_target_update_time_(0, 0, RCL_ROS_TIME),
  last_joy_time_(0, 0, RCL_ROS_TIME),
  last_button_action_time_(0, 0, RCL_ROS_TIME),
  controller_check_stage_(ControllerCheckStage::DISABLED),
  controller_check_start_time_(0, 0, RCL_ROS_TIME),
  last_controller_check_prompt_time_(0, 0, RCL_ROS_TIME),
  controller_check_baseline_ready_(false)
{
  this->current_pose_.orientation.w = 1.0;
  this->servo_target_pose_.orientation.w = 1.0;

  this->joy_mapping_file_ = this->declare_parameter<std::string>("mapping_file", "");
  const auto default_control_type =
    this->declare_parameter<std::string>("servo_control_type", "ServoJ");
  this->servo_pose_frame_id_ =
    this->declare_parameter<std::string>("servo_pose_frame_id", "mg400_origin_link");

  const auto linear_speed_mmps =
    this->declare_parameter<double>("linear_speed_mm_s", 20.0);
  const auto angular_speed_degps =
    this->declare_parameter<double>("angular_speed_deg_s", 15.0);
  const auto joint_speed_degps =
    this->declare_parameter<double>("joint_speed_deg_s", 15.0);
  this->linear_speed_mps_ = std::max(0.0, linear_speed_mmps) * kMillimetersToMeters;
  this->angular_speed_radps_ = std::max(0.0, angular_speed_degps) * kDegreesToRadians;
  this->joint_speed_radps_ = std::max(0.0, joint_speed_degps) * kDegreesToRadians;
  this->stick_deadzone_ =
    clampValue(this->declare_parameter<double>("stick_deadzone", 0.05), 0.0, 1.0);
  this->controller_axis_threshold_ =
    clampValue(this->declare_parameter<double>("controller_axis_threshold", 0.5), 0.0, 1.0);

  const bool joy_mapping_loaded_from_file =
    !this->joy_mapping_file_.empty() && this->loadJoyMappingFile(this->joy_mapping_file_);

  this->service_timeout_ms_ =
    std::max<int>(1, static_cast<int>(this->declare_parameter<int>("service_timeout_ms", 5000)));
  this->button_cooldown_ms_ =
    std::max<int>(1, static_cast<int>(this->declare_parameter<int>("button_cooldown_ms", 700)));
  this->joy_timeout_ms_ =
    std::max<int>(1, static_cast<int>(this->declare_parameter<int>("joy_timeout_ms", 250)));
  const auto servo_publish_period_ms =
    std::max<int>(
    1, static_cast<int>(this->declare_parameter<int>("servo_publish_period_ms", 15)));
  this->controller_check_timeout_sec_ =
    std::max(1.0, this->declare_parameter<double>("controller_check_timeout_sec", 60.0));

  if (normalizeControlType(default_control_type) == "servop") {
    this->servo_control_type_ = ServoControlType::SERVO_P;
  } else {
    this->servo_control_type_ = ServoControlType::SERVO_J;
  }

  if (!this->joy_mapping_file_.empty() && !joy_mapping_loaded_from_file) {
    this->controller_check_stage_ = ControllerCheckStage::FAILED;
  } else if (joy_mapping_loaded_from_file) {
    this->controller_check_stage_ = ControllerCheckStage::PASSED;
  } else {
    this->controller_check_stage_ = ControllerCheckStage::WAITING_CROSS;
  }

  this->callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  this->callback_group_executor_.add_callback_group(
    this->callback_group_, this->get_node_base_interface());

  this->reset_robot_clnt_ =
    this->create_client<mg400_msgs::srv::ResetRobot>(
    "reset_robot", rmw_qos_profile_default, this->callback_group_);
  this->enable_robot_clnt_ =
    this->create_client<mg400_msgs::srv::EnableRobot>(
    "enable_robot", rmw_qos_profile_default, this->callback_group_);
  this->disable_robot_clnt_ =
    this->create_client<mg400_msgs::srv::DisableRobot>(
    "disable_robot", rmw_qos_profile_default, this->callback_group_);
  this->servo_mode_clnt_ =
    this->create_client<ServoModeService>(
    "servo_mode", rmw_qos_profile_default, this->callback_group_);

  this->servo_j_pub_ = this->create_publisher<ServoJ>(
    "servo_j", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
  this->servo_p_pub_ = this->create_publisher<ServoP>(
    "servo_p", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
  this->joy_control_active_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "joy_control_active", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

  this->robot_mode_sub_ = this->create_subscription<RobotMode>(
    "robot_mode", rclcpp::SensorDataQoS().keep_last(1),
    [this](const RobotMode::ConstSharedPtr msg) {
      const std::lock_guard<std::mutex> lock(this->state_mutex_);
      this->current_robot_mode_ = msg->robot_mode;
    });

  this->realtime_feedback_sub_ = this->create_subscription<RealtimeFeedback>(
    "realtime_feedback", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&MG400JoyInterfaceNode::onRealtimeFeedback, this, std::placeholders::_1));

  this->servo_mode_enabled_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "servo_mode_enabled",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    std::bind(&MG400JoyInterfaceNode::onServoModeEnabled, this, std::placeholders::_1));

  this->joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&MG400JoyInterfaceNode::onJoy, this, std::placeholders::_1));

  this->servo_publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(servo_publish_period_ms),
    std::bind(&MG400JoyInterfaceNode::onServoPublishTimer, this));
  this->joy_control_active_timer_ = this->create_wall_timer(
    kJoyControlActivePeriod,
    std::bind(&MG400JoyInterfaceNode::publishJoyControlActive, this));
  this->publishJoyControlActive();

  RCLCPP_INFO(
    this->get_logger(),
    "MG400 joy ServoMode controller initialized: mapping_file=%s, control=%s",
    this->joy_mapping_file_.empty() ? "<learn at startup>" : this->joy_mapping_file_.c_str(),
    this->servoControlTypeName(this->servo_control_type_));
}

bool MG400JoyInterfaceNode::loadJoyMappingFile(const std::string & path)
{
  const auto require_node = [](const YAML::Node & parent, const std::string & name) {
      const YAML::Node node = parent[name];
      if (!node) {
        throw std::runtime_error("missing required key: " + name);
      }
      return node;
    };
  const auto require_int = [&require_node](const YAML::Node & parent, const std::string & name) {
      return require_node(parent, name).as<int>();
    };
  const auto require_double =
    [&require_node](const YAML::Node & parent, const std::string & name) {
      return require_node(parent, name).as<double>();
    };
  const auto optional_int =
    [](const YAML::Node & parent, const std::string & name, const int default_value) {
      const YAML::Node node = parent[name];
      return node ? node.as<int>() : default_value;
    };
  const auto optional_double =
    [](const YAML::Node & parent, const std::string & name, const double default_value) {
      const YAML::Node node = parent[name];
      return node ? node.as<double>() : default_value;
    };
  const auto require_axis_mapping =
    [&require_node, &require_int, &require_double](
    const YAML::Node & parent, const std::string & name) {
      const YAML::Node node = require_node(parent, name);
      JoyMapping::AxisMapping mapping;
      mapping.index = require_int(node, "index");
      mapping.positive_value = require_double(node, "positive_value");
      if (std::fabs(mapping.positive_value) < 1.0e-6) {
        throw std::runtime_error(name + ".positive_value must be nonzero");
      }
      return mapping;
    };

  try {
    const YAML::Node config = YAML::LoadFile(path);
    const YAML::Node buttons = require_node(config, "buttons");
    const YAML::Node axes = require_node(config, "axes");
    const YAML::Node dpad_up = require_node(config, "dpad_up");

    JoyMapping mapping;
    mapping.button_cross = require_int(buttons, "cross");
    mapping.button_l1 = require_int(buttons, "l1");
    mapping.button_r1 = require_int(buttons, "r1");
    mapping.button_select = require_int(buttons, "select");
    mapping.button_start = require_int(buttons, "start");
    mapping.button_ps = require_int(buttons, "ps");
    mapping.axis_stick_lx = require_axis_mapping(axes, "stick_lx");
    mapping.axis_stick_ly = require_axis_mapping(axes, "stick_ly");
    mapping.axis_stick_rx = require_axis_mapping(axes, "stick_rx");
    mapping.axis_stick_ry = require_axis_mapping(axes, "stick_ry");
    mapping.button_dpad_up = optional_int(dpad_up, "button", -1);
    mapping.axis_dpad_y = optional_int(dpad_up, "axis", -1);
    mapping.dpad_up_axis_value = optional_double(dpad_up, "axis_value", 0.0);

    if (mapping.button_dpad_up < 0 &&
      (mapping.axis_dpad_y < 0 || std::fabs(mapping.dpad_up_axis_value) < 1.0e-6))
    {
      throw std::runtime_error("dpad_up requires either button or axis with nonzero axis_value");
    }

    std::string reason;
    if (!this->validateJoyMappingNoOverlap(mapping, reason)) {
      throw std::runtime_error(reason);
    }

    this->joy_mapping_ = mapping;
    RCLCPP_INFO(this->get_logger(), "Loaded joy mapping file: %s", path.c_str());
    return true;
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to load joy mapping file '%s': %s",
      path.c_str(), exception.what());
    return false;
  }
}

bool MG400JoyInterfaceNode::validateJoyMappingNoOverlap(
  const JoyMapping & mapping, std::string & reason) const
{
  struct IndexedInput
  {
    int index;
    const char * label;
  };

  const std::array<IndexedInput, 7> buttons = {
    IndexedInput{mapping.button_cross, "Cross/A"},
    IndexedInput{mapping.button_l1, "L1/LB"},
    IndexedInput{mapping.button_r1, "R1/RB"},
    IndexedInput{mapping.button_select, "Select/Back"},
    IndexedInput{mapping.button_start, "Start/Menu"},
    IndexedInput{mapping.button_ps, "PS/Guide"},
    IndexedInput{mapping.button_dpad_up, "D-pad Up"}
  };
  const std::array<IndexedInput, 5> axes = {
    IndexedInput{mapping.axis_stick_lx.index, "Left Stick right"},
    IndexedInput{mapping.axis_stick_ly.index, "Left Stick up"},
    IndexedInput{mapping.axis_stick_rx.index, "Right Stick right"},
    IndexedInput{mapping.axis_stick_ry.index, "Right Stick up"},
    IndexedInput{mapping.axis_dpad_y, "D-pad Up"}
  };

  const auto check_duplicates =
    [&reason](const std::array<IndexedInput, 7> & inputs, const char * input_type) {
      for (size_t lhs = 0; lhs < inputs.size(); ++lhs) {
        if (inputs[lhs].index < 0) {
          continue;
        }
        for (size_t rhs = lhs + 1; rhs < inputs.size(); ++rhs) {
          if (inputs[rhs].index < 0 || inputs[lhs].index != inputs[rhs].index) {
            continue;
          }
          reason = std::string("duplicate ") + input_type + " mapping: " +
                   inputs[lhs].label + " and " + inputs[rhs].label +
                   " both use index " + std::to_string(inputs[lhs].index);
          return false;
        }
      }
      return true;
    };
  const auto check_axis_duplicates =
    [&reason](const std::array<IndexedInput, 5> & inputs) {
      for (size_t lhs = 0; lhs < inputs.size(); ++lhs) {
        if (inputs[lhs].index < 0) {
          continue;
        }
        for (size_t rhs = lhs + 1; rhs < inputs.size(); ++rhs) {
          if (inputs[rhs].index < 0 || inputs[lhs].index != inputs[rhs].index) {
            continue;
          }
          reason = std::string("duplicate axis mapping: ") +
                   inputs[lhs].label + " and " + inputs[rhs].label +
                   " both use index " + std::to_string(inputs[lhs].index);
          return false;
        }
      }
      return true;
    };

  if (!check_duplicates(buttons, "button")) {
    return false;
  }
  if (!check_axis_duplicates(axes)) {
    return false;
  }
  if (std::fabs(mapping.axis_stick_lx.positive_value) < 1.0e-6 ||
    std::fabs(mapping.axis_stick_ly.positive_value) < 1.0e-6 ||
    std::fabs(mapping.axis_stick_rx.positive_value) < 1.0e-6 ||
    std::fabs(mapping.axis_stick_ry.positive_value) < 1.0e-6)
  {
    reason = "stick axis positive_value must be nonzero";
    return false;
  }
  if (mapping.button_dpad_up >= 0 && mapping.axis_dpad_y >= 0) {
    reason = "D-pad Up must use either button or axis, not both";
    return false;
  }
  if (mapping.button_dpad_up < 0 &&
    (mapping.axis_dpad_y < 0 || std::fabs(mapping.dpad_up_axis_value) < 1.0e-6))
  {
    reason = "D-pad Up requires either button or axis with nonzero axis_value";
    return false;
  }

  return true;
}

void MG400JoyInterfaceNode::onJoy(const sensor_msgs::msg::Joy::ConstSharedPtr joy_msg)
{
  this->ensureControllerCheckBaseline(*joy_msg);

  if (!this->handleControllerCheck(joy_msg)) {
    this->zeroServoAxes();
    return;
  }

  {
    const std::lock_guard<std::mutex> lock(this->state_mutex_);
    this->last_joy_time_ = this->now();
  }

  if (this->handleButtonActions(*joy_msg)) {
    return;
  }

  this->updateServoAxesFromJoy(*joy_msg);
}

void MG400JoyInterfaceNode::onRealtimeFeedback(const RealtimeFeedback::ConstSharedPtr msg)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = msg->tool_vector_actual[0] * kMillimetersToMeters;
  pose.position.y = msg->tool_vector_actual[1] * kMillimetersToMeters;
  pose.position.z = msg->tool_vector_actual[2] * kMillimetersToMeters;

  const double yaw = msg->tool_vector_actual[3] * kDegreesToRadians;
  this->setPoseYaw(pose, yaw);

  const std::array<double, 4> joint_state = {
    msg->q_actual[0] * kDegreesToRadians,
    msg->q_actual[1] * kDegreesToRadians,
    msg->q_actual[2] * kDegreesToRadians,
    msg->q_actual[3] * kDegreesToRadians
  };

  const std::lock_guard<std::mutex> lock(this->state_mutex_);
  this->current_pose_ = pose;
  this->current_pose_yaw_ = yaw;
  this->current_joint_state_ = joint_state;
  this->has_actual_state_ = true;
}

void MG400JoyInterfaceNode::onServoModeEnabled(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  bool should_initialize = false;
  {
    const std::lock_guard<std::mutex> lock(this->state_mutex_);
    should_initialize = msg->data && !this->servo_mode_enabled_;
    this->servo_mode_enabled_ = msg->data;
    this->servo_pose_axes_ = {0.0, 0.0, 0.0, 0.0};
    this->servo_joint_axes_ = {0.0, 0.0, 0.0, 0.0};
    if (!msg->data) {
      this->servo_pose_target_initialized_ = false;
      this->servo_joint_target_initialized_ = false;
      this->last_servo_target_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
  }

  if (should_initialize) {
    if (this->initializeSelectedServoTarget()) {
      RCLCPP_INFO(
        this->get_logger(), "ServoMode active. Publishing %s targets.",
        this->servoControlTypeName(this->servo_control_type_));
    } else {
      RCLCPP_WARN(this->get_logger(), "ServoMode active, waiting for realtime feedback.");
    }
  } else if (!msg->data) {
    RCLCPP_INFO(this->get_logger(), "ServoMode inactive.");
  }
}

void MG400JoyInterfaceNode::onServoPublishTimer()
{
  if (!this->handleControllerCheck(sensor_msgs::msg::Joy::ConstSharedPtr())) {
    return;
  }
  this->publishServoTarget();
}

void MG400JoyInterfaceNode::publishJoyControlActive()
{
  if (!this->joy_control_active_pub_) {
    return;
  }

  std_msgs::msg::Bool msg;
  msg.data = true;
  this->joy_control_active_pub_->publish(msg);
}

bool MG400JoyInterfaceNode::handleControllerCheck(
  const sensor_msgs::msg::Joy::ConstSharedPtr joy_msg)
{
  ControllerCheckStage stage;
  const auto now = this->now();
  const bool evaluate_input = static_cast<bool>(joy_msg);
  {
    const std::lock_guard<std::mutex> lock(this->state_mutex_);
    stage = this->controller_check_stage_;
    if (stage == ControllerCheckStage::DISABLED || stage == ControllerCheckStage::PASSED) {
      return true;
    }
    if (stage == ControllerCheckStage::FAILED) {
      return false;
    }
    if (evaluate_input && timeIsUnset(this->controller_check_start_time_)) {
      this->controller_check_start_time_ = now;
    }
    if (!timeIsUnset(this->controller_check_start_time_) &&
      (now - this->controller_check_start_time_).seconds() >=
      this->controller_check_timeout_sec_)
    {
      stage = ControllerCheckStage::FAILED;
    }
  }

  if (stage == ControllerCheckStage::FAILED) {
    this->failControllerCheck("controller check timed out");
    return false;
  }

  if (evaluate_input) {
    this->ensureControllerCheckBaseline(*joy_msg);
  }

  const auto advance_stage = [this](const ControllerCheckStage next_stage) {
      const std::lock_guard<std::mutex> lock(this->state_mutex_);
      this->controller_check_stage_ = next_stage;
      this->last_controller_check_prompt_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    };

  switch (stage) {
    case ControllerCheckStage::WAITING_CROSS:
      this->promptControllerCheck("Press Cross/A on the controller.");
      if (!evaluate_input) {
        return false;
      }
      if (this->learnButtonInput(*joy_msg, this->joy_mapping_.button_cross, "Cross/A")) {
        advance_stage(ControllerCheckStage::WAITING_RELEASE_AFTER_CROSS);
      }
      return false;

    case ControllerCheckStage::WAITING_RELEASE_AFTER_CROSS:
      this->promptControllerCheck("Release all controller inputs.");
      if (!evaluate_input) {
        return false;
      }
      if (!this->controllerCheckInputActive(*joy_msg)) {
        advance_stage(ControllerCheckStage::WAITING_L1);
      }
      return false;

    case ControllerCheckStage::WAITING_L1:
      this->promptControllerCheck("Press L1/LB on the controller.");
      if (!evaluate_input) {
        return false;
      }
      if (this->learnButtonInput(*joy_msg, this->joy_mapping_.button_l1, "L1/LB")) {
        advance_stage(ControllerCheckStage::WAITING_RELEASE_AFTER_L1);
      }
      return false;

    case ControllerCheckStage::WAITING_RELEASE_AFTER_L1:
      this->promptControllerCheck("Release all controller inputs.");
      if (!evaluate_input) {
        return false;
      }
      if (!this->controllerCheckInputActive(*joy_msg)) {
        advance_stage(ControllerCheckStage::WAITING_R1);
      }
      return false;

    case ControllerCheckStage::WAITING_R1:
      this->promptControllerCheck("Press R1/RB on the controller.");
      if (!evaluate_input) {
        return false;
      }
      if (this->learnButtonInput(*joy_msg, this->joy_mapping_.button_r1, "R1/RB")) {
        advance_stage(ControllerCheckStage::WAITING_RELEASE_AFTER_R1);
      }
      return false;

    case ControllerCheckStage::WAITING_RELEASE_AFTER_R1:
      this->promptControllerCheck("Release all controller inputs.");
      if (!evaluate_input) {
        return false;
      }
      if (!this->controllerCheckInputActive(*joy_msg)) {
        advance_stage(ControllerCheckStage::WAITING_SELECT);
      }
      return false;

    case ControllerCheckStage::WAITING_SELECT:
      this->promptControllerCheck("Press Select/Back on the controller.");
      if (!evaluate_input) {
        return false;
      }
      if (this->learnButtonInput(*joy_msg, this->joy_mapping_.button_select, "Select/Back")) {
        advance_stage(ControllerCheckStage::WAITING_RELEASE_AFTER_SELECT);
      }
      return false;

    case ControllerCheckStage::WAITING_RELEASE_AFTER_SELECT:
      this->promptControllerCheck("Release all controller inputs.");
      if (!evaluate_input) {
        return false;
      }
      if (!this->controllerCheckInputActive(*joy_msg)) {
        advance_stage(ControllerCheckStage::WAITING_START);
      }
      return false;

    case ControllerCheckStage::WAITING_START:
      this->promptControllerCheck("Press Start/Menu on the controller.");
      if (!evaluate_input) {
        return false;
      }
      if (this->learnButtonInput(*joy_msg, this->joy_mapping_.button_start, "Start/Menu")) {
        advance_stage(ControllerCheckStage::WAITING_RELEASE_AFTER_START);
      }
      return false;

    case ControllerCheckStage::WAITING_RELEASE_AFTER_START:
      this->promptControllerCheck("Release all controller inputs.");
      if (!evaluate_input) {
        return false;
      }
      if (!this->controllerCheckInputActive(*joy_msg)) {
        advance_stage(ControllerCheckStage::WAITING_PS);
      }
      return false;

    case ControllerCheckStage::WAITING_PS:
      this->promptControllerCheck("Press PS/Guide on the controller.");
      if (!evaluate_input) {
        return false;
      }
      if (this->learnButtonInput(*joy_msg, this->joy_mapping_.button_ps, "PS/Guide")) {
        advance_stage(ControllerCheckStage::WAITING_RELEASE_AFTER_PS);
      }
      return false;

    case ControllerCheckStage::WAITING_RELEASE_AFTER_PS:
      this->promptControllerCheck("Release all controller inputs.");
      if (!evaluate_input) {
        return false;
      }
      if (!this->controllerCheckInputActive(*joy_msg)) {
        advance_stage(ControllerCheckStage::WAITING_LEFT_STICK_X);
      }
      return false;

    case ControllerCheckStage::WAITING_LEFT_STICK_X:
      this->promptControllerCheck("Move Left Stick right.");
      if (!evaluate_input) {
        return false;
      }
      if (this->learnAxisInput(*joy_msg, this->joy_mapping_.axis_stick_lx, "Left Stick right")) {
        advance_stage(ControllerCheckStage::WAITING_RELEASE_AFTER_LEFT_STICK_X);
      }
      return false;

    case ControllerCheckStage::WAITING_RELEASE_AFTER_LEFT_STICK_X:
      this->promptControllerCheck("Release all controller inputs.");
      if (!evaluate_input) {
        return false;
      }
      if (!this->controllerCheckInputActive(*joy_msg)) {
        advance_stage(ControllerCheckStage::WAITING_LEFT_STICK_Y);
      }
      return false;

    case ControllerCheckStage::WAITING_LEFT_STICK_Y:
      this->promptControllerCheck("Move Left Stick up.");
      if (!evaluate_input) {
        return false;
      }
      if (this->learnAxisInput(*joy_msg, this->joy_mapping_.axis_stick_ly, "Left Stick up")) {
        advance_stage(ControllerCheckStage::WAITING_RELEASE_AFTER_LEFT_STICK_Y);
      }
      return false;

    case ControllerCheckStage::WAITING_RELEASE_AFTER_LEFT_STICK_Y:
      this->promptControllerCheck("Release all controller inputs.");
      if (!evaluate_input) {
        return false;
      }
      if (!this->controllerCheckInputActive(*joy_msg)) {
        advance_stage(ControllerCheckStage::WAITING_RIGHT_STICK_X);
      }
      return false;

    case ControllerCheckStage::WAITING_RIGHT_STICK_X:
      this->promptControllerCheck("Move Right Stick right.");
      if (!evaluate_input) {
        return false;
      }
      if (this->learnAxisInput(*joy_msg, this->joy_mapping_.axis_stick_rx, "Right Stick right")) {
        advance_stage(ControllerCheckStage::WAITING_RELEASE_AFTER_RIGHT_STICK_X);
      }
      return false;

    case ControllerCheckStage::WAITING_RELEASE_AFTER_RIGHT_STICK_X:
      this->promptControllerCheck("Release all controller inputs.");
      if (!evaluate_input) {
        return false;
      }
      if (!this->controllerCheckInputActive(*joy_msg)) {
        advance_stage(ControllerCheckStage::WAITING_RIGHT_STICK_Y);
      }
      return false;

    case ControllerCheckStage::WAITING_RIGHT_STICK_Y:
      this->promptControllerCheck("Move Right Stick up.");
      if (!evaluate_input) {
        return false;
      }
      if (this->learnAxisInput(*joy_msg, this->joy_mapping_.axis_stick_ry, "Right Stick up")) {
        advance_stage(ControllerCheckStage::WAITING_RELEASE_AFTER_RIGHT_STICK_Y);
      }
      return false;

    case ControllerCheckStage::WAITING_RELEASE_AFTER_RIGHT_STICK_Y:
      this->promptControllerCheck("Release all controller inputs.");
      if (!evaluate_input) {
        return false;
      }
      if (!this->controllerCheckInputActive(*joy_msg)) {
        advance_stage(ControllerCheckStage::WAITING_DPAD_UP);
      }
      return false;

    case ControllerCheckStage::WAITING_DPAD_UP:
      this->promptControllerCheck("Press D-pad Up on the controller.");
      if (!evaluate_input) {
        return false;
      }
      if (this->learnDpadUpInput(*joy_msg)) {
        std::string reason;
        if (!this->validateJoyMappingNoOverlap(this->joy_mapping_, reason)) {
          this->failControllerCheck(reason);
          return false;
        }
        {
          const std::lock_guard<std::mutex> lock(this->state_mutex_);
          this->controller_check_stage_ = ControllerCheckStage::PASSED;
        }
        RCLCPP_INFO(this->get_logger(), "Controller mapping calibration passed.");
        return true;
      }
      return false;

    default:
      return false;
  }
}

void MG400JoyInterfaceNode::ensureControllerCheckBaseline(
  const sensor_msgs::msg::Joy & joy_msg)
{
  bool baseline_ready = false;
  {
    const std::lock_guard<std::mutex> lock(this->state_mutex_);
    baseline_ready = this->controller_check_baseline_ready_;
    if (!this->controller_check_baseline_ready_) {
      this->controller_check_baseline_ = joy_msg;
      this->controller_check_baseline_ready_ = true;
    }
  }

  if (!baseline_ready) {
    RCLCPP_INFO(
      this->get_logger(),
      "Controller check: captured neutral raw Joy baseline (%zu buttons, %zu axes).",
      joy_msg.buttons.size(), joy_msg.axes.size());
  }
}

bool MG400JoyInterfaceNode::controllerCheckInputActive(
  const sensor_msgs::msg::Joy & joy_msg) const
{
  return this->anyButtonPressed(joy_msg) ||
         this->anyAxisChangedFromBaseline(joy_msg);
}

bool MG400JoyInterfaceNode::learnButtonInput(
  const sensor_msgs::msg::Joy & joy_msg, int & target_index, const std::string & label)
{
  const int button_index = this->singleActiveButtonIndex(joy_msg);
  const int axis_index = this->singleActiveAxisIndex(joy_msg);
  if (button_index >= 0 && axis_index == -1) {
    target_index = button_index;
    RCLCPP_INFO(
      this->get_logger(), "Controller mapping: %s -> buttons[%d]",
      label.c_str(), button_index);
    return true;
  }
  if (this->controllerCheckInputActive(joy_msg)) {
    this->failControllerCheck(
      "expected only " + label + ", but multiple or wrong inputs were detected.");
  }
  return false;
}

bool MG400JoyInterfaceNode::learnAxisInput(
  const sensor_msgs::msg::Joy & joy_msg, JoyMapping::AxisMapping & target_axis,
  const std::string & label)
{
  const int button_index = this->singleActiveButtonIndex(joy_msg);
  const int axis_index = this->singleActiveAxisIndex(joy_msg);
  if (axis_index >= 0 && button_index == -1) {
    const double axis_value = this->normalizedAxisValue(joy_msg, axis_index);
    target_axis.index = axis_index;
    target_axis.positive_value = axis_value;
    RCLCPP_INFO(
      this->get_logger(), "Controller mapping: %s -> axes[%d] positive delta %.3f",
      label.c_str(), axis_index, axis_value);
    return true;
  }
  if (this->controllerCheckInputActive(joy_msg)) {
    this->failControllerCheck(
      "expected only " + label + ", but multiple or wrong inputs were detected.");
  }
  return false;
}

bool MG400JoyInterfaceNode::learnDpadUpInput(const sensor_msgs::msg::Joy & joy_msg)
{
  const int button_index = this->singleActiveButtonIndex(joy_msg);
  const int axis_index = this->singleActiveAxisIndex(joy_msg);
  if (button_index >= 0 && axis_index == -1) {
    this->joy_mapping_.button_dpad_up = button_index;
    this->joy_mapping_.axis_dpad_y = -1;
    this->joy_mapping_.dpad_up_axis_value = 0.0;
    RCLCPP_INFO(
      this->get_logger(), "Controller mapping: D-pad Up -> buttons[%d]",
      button_index);
    return true;
  }
  if (axis_index >= 0 && button_index == -1) {
    const double axis_value = this->normalizedAxisValue(joy_msg, axis_index);
    this->joy_mapping_.button_dpad_up = -1;
    this->joy_mapping_.axis_dpad_y = axis_index;
    this->joy_mapping_.dpad_up_axis_value = axis_value;
    RCLCPP_INFO(
      this->get_logger(), "Controller mapping: D-pad Up -> axes[%d] delta %.3f",
      axis_index, axis_value);
    return true;
  }
  if (this->controllerCheckInputActive(joy_msg)) {
    this->failControllerCheck(
      "expected only D-pad Up, but multiple or wrong inputs were detected.");
  }
  return false;
}

int MG400JoyInterfaceNode::singleActiveButtonIndex(const sensor_msgs::msg::Joy & joy_msg) const
{
  int active_index = -1;
  for (size_t i = 0; i < joy_msg.buttons.size(); ++i) {
    if (joy_msg.buttons[i] == 0) {
      continue;
    }
    if (active_index >= 0) {
      return -2;
    }
    active_index = static_cast<int>(i);
  }
  return active_index;
}

int MG400JoyInterfaceNode::singleActiveAxisIndex(const sensor_msgs::msg::Joy & joy_msg) const
{
  int active_index = -1;
  for (size_t i = 0; i < joy_msg.axes.size(); ++i) {
    if (this->axisDeltaFromBaseline(joy_msg, static_cast<int>(i)) <
      this->controller_axis_threshold_)
    {
      continue;
    }
    if (active_index >= 0) {
      return -2;
    }
    active_index = static_cast<int>(i);
  }
  return active_index;
}

bool MG400JoyInterfaceNode::anyAxisChangedFromBaseline(
  const sensor_msgs::msg::Joy & joy_msg) const
{
  for (size_t i = 0; i < joy_msg.axes.size(); ++i) {
    if (this->axisDeltaFromBaseline(joy_msg, static_cast<int>(i)) >=
      this->controller_axis_threshold_)
    {
      return true;
    }
  }
  return false;
}

bool MG400JoyInterfaceNode::buttonPressed(
  const sensor_msgs::msg::Joy & joy_msg, const int index) const
{
  if (index < 0 || static_cast<size_t>(index) >= joy_msg.buttons.size()) {
    return false;
  }
  return joy_msg.buttons[static_cast<size_t>(index)] != 0;
}

bool MG400JoyInterfaceNode::anyButtonPressed(const sensor_msgs::msg::Joy & joy_msg) const
{
  return std::any_of(
    joy_msg.buttons.begin(), joy_msg.buttons.end(),
    [](const int32_t value) {return value != 0;});
}

double MG400JoyInterfaceNode::axisValue(
  const sensor_msgs::msg::Joy & joy_msg, const int index) const
{
  if (index < 0 || static_cast<size_t>(index) >= joy_msg.axes.size()) {
    return 0.0;
  }
  return static_cast<double>(joy_msg.axes[static_cast<size_t>(index)]);
}

double MG400JoyInterfaceNode::baselineAxisValue(const int index) const
{
  const std::lock_guard<std::mutex> lock(this->state_mutex_);
  if (!this->controller_check_baseline_ready_ || index < 0 ||
    static_cast<size_t>(index) >= this->controller_check_baseline_.axes.size())
  {
    return 0.0;
  }
  return static_cast<double>(this->controller_check_baseline_.axes[static_cast<size_t>(index)]);
}

double MG400JoyInterfaceNode::normalizedAxisValue(
  const sensor_msgs::msg::Joy & joy_msg, const int index) const
{
  return this->axisValue(joy_msg, index) - this->baselineAxisValue(index);
}

double MG400JoyInterfaceNode::orientedAxisValue(
  const sensor_msgs::msg::Joy & joy_msg, const JoyMapping::AxisMapping & mapping) const
{
  const double positive_sign = mapping.positive_value < 0.0 ? -1.0 : 1.0;
  return this->normalizedAxisValue(joy_msg, mapping.index) * positive_sign;
}

double MG400JoyInterfaceNode::axisDeltaFromBaseline(
  const sensor_msgs::msg::Joy & joy_msg, const int index) const
{
  return std::fabs(this->normalizedAxisValue(joy_msg, index));
}

bool MG400JoyInterfaceNode::axisPressed(
  const sensor_msgs::msg::Joy & joy_msg, const int index, const double expected_value) const
{
  if (std::fabs(expected_value) < 1.0e-6) {
    return false;
  }
  return this->normalizedAxisValue(joy_msg, index) * expected_value >=
         this->controller_axis_threshold_;
}

bool MG400JoyInterfaceNode::dpadUpPressed(const sensor_msgs::msg::Joy & joy_msg) const
{
  return this->buttonPressed(joy_msg, this->joy_mapping_.button_dpad_up) ||
         this->axisPressed(
    joy_msg, this->joy_mapping_.axis_dpad_y, this->joy_mapping_.dpad_up_axis_value);
}

void MG400JoyInterfaceNode::promptControllerCheck(const std::string & message)
{
  const auto now = this->now();
  {
    const std::lock_guard<std::mutex> lock(this->state_mutex_);
    if (!elapsed(now, this->last_controller_check_prompt_time_, kControllerPromptPeriodSec)) {
      return;
    }
    this->last_controller_check_prompt_time_ = now;
  }
  RCLCPP_INFO(this->get_logger(), "Controller check: %s", message.c_str());
}

void MG400JoyInterfaceNode::failControllerCheck(const std::string & reason)
{
  {
    const std::lock_guard<std::mutex> lock(this->state_mutex_);
    this->controller_check_stage_ = ControllerCheckStage::FAILED;
  }
  RCLCPP_ERROR(
    this->get_logger(),
    "Controller mapping failed: %s. Joy control is locked.",
    reason.c_str());
}

bool MG400JoyInterfaceNode::handleButtonActions(const sensor_msgs::msg::Joy & joy_msg)
{
  if (!this->buttonActionReady()) {
    return false;
  }

  if (this->buttonPressed(joy_msg, this->joy_mapping_.button_ps)) {
    bool servo_mode_enabled = false;
    {
      const std::lock_guard<std::mutex> lock(this->state_mutex_);
      servo_mode_enabled = this->servo_mode_enabled_;
    }
    this->zeroServoAxes();
    if (servo_mode_enabled) {
      this->requestServoMode(false);
    }
    this->callResetRobot();
    this->markButtonAction();
    return true;
  }

  if (this->buttonPressed(joy_msg, this->joy_mapping_.button_start)) {
    RobotMode::_robot_mode_type robot_mode;
    bool servo_mode_enabled = false;
    {
      const std::lock_guard<std::mutex> lock(this->state_mutex_);
      robot_mode = this->current_robot_mode_;
      servo_mode_enabled = this->servo_mode_enabled_;
    }

    this->zeroServoAxes();
    if (servo_mode_enabled) {
      this->requestServoMode(false);
    } else if (robot_mode == RobotMode::DISABLED) {
      if (this->callEnableRobot()) {
        RCLCPP_INFO(
          this->get_logger(), "Robot enable requested. Press Start again to enter ServoMode.");
      }
    } else if (robot_mode == RobotMode::ENABLE) {
      this->requestServoMode(true);
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Cannot enter ServoMode from robot_mode=%lu.",
        static_cast<unsigned long>(robot_mode));
    }
    this->markButtonAction();
    return true;
  }

  if (this->buttonPressed(joy_msg, this->joy_mapping_.button_select)) {
    RobotMode::_robot_mode_type robot_mode;
    bool servo_mode_enabled = false;
    {
      const std::lock_guard<std::mutex> lock(this->state_mutex_);
      robot_mode = this->current_robot_mode_;
      servo_mode_enabled = this->servo_mode_enabled_;
    }

    this->zeroServoAxes();
    if (servo_mode_enabled) {
      this->requestServoMode(false);
    } else if (robot_mode == RobotMode::ENABLE) {
      this->callDisableRobot();
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Cannot disable robot from robot_mode=%lu.",
        static_cast<unsigned long>(robot_mode));
    }
    this->markButtonAction();
    return true;
  }

  if (this->buttonPressed(joy_msg, this->joy_mapping_.button_r1)) {
    bool servo_mode_enabled = false;
    ServoControlType control_type;
    {
      const std::lock_guard<std::mutex> lock(this->state_mutex_);
      servo_mode_enabled = this->servo_mode_enabled_;
      control_type = this->servo_control_type_;
    }

    if (servo_mode_enabled) {
      RCLCPP_WARN(this->get_logger(), "Exit ServoMode before switching ServoJ/ServoP.");
    } else if (control_type == ServoControlType::SERVO_J) {
      this->setServoControlType(ServoControlType::SERVO_P);
    } else {
      this->setServoControlType(ServoControlType::SERVO_J);
    }
    this->markButtonAction();
    return true;
  }

  return false;
}

bool MG400JoyInterfaceNode::buttonActionReady() const
{
  const auto now = this->now();
  const std::lock_guard<std::mutex> lock(this->state_mutex_);
  if (timeIsUnset(this->last_button_action_time_)) {
    return true;
  }
  return (now - this->last_button_action_time_).nanoseconds() >=
         static_cast<int64_t>(this->button_cooldown_ms_) * 1000000LL;
}

void MG400JoyInterfaceNode::markButtonAction()
{
  const std::lock_guard<std::mutex> lock(this->state_mutex_);
  this->last_button_action_time_ = this->now();
}

void MG400JoyInterfaceNode::updateServoAxesFromJoy(const sensor_msgs::msg::Joy & joy_msg)
{
  const double lx =
    this->applyDeadzone(this->orientedAxisValue(joy_msg, this->joy_mapping_.axis_stick_lx));
  const double ly =
    this->applyDeadzone(this->orientedAxisValue(joy_msg, this->joy_mapping_.axis_stick_ly));
  const double rx =
    this->applyDeadzone(this->orientedAxisValue(joy_msg, this->joy_mapping_.axis_stick_rx));
  const double ry =
    this->applyDeadzone(this->orientedAxisValue(joy_msg, this->joy_mapping_.axis_stick_ry));

  const std::lock_guard<std::mutex> lock(this->state_mutex_);
  this->servo_joint_axes_ = {
    -lx,
    -ly,
    ry,
    -rx
  };
  this->servo_pose_axes_ = {
    ly,
    -lx,
    ry,
    -rx
  };
}

void MG400JoyInterfaceNode::zeroServoAxes()
{
  const std::lock_guard<std::mutex> lock(this->state_mutex_);
  this->servo_pose_axes_ = {0.0, 0.0, 0.0, 0.0};
  this->servo_joint_axes_ = {0.0, 0.0, 0.0, 0.0};
}

double MG400JoyInterfaceNode::applyDeadzone(const double value) const
{
  if (std::fabs(value) < this->stick_deadzone_) {
    return 0.0;
  }
  return clampValue(value, -1.0, 1.0);
}

bool MG400JoyInterfaceNode::requestServoMode(const bool enable)
{
  const auto service_timeout = std::chrono::milliseconds(this->service_timeout_ms_);

  if (enable) {
    bool has_actual_state = false;
    RobotMode::_robot_mode_type robot_mode = RobotMode::INVALID;
    {
      const std::lock_guard<std::mutex> lock(this->state_mutex_);
      has_actual_state = this->has_actual_state_;
      robot_mode = this->current_robot_mode_;
    }

    if (!has_actual_state) {
      RCLCPP_WARN(this->get_logger(), "Cannot enter ServoMode: realtime feedback is unavailable.");
      return false;
    }
    if (robot_mode != RobotMode::ENABLE) {
      RCLCPP_WARN(
        this->get_logger(),
        "Cannot enter ServoMode: robot_mode must be ENABLE, current=%lu.",
        static_cast<unsigned long>(robot_mode));
      return false;
    }
  }

  if (!this->servo_mode_clnt_->wait_for_service(service_timeout)) {
    RCLCPP_ERROR(
      this->get_logger(), "\"%s\" is not ready",
      this->servo_mode_clnt_->get_service_name());
    return false;
  }

  auto req = std::make_shared<ServoModeService::Request>();
  req->enable = enable;
  auto future_result = this->servo_mode_clnt_->async_send_request(req);

  if (this->callback_group_executor_.spin_until_future_complete(
      future_result, service_timeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    this->servo_mode_clnt_->remove_pending_request(future_result);
    RCLCPP_ERROR(
      this->get_logger(),
      "\"%s\" service client: response timeout",
      this->servo_mode_clnt_->get_service_name());
    return false;
  }

  const auto wrapped_result = future_result.get();
  if (wrapped_result->error_id != 0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "\"%s\" service client: failed, error_id=%d, message=%s",
      this->servo_mode_clnt_->get_service_name(),
      wrapped_result->error_id,
      wrapped_result->message.c_str());
    return false;
  }

  {
    const std::lock_guard<std::mutex> lock(this->state_mutex_);
    this->servo_mode_enabled_ = enable;
    this->servo_pose_axes_ = {0.0, 0.0, 0.0, 0.0};
    this->servo_joint_axes_ = {0.0, 0.0, 0.0, 0.0};
    if (!enable) {
      this->servo_pose_target_initialized_ = false;
      this->servo_joint_target_initialized_ = false;
      this->last_servo_target_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
  }

  if (enable) {
    if (!this->initializeSelectedServoTarget()) {
      RCLCPP_WARN(this->get_logger(), "ServoMode entered, but target initialization failed.");
      return true;
    }
    RCLCPP_INFO(
      this->get_logger(), "ServoMode entered. Publishing %s targets.",
      this->servoControlTypeName(this->servo_control_type_));
    this->publishServoTarget();
  } else {
    RCLCPP_INFO(this->get_logger(), "ServoMode exited.");
  }
  return true;
}

void MG400JoyInterfaceNode::callResetRobot()
{
  const auto service_timeout = std::chrono::milliseconds(this->service_timeout_ms_);

  if (!this->reset_robot_clnt_->wait_for_service(service_timeout)) {
    RCLCPP_ERROR(
      this->get_logger(), "\"%s\" is not ready",
      this->reset_robot_clnt_->get_service_name());
    return;
  }

  auto req = std::make_shared<mg400_msgs::srv::ResetRobot::Request>();
  auto future_result = this->reset_robot_clnt_->async_send_request(req);

  if (this->callback_group_executor_.spin_until_future_complete(
      future_result, service_timeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    this->reset_robot_clnt_->remove_pending_request(future_result);
    RCLCPP_ERROR(
      this->get_logger(),
      "\"%s\" service client: response timeout",
      this->reset_robot_clnt_->get_service_name());
    return;
  }

  const auto wrapped_result = future_result.get();
  if (!wrapped_result->result) {
    RCLCPP_ERROR(
      this->get_logger(),
      "\"%s\" service client: failed, error_id=%d",
      this->reset_robot_clnt_->get_service_name(), wrapped_result->error_id);
  }
}

bool MG400JoyInterfaceNode::callEnableRobot()
{
  const auto service_timeout = std::chrono::milliseconds(this->service_timeout_ms_);

  if (!this->enable_robot_clnt_->wait_for_service(service_timeout)) {
    RCLCPP_ERROR(
      this->get_logger(), "\"%s\" is not ready",
      this->enable_robot_clnt_->get_service_name());
    return false;
  }

  auto req = std::make_shared<mg400_msgs::srv::EnableRobot::Request>();
  req->num_of_params = mg400_msgs::srv::EnableRobot::Request::NO_PARAM;
  auto future_result = this->enable_robot_clnt_->async_send_request(req);

  if (this->callback_group_executor_.spin_until_future_complete(
      future_result, service_timeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    this->enable_robot_clnt_->remove_pending_request(future_result);
    RCLCPP_ERROR(
      this->get_logger(),
      "\"%s\" service client: response timeout",
      this->enable_robot_clnt_->get_service_name());
    return false;
  }

  const auto wrapped_result = future_result.get();
  if (!wrapped_result->result) {
    RCLCPP_ERROR(
      this->get_logger(),
      "\"%s\" service client: failed, error_id=%d",
      this->enable_robot_clnt_->get_service_name(), wrapped_result->error_id);
    return false;
  }
  return true;
}

bool MG400JoyInterfaceNode::callDisableRobot()
{
  const auto service_timeout = std::chrono::milliseconds(this->service_timeout_ms_);

  if (!this->disable_robot_clnt_->wait_for_service(service_timeout)) {
    RCLCPP_ERROR(
      this->get_logger(), "\"%s\" is not ready",
      this->disable_robot_clnt_->get_service_name());
    return false;
  }

  auto req = std::make_shared<mg400_msgs::srv::DisableRobot::Request>();
  auto future_result = this->disable_robot_clnt_->async_send_request(req);

  if (this->callback_group_executor_.spin_until_future_complete(
      future_result, service_timeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    this->disable_robot_clnt_->remove_pending_request(future_result);
    RCLCPP_ERROR(
      this->get_logger(),
      "\"%s\" service client: response timeout",
      this->disable_robot_clnt_->get_service_name());
    return false;
  }

  const auto wrapped_result = future_result.get();
  if (!wrapped_result->result) {
    RCLCPP_ERROR(
      this->get_logger(),
      "\"%s\" service client: failed, error_id=%d",
      this->disable_robot_clnt_->get_service_name(), wrapped_result->error_id);
    return false;
  }
  return true;
}

bool MG400JoyInterfaceNode::initializeServoPoseTarget()
{
  const std::lock_guard<std::mutex> lock(this->state_mutex_);
  if (!this->has_actual_state_) {
    return false;
  }
  this->servo_target_pose_ = this->current_pose_;
  this->servo_target_yaw_ = this->current_pose_yaw_;
  this->servo_pose_target_initialized_ = true;
  this->last_servo_target_update_time_ = this->now();
  return true;
}

bool MG400JoyInterfaceNode::initializeServoJointTarget()
{
  const std::lock_guard<std::mutex> lock(this->state_mutex_);
  if (!this->has_actual_state_) {
    return false;
  }
  this->servo_joint_target_ = this->current_joint_state_;
  this->servo_joint_target_initialized_ = true;
  this->last_servo_target_update_time_ = this->now();
  return true;
}

bool MG400JoyInterfaceNode::initializeSelectedServoTarget()
{
  ServoControlType control_type;
  {
    const std::lock_guard<std::mutex> lock(this->state_mutex_);
    control_type = this->servo_control_type_;
  }

  if (control_type == ServoControlType::SERVO_P) {
    return this->initializeServoPoseTarget();
  }
  return this->initializeServoJointTarget();
}

void MG400JoyInterfaceNode::resetServoTargets()
{
  const std::lock_guard<std::mutex> lock(this->state_mutex_);
  this->servo_pose_target_initialized_ = false;
  this->servo_joint_target_initialized_ = false;
  this->last_servo_target_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void MG400JoyInterfaceNode::setServoControlType(const ServoControlType servo_control_type)
{
  bool changed = false;
  {
    const std::lock_guard<std::mutex> lock(this->state_mutex_);
    if (this->servo_mode_enabled_) {
      return;
    }
    changed = this->servo_control_type_ != servo_control_type;
    this->servo_control_type_ = servo_control_type;
    this->servo_pose_axes_ = {0.0, 0.0, 0.0, 0.0};
    this->servo_joint_axes_ = {0.0, 0.0, 0.0, 0.0};
    this->servo_pose_target_initialized_ = false;
    this->servo_joint_target_initialized_ = false;
  }
  if (changed) {
    RCLCPP_INFO(
      this->get_logger(), "Servo control: %s",
      this->servoControlTypeName(servo_control_type));
  }
}

void MG400JoyInterfaceNode::publishServoTarget()
{
  ServoControlType control_type;
  const auto now = this->now();
  {
    const std::lock_guard<std::mutex> lock(this->state_mutex_);
    if (this->controller_check_stage_ != ControllerCheckStage::DISABLED &&
      this->controller_check_stage_ != ControllerCheckStage::PASSED)
    {
      return;
    }
    if (!this->servo_mode_enabled_) {
      return;
    }
    if (!this->joyInputIsFresh(now)) {
      this->servo_pose_axes_ = {0.0, 0.0, 0.0, 0.0};
      this->servo_joint_axes_ = {0.0, 0.0, 0.0, 0.0};
      return;
    }
    control_type = this->servo_control_type_;
  }

  if (control_type == ServoControlType::SERVO_P) {
    this->publishServoPoseTarget();
  } else {
    this->publishServoJointTarget();
  }
}

void MG400JoyInterfaceNode::publishServoPoseTarget()
{
  if (!this->servo_p_pub_) {
    return;
  }

  bool needs_initialize = false;
  {
    const std::lock_guard<std::mutex> lock(this->state_mutex_);
    if (!this->servo_mode_enabled_) {
      return;
    }
    needs_initialize = !this->servo_pose_target_initialized_;
  }
  if (needs_initialize && !this->initializeServoPoseTarget()) {
    return;
  }

  const auto now = this->now();
  ServoP msg;
  {
    const std::lock_guard<std::mutex> lock(this->state_mutex_);
    if (!this->servo_mode_enabled_ || !this->servo_pose_target_initialized_) {
      return;
    }

    const double dt = this->consumeTargetElapsedSeconds(now);
    this->servo_target_pose_.position.x += this->servo_pose_axes_[0] * this->linear_speed_mps_ * dt;
    this->servo_target_pose_.position.y += this->servo_pose_axes_[1] * this->linear_speed_mps_ * dt;
    this->servo_target_pose_.position.z += this->servo_pose_axes_[2] * this->linear_speed_mps_ * dt;
    this->servo_target_yaw_ += this->servo_pose_axes_[3] * this->angular_speed_radps_ * dt;
    this->setPoseYaw(this->servo_target_pose_, this->servo_target_yaw_);

    msg.pose.header.frame_id = this->servo_pose_frame_id_;
    msg.pose.header.stamp = now;
    msg.pose.pose = this->servo_target_pose_;
    msg.t = kUnusedServoOption;
    msg.aheadtime = kUnusedServoOption;
    msg.gain = kUnusedServoOption;
  }

  this->servo_p_pub_->publish(msg);
}

void MG400JoyInterfaceNode::publishServoJointTarget()
{
  if (!this->servo_j_pub_) {
    return;
  }

  bool needs_initialize = false;
  {
    const std::lock_guard<std::mutex> lock(this->state_mutex_);
    if (!this->servo_mode_enabled_) {
      return;
    }
    needs_initialize = !this->servo_joint_target_initialized_;
  }
  if (needs_initialize && !this->initializeServoJointTarget()) {
    return;
  }

  const auto now = this->now();
  ServoJ msg;
  {
    const std::lock_guard<std::mutex> lock(this->state_mutex_);
    if (!this->servo_mode_enabled_ || !this->servo_joint_target_initialized_) {
      return;
    }

    const double dt = this->consumeTargetElapsedSeconds(now);
    for (size_t i = 0; i < this->servo_joint_target_.size(); ++i) {
      this->servo_joint_target_[i] +=
        this->servo_joint_axes_[i] * this->joint_speed_radps_ * dt;
    }

    msg.joint_angles = this->servo_joint_target_;
    msg.t = kUnusedServoOption;
    msg.aheadtime = kUnusedServoOption;
    msg.gain = kUnusedServoOption;
  }

  this->servo_j_pub_->publish(msg);
}

double MG400JoyInterfaceNode::consumeTargetElapsedSeconds(const rclcpp::Time & now)
{
  if (timeIsUnset(this->last_servo_target_update_time_)) {
    this->last_servo_target_update_time_ = now;
    return 0.0;
  }

  const double dt = (now - this->last_servo_target_update_time_).seconds();
  this->last_servo_target_update_time_ = now;
  if (dt <= 0.0) {
    return 0.0;
  }
  return std::min(dt, kMaxServoTargetDtSec);
}

bool MG400JoyInterfaceNode::joyInputIsFresh(const rclcpp::Time & now) const
{
  if (timeIsUnset(this->last_joy_time_)) {
    return false;
  }
  return (now - this->last_joy_time_).nanoseconds() <=
         static_cast<int64_t>(this->joy_timeout_ms_) * 1000000LL;
}

void MG400JoyInterfaceNode::setPoseYaw(geometry_msgs::msg::Pose & pose, const double yaw) const
{
  const double half_yaw = yaw * 0.5;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = std::sin(half_yaw);
  pose.orientation.w = std::cos(half_yaw);
}

const char * MG400JoyInterfaceNode::servoControlTypeName(
  const ServoControlType control_type) const
{
  if (control_type == ServoControlType::SERVO_P) {
    return "ServoP";
  }
  return "ServoJ";
}
}  // namespace mg400_joy
