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


namespace mg400_node
{
using namespace std::chrono_literals;   // NOLINT

MG400Node::MG400Node(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("mg400_node", options)
{
  this->declare_parameter<bool>("auto_connect", true);
  this->declare_parameter<std::string>("ip_address", "192.168.1.6");
  this->declare_parameter<std::vector<std::string>>(
    "dashboard_api_plugins", this->default_dashboard_api_plugins_);
  this->declare_parameter<std::vector<std::string>>(
    "motion_api_plugins", this->default_motion_api_plugins_);
  this->declare_parameter<std::string>("prefix", "");

  if (this->get_parameter("auto_connect").as_bool()) {
    RCLCPP_INFO(this->get_logger(), "Auto connect is enabled");
    this->configure();
  }
}

MG400Node::~MG400Node()
{
  this->cancelTimer();
  if (this->interface_) {
    this->interface_->deactivate();
  }
}

CallbackReturn MG400Node::on_configure(const State &)
{
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
    RCLCPP_WARN(this->get_logger(), "Failed to connect to MG400 at %s", this->ip_address_.c_str());
    if (this->get_parameter("auto_connect").as_bool()) {
      RCLCPP_INFO(this->get_logger(), "Try reconnecting in 5 seconds ...");
      this->connect_timer_ = this->create_wall_timer(5s, [this]() {this->activate();});
    }
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
  this->robot_mode_pub_ = this->create_publisher<mg400_msgs::msg::RobotMode>(
    "robot_mode", rclcpp::SensorDataQoS());
  this->error_id_pub_ = this->create_publisher<mg400_msgs::msg::ErrorID>(
    "error_id", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
  this->runTimer();

  RCLCPP_INFO(this->get_logger(), "Connected to MG400 at %s", this->ip_address_.c_str());
  this->mg400_connected_pub_->publish(std_msgs::msg::Bool().set__data(true));
  return CallbackReturn::SUCCESS;
}

CallbackReturn MG400Node::on_deactivate(const State &)
{
  RCLCPP_WARN(this->get_logger(), "Disconnected from MG400 at %s", this->ip_address_.c_str());
  this->mg400_connected_pub_->publish(std_msgs::msg::Bool().set__data(false));

  this->cancelTimer();
  this->dashboard_api_loader_.reset();
  this->motion_api_loader_.reset();
  this->joint_state_pub_.reset();
  this->robot_mode_pub_.reset();
  this->error_id_pub_.reset();
  this->interface_->deactivate();

  if (this->get_parameter("auto_connect").as_bool()) {
    RCLCPP_INFO(this->get_logger(), "Try reconnecting in 5 seconds ...");
    this->connect_timer_ = this->create_wall_timer(5s, [this]() {this->activate();});
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn MG400Node::on_cleanup(const State &)
{
  this->mg400_connected_pub_.reset();
  this->interface_.reset();
  this->connect_timer_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn MG400Node::on_shutdown(const State &)
{
  this->cancelTimer();
  this->dashboard_api_loader_.reset();
  this->motion_api_loader_.reset();
  this->joint_state_pub_.reset();
  this->robot_mode_pub_.reset();
  this->error_id_pub_.reset();
  this->mg400_connected_pub_.reset();
  this->interface_.reset();
  this->connect_timer_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn MG400Node::on_error(const State &)
{
  this->cancelTimer();
  this->dashboard_api_loader_.reset();
  this->motion_api_loader_.reset();
  this->joint_state_pub_.reset();
  this->robot_mode_pub_.reset();
  this->error_id_pub_.reset();
  this->mg400_connected_pub_.reset();
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

  try {
    auto msg = std::make_unique<mg400_msgs::msg::ErrorID>();
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
  } catch (const std::out_of_range & ex) {
    RCLCPP_ERROR(this->get_logger(), "Out of range %s", ex.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Unknown exception");
  }
}

void MG400Node::onInterfaceCheckTimer()
{
  if (!this->interface_->ok()) {
    this->deactivate();
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

}  // namespace mg400_node
