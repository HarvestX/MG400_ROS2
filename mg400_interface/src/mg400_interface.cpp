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

#include "mg400_interface/mg400_interface.hpp"

namespace mg400_interface
{

MG400Interface::MG400Interface(const std::string & ip_address)
: IP(ip_address),
  control_state_manager_(std::make_shared<ControlStateManager>())
{
}

bool MG400Interface::configure(const std::string & frame_id_prefix)
{
  this->dashboard_tcp_if_ = std::make_unique<DashboardTcpInterface>(this->IP);
  this->motion_tcp_if_ = std::make_unique<MotionTcpInterface>(this->IP);
  this->realtime_tcp_interface = std::make_shared<RealtimeFeedbackTcpInterface>(
    this->IP, frame_id_prefix);

  this->controller_error_msg_generator =
    std::make_unique<ErrorMsgGenerator>("alarm_controller.json");
  this->servo_error_msg_generator =
    std::make_unique<ErrorMsgGenerator>("alarm_servo.json");

  this->dashboard_commander = std::make_shared<DashboardCommander>(this->dashboard_tcp_if_.get());
  this->motion_commander = std::make_shared<MotionCommander>(this->motion_tcp_if_.get());

  return this->controller_error_msg_generator->loadJsonFile() &&
         this->servo_error_msg_generator->loadJsonFile();
}


bool MG400Interface::activate()
{
  using namespace std::chrono_literals;  // NOLINT
  this->dashboard_tcp_if_->init();
  this->realtime_tcp_interface->init();
  this->motion_tcp_if_->init();

  // Wait for the connection to be established.
  auto clock = rclcpp::Clock();
  const auto start = clock.now();
  const auto timeout = rclcpp::Duration::from_seconds(10.0);
  while ((clock.now() - start) < timeout) {
    if (this->isConnected()) {
      break;
    }
    rclcpp::sleep_for(500ms);
    RCLCPP_INFO(this->getLogger(), "Connecting to DOBOT MG400 at %s ...", this->IP.c_str());
  }
  if (!this->isConnected()) {
    RCLCPP_ERROR(this->getLogger(), "Could not connect to DOBOT MG400.");
    this->deactivate();
    return false;
  }

  // Wait for the data to be received.
  const auto data_timeout = rclcpp::Duration::from_seconds(10.0);
  const auto data_start = clock.now();
  while ((clock.now() - data_start) < data_timeout) {
    if (this->ok()) {
      break;
    }
    rclcpp::sleep_for(500ms);
  }
  if (!this->ok()) {
    RCLCPP_ERROR(this->getLogger(), "Connection established but no data received.");
    this->deactivate();
    return false;
  }

  RCLCPP_INFO(this->getLogger(), "Connected to DOBOT MG400");
  return true;
}

bool MG400Interface::deactivate()
{
  this->control_state_manager_->updateRobotStatus(false, 0);

  // disconnect each interface in parallel because it takes time sometimes.
  std::thread discnt_dashboard_tcp_if_([this]() {this->dashboard_tcp_if_->disConnect();});
  std::thread discnt_realtime_tcp_if_([this]() {this->realtime_tcp_interface->disConnect();});
  std::thread discnt_motion_tcp_if_([this]() {this->motion_tcp_if_->disConnect();});
  discnt_dashboard_tcp_if_.join();
  discnt_realtime_tcp_if_.join();
  discnt_motion_tcp_if_.join();

  return true;
}

bool MG400Interface::ok()
{
  // When MG400 is being initialized when booting up, realtime tcp interface
  // will be connected but not active yet.
  // We assume MG400Interface is ok when realtime tcp interface is active.
  const bool active = this->isConnected() &&
    this->realtime_tcp_interface->isActive();
  uint64_t robot_mode = 0;
  const bool robot_mode_available = active &&
    this->realtime_tcp_interface->getRobotMode(robot_mode);
  this->control_state_manager_->updateRobotStatus(
    robot_mode_available, robot_mode);
  return robot_mode_available;
}

ControlStateManager & MG400Interface::getControlStateManager() noexcept
{
  return *this->control_state_manager_;
}

const ControlStateManager & MG400Interface::getControlStateManager() const noexcept
{
  return *this->control_state_manager_;
}

ControlStateManager::SharedPtr MG400Interface::getControlStateManagerShared() noexcept
{
  return this->control_state_manager_;
}

const rclcpp::Logger MG400Interface::getLogger() noexcept
{
  return rclcpp::get_logger("MG400Interface");
}

bool MG400Interface::isConnected()
{
  return this->dashboard_tcp_if_->isConnected() &&
         this->realtime_tcp_interface->isConnected() &&
         this->motion_tcp_if_->isConnected();
}
}  // namespace mg400_interface
