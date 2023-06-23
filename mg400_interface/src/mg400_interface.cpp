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
: IP(ip_address)
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

  return this->controller_error_msg_generator->loadJsonFile() &&
         this->servo_error_msg_generator->loadJsonFile();
}


bool MG400Interface::activate()
{
  using namespace std::chrono_literals;  // NOLINT
  this->dashboard_tcp_if_->init();
  this->realtime_tcp_interface->init();
  this->motion_tcp_if_->init();

  auto clock = rclcpp::Clock();
  const auto start = clock.now();

  using namespace std::chrono_literals;  // NOLINT
  if (rclcpp::sleep_for(2s) && !this->isConnected()) {
    RCLCPP_ERROR(this->getLogger(), "Could not connect to DOBOT MG400.");
    this->deactivate();
    return false;
  }

  if (this->isConnected() && !this->ok()) {
    RCLCPP_WARN(
      this->getLogger(),
      "Connection established but no data sent from DOBOT MG400. Waiting...");
    if (rclcpp::sleep_for(10s) && !this->ok()) {
      this->deactivate();
      return false;
    }
  }

  this->dashboard_commander = std::make_shared<DashboardCommander>(this->dashboard_tcp_if_.get());
  this->motion_commander = std::make_shared<MotionCommander>(this->motion_tcp_if_.get());

  RCLCPP_INFO(this->getLogger(), "Connected to DOBOT MG400");
  return true;
}

bool MG400Interface::deactivate()
{
  this->dashboard_commander.reset();
  this->motion_commander.reset();

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
  return this->isConnected() &&
         this->realtime_tcp_interface->isActive();
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
