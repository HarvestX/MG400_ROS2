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

bool MG400Interface::configure()
{
  this->dashboard_tcp_if_ =
    std::make_unique<mg400_interface::DashboardTcpInterface>(this->IP);
  this->motion_tcp_if_ =
    std::make_unique<mg400_interface::MotionTcpInterface>(this->IP);
  this->realtime_tcp_interface =
    std::make_unique<mg400_interface::RealtimeFeedbackTcpInterface>(this->IP);

  this->error_msg_generator =
    std::make_unique<mg400_interface::ErrorMsgGenerator>(
    "alarm_controller.json");
}


bool MG400Interface::activate()
{
  using namespace std::chrono_literals;
  this->dashboard_tcp_if_->init();
  this->realtime_tcp_interface->init();
  this->motion_tcp_if_->init();

  auto clock = rclcpp::Clock();
  const auto start = clock.now();
  while (!this->dashboard_tcp_if_->isConnected() ||
    !this->realtime_tcp_interface->isConnected() ||
    !this->motion_tcp_if_->isConnected())
  {
    if (clock.now() - start > rclcpp::Duration(3s)) {
      RCLCPP_ERROR(
        this->getLogger(),
        "Could not connect DOBOT MG400.");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_WARN(
      this->getLogger(),
      "Waiting for the connection...");
    rclcpp::sleep_for(1s);
  }

  this->dashboard_commander =
    std::make_unique<mg400_interface::DashboardCommander>(
    this->dashboard_tcp_if_.get());
  this->motion_commander =
    std::make_unique<mg400_interface::MotionCommander>(
    this->motion_tcp_if_.get());
}

bool MG400Interface::deactivate()
{
  this->dashboard_commander = nullptr;
  this->motion_commander = nullptr;
  this->dashboard_tcp_if_->disConnect();
  this->motion_tcp_if_->disConnect();
  this->realtime_tcp_interface->disConnect();
}

const rclcpp::Logger MG400Interface::getLogger()
{
  return rclcpp::get_logger("MG400Interface");
}
}  // namespace mg400_interface
