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

#include "mg400_interface/dashboard_commander.hpp"

namespace mg400_interface
{
DashboardCommander::DashboardCommander(const std::string & ip)
{
  this->tcp_socket_ = std::make_shared<TcpSocketHandler>(ip, this->PORT_);
}

DashboardCommander::~DashboardCommander()
{
  this->thread_->join();
}

rclcpp::Logger DashboardCommander::getLogger()
{
  return rclcpp::get_logger("Dashboard Commander");
}

void DashboardCommander::init() noexcept
{
  try {
    this->thread_ = std::make_unique<std::thread>(
      &DashboardCommander::checkConnection, this);
  } catch (const TcpSocketException & err) {
    RCLCPP_ERROR(this->getLogger(), "%s", err.what());
  }
}

void DashboardCommander::checkConnection()
{
  int failed_cnt = 0;
  while (failed_cnt < 3) {
    try {
      if (this->tcp_socket_->isConnected()) {
        failed_cnt = 0;
      } else {
        failed_cnt += 1;
        try {
          this->tcp_socket_->connect();
        } catch (const TcpSocketException & err) {
          RCLCPP_ERROR(
            this->getLogger(),
            "Tcp recv error : %s", err.what());
          using namespace std::chrono_literals;
          rclcpp::sleep_for(3s);
        }
      }
    } catch (const TcpSocketException & err) {
      this->tcp_socket_->disConnect();
      RCLCPP_ERROR(
        this->getLogger(),
        "Tcp rec error : %s", err.what());
    }
  }

  throw std::runtime_error("Connection failed");
}

std::string DashboardCommander::sendCommand(const std::string & cmd)
{
  this->tcp_socket_->send(cmd.data(), cmd.size());

  char buf[100];
  this->tcp_socket_->recv(buf, sizeof(buf), 1000);

  RCLCPP_INFO(this->getLogger(), "recv: %s", buf);
  return std::string(buf);
}

void DashboardCommander::enableRobot()
{
  this->sendCommand("EnableRobot()");
}

void DashboardCommander::enableRobot(const double load)
{
  if (load < 0.0 || load > 0.5) {
    RCLCPP_ERROR(
      this->getLogger(), "Given load %.3lf is out of range (0~0.5 kg)",
      load);
    return;
  }

  std::string buf;
  buf.resize(100);
  snprintf(buf.data(), buf.size(), "EnableRobot(%.3lf)", load);
  this->sendCommand(buf);
}

void DashboardCommander::enableRobot(
  const double load, const double cx, const double cy, const double cz)
{
  if (load < 0.0 || load > 0.5) {
    RCLCPP_ERROR(
      this->getLogger(), "Given load %.3lf is out of range (0~0.5 kg)",
      load);
    return;
  }

  if (std::abs(cx) > 500.0 || std::abs(cy) > 500.0 || std::abs(cz) > 500.0) {
    RCLCPP_ERROR(
      this->getLogger(),
      "Given center (%.3lf, %.3lf, %.3lf) is out of range ( < 500.0 mm)",
      cx, cy, cz);
    return;
  }

  std::string buf;
  buf.resize(100);
  snprintf(
    buf.data(), buf.size(),
    "EnableRobot(%.3lf, %.3lf. %.3lf, %.3lf)", load, cx, cy, cz);
  this->sendCommand(buf);
}

void DashboardCommander::disableRobot()
{
  this->sendCommand("DisableRobot()");
}

void DashboardCommander::clearError()
{
  this->sendCommand("ClearError()");
}

void DashboardCommander::getErrorId()
{
  this->sendCommand("GetErrorID()");
}

}  // namespace mg400_interface
