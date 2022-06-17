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

#include "mg400_interface/tcp_interface/dashboard_tcp_interface.hpp"

namespace mg400_interface
{
DashboardTcpInterface::DashboardTcpInterface(const std::string & ip)
: is_running_(false)
{
  this->tcp_socket_ = std::make_shared<TcpSocketHandler>(ip, this->PORT_);

}

DashboardTcpInterface::~DashboardTcpInterface()
{
  this->is_running_ = false;
  this->thread_->join();
}

rclcpp::Logger DashboardTcpInterface::getLogger()
{
  return rclcpp::get_logger("Dashboard Tcp Interface");
}

void DashboardTcpInterface::init() noexcept
{
  try {
    this->is_running_ = true;
    this->thread_ = std::make_unique<std::thread>(
      &DashboardTcpInterface::checkConnection, this);
  } catch (const TcpSocketException & err) {
    RCLCPP_ERROR(this->getLogger(), "%s", err.what());
  }
}

void DashboardTcpInterface::checkConnection()
{
  int failed_cnt = 0;
  while (failed_cnt < this->CONNECTION_TRIAL_) {
    if (!this->is_running_) {
      return;
    }
    try {
      if (this->tcp_socket_->isConnected()) {
        failed_cnt = 0;
        using namespace std::chrono_literals;
        rclcpp::sleep_for(1s);
        continue;
      } else {
        try {
          this->tcp_socket_->connect();
        } catch (const TcpSocketException & err) {
          RCLCPP_ERROR(
            this->getLogger(),
            "Tcp recv error : %s", err.what());
          using namespace std::chrono_literals;
          rclcpp::sleep_for(1s);
        }
      }
    } catch (const TcpSocketException & err) {
      this->tcp_socket_->disConnect();
      RCLCPP_ERROR(
        this->getLogger(),
        "Tcp rec error : %s", err.what());
    }
    failed_cnt++;
  }

  RCLCPP_ERROR(
    this->getLogger(),
    "Failed more than %d times... Close connection.",
    this->CONNECTION_TRIAL_);
  std::terminate();
}

bool DashboardTcpInterface::isConnected()
{
  return this->tcp_socket_->isConnected();
}

void DashboardTcpInterface::sendCommand(const std::string & cmd)
{
  this->tcp_socket_->send(cmd.data(), cmd.size());
}

std::string DashboardTcpInterface::recvResponse()
{
  char buf[100];
  this->tcp_socket_->recv(buf, sizeof(buf), 1000);
  RCLCPP_INFO(
    this->getLogger(),
    "recv: %s", std::string(buf).c_str());
  return std::string(buf);
}

void DashboardTcpInterface::waitForResponse()
{
  using namespace std::chrono_literals;
  rclcpp::sleep_for(500ms);
}
}  // namespace mg400_interface
