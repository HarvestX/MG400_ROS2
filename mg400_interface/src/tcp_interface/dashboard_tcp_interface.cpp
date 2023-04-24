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
{
  this->is_running_.store(false);
  this->tcp_socket_ = std::make_shared<TcpSocketHandler>(ip, this->PORT_);
}

DashboardTcpInterface::~DashboardTcpInterface()
{
  this->disConnect();
}

rclcpp::Logger DashboardTcpInterface::getLogger()
{
  return rclcpp::get_logger("Dashboard Tcp Interface");
}

void DashboardTcpInterface::init() noexcept
{
  try {
    this->is_running_.store(true);
    this->thread_ = std::make_unique<std::thread>(&DashboardTcpInterface::checkConnection, this);
  } catch (const TcpSocketException & err) {
    RCLCPP_ERROR(this->getLogger(), "%s", err.what());
  }
}

void DashboardTcpInterface::checkConnection()
{
  static const int CONNECTION_TRIAL = 3;
  using namespace std::chrono_literals;
  int failed_cnt = 0;
  while (failed_cnt < CONNECTION_TRIAL) {
    if (!this->is_running_.load()) {
      return;
    }
    try {
      if (this->tcp_socket_->isConnected()) {
        failed_cnt = 0;
        rclcpp::sleep_for(1s);
        continue;
      } else {
        try {
          this->tcp_socket_->connect();
        } catch (const TcpSocketException & err) {
          RCLCPP_ERROR(this->getLogger(), "Tcp recv error : %s", err.what());
          rclcpp::sleep_for(500ms);
          failed_cnt++;
        }
      }
    } catch (const TcpSocketException & err) {
      this->tcp_socket_->disConnect();
      RCLCPP_ERROR(this->getLogger(), "Tcp recv error : %s", err.what());
      rclcpp::sleep_for(500ms);
      failed_cnt++;
    }
  }

  RCLCPP_ERROR(this->getLogger(), "Failed more than %d times... Close connection.", failed_cnt);
  this->is_running_.store(false);
}

bool DashboardTcpInterface::isConnected()
{
  return this->tcp_socket_->isConnected();
}

void DashboardTcpInterface::sendCommand(const std::string & cmd)
{
  this->tcp_socket_->send(cmd.data(), cmd.size());
}

void DashboardTcpInterface::disConnect()
{
  this->is_running_.store(false);
  if (this->thread_->joinable()) {
    this->thread_->join();
  }
  this->tcp_socket_->disConnect();
  RCLCPP_INFO(this->getLogger(), "Close connection.");
}

std::string DashboardTcpInterface::recvResponse()
{
  char buf[100];
  this->tcp_socket_->recv(buf, sizeof(buf), 500);
  RCLCPP_DEBUG(this->getLogger(), "recv: %s", std::string(buf).c_str());
  return std::string(buf);
}
}  // namespace mg400_interface
