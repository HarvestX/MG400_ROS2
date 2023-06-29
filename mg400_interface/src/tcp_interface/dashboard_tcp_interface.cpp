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
using namespace std::chrono_literals; // NOLINT

DashboardTcpInterface::DashboardTcpInterface(const std::string & ip)
{
  this->is_running_ = false;
  this->tcp_socket_ = std::make_shared<TcpSocketHandler>(ip, this->PORT_);
}

DashboardTcpInterface::~DashboardTcpInterface()
{
  if (this->is_running_) {
    this->disConnect();
  }
}

rclcpp::Logger DashboardTcpInterface::getLogger()
{
  return rclcpp::get_logger("Dashboard Tcp Interface");
}

void DashboardTcpInterface::init() noexcept
{
  try {
    this->is_running_ = true;
    this->thread_ = std::make_unique<std::thread>(&DashboardTcpInterface::checkConnection, this);
  } catch (const TcpSocketException & err) {
    RCLCPP_ERROR(this->getLogger(), "%s", err.what());
  }
}

void DashboardTcpInterface::checkConnection()
{
  while (this->is_running_) {
    try {
      if (!this->tcp_socket_->isConnected()) {
        this->tcp_socket_->connect(1s);
      } else {
        rclcpp::sleep_for(1s);
        continue;
      }
    } catch (const TcpSocketException & err) {
      this->tcp_socket_->disConnect();
      RCLCPP_ERROR(this->getLogger(), "Tcp recv error : %s", err.what());
      return;
    }
  }
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
  this->is_running_ = false;
  if (this->thread_->joinable()) {
    this->thread_->join();
  }
  this->tcp_socket_->disConnect();
  RCLCPP_INFO(this->getLogger(), "Close connection.");
}

std::string DashboardTcpInterface::recvResponse()
{
  char buf[100];
  this->tcp_socket_->recv(buf, sizeof(buf), 500ms);
  RCLCPP_DEBUG(this->getLogger(), "recv: %s", std::string(buf).c_str());
  return std::string(buf);
}
}  // namespace mg400_interface
