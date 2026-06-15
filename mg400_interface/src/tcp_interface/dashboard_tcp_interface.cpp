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
#include <cctype>

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
        this->tcp_socket_->connect(10s);
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

bool DashboardTcpInterface::sendCommand(const std::string & cmd)
{
  if (!isValidMessageFormat(cmd)) {
    RCLCPP_ERROR(this->getLogger(), "Invalid message format: %s", cmd.c_str());
    RCLCPP_ERROR(this->getLogger(), "Expected format: MessageName(Param1,Param2,...,Paramn)");
    return false;
  }

  this->tcp_socket_->send(cmd.data(), cmd.size());
  return true;
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
  std::string response;
  if (!this->tcp_socket_->recvDelimited(response, ';', 3s)) {
    RCLCPP_WARN(this->getLogger(), "recv timeout or error");
    return "TIMEOUT ERROR";
  }
  RCLCPP_DEBUG(this->getLogger(), "recv: %s", response.c_str());
  return response;
}

bool DashboardTcpInterface::isValidMessageFormat(const std::string & cmd) const
{
  // Check if command is empty
  if (cmd.empty()) {
    return false;
  }

  // Check if command contains only ASCII characters (0-127)
  for (char c : cmd) {
    if (static_cast<unsigned char>(c) > 127) {
      return false;
    }
  }

  // Find opening and closing parentheses
  size_t open_paren = cmd.find('(');
  if (open_paren == std::string::npos || open_paren == 0) {
    return false;
  }

  // Must end with closing parenthesis
  if (cmd.back() != ')') {
    return false;
  }

  size_t close_paren = cmd.length() - 1;
  if (close_paren <= open_paren) {
    return false;
  }

  // Validate message name
  std::string message_name = cmd.substr(0, open_paren);
  if (!std::isalpha(message_name[0]) && message_name[0] != '_') {
    return false;
  }

  for (char c : message_name) {
    if (!std::isalnum(c) && c != '_') {
      return false;
    }
  }

  // Extract and validate parameters
  std::string params = cmd.substr(open_paren + 1, close_paren - open_paren - 1);

  // Parameters can be empty
  if (params.empty()) {
    return true;
  }

  // Check for invalid parameter format
  if (params.front() == ',' || params.back() == ',' ||
    params.find(",,") != std::string::npos ||
    params.find('(') != std::string::npos || params.find(')') != std::string::npos)
  {
    return false;
  }

  return true;
}

}  // namespace mg400_interface
