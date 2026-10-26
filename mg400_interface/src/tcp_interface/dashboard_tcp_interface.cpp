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
  std::string response;
  constexpr int kChunkSize = 128;  // Larger chunk for better efficiency
  constexpr int kMaxResponseSize = 4096;  // Maximum response size limit

  response.reserve(256);  // Pre-allocate memory to reduce reallocations

  while (response.size() < kMaxResponseSize) {
    try {
      // Use stack buffer for temporary storage
      std::array<char, kChunkSize> buffer{};

      uint32_t bytes_received = 0;
      this->tcp_socket_->recv(buffer.data(), kChunkSize, bytes_received, 500ms);

      if (bytes_received == 0) {
        // No data received, break the loop
        break;
      }

      // Find semicolon using efficient search
      auto * semicolon_pos = std::find(buffer.begin(), buffer.end(), ';');

      if (bytes_received > 0) {
        if (semicolon_pos != buffer.end()) {
          // Found terminator - append up to and including semicolon
          size_t bytes_to_copy = std::distance(buffer.begin(), semicolon_pos) + 1;
          response.append(buffer.data(), bytes_to_copy);

          RCLCPP_DEBUG(this->getLogger(), "recv: %s", response.c_str());
          return response;
        } else {
          // No terminator found - append entire buffer
          response.append(buffer.data(), bytes_received);
        }
      }

    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->getLogger(), "Error receiving data: %s", e.what());
      break;
    }
  }

  if (response.size() >= kMaxResponseSize) {
    RCLCPP_ERROR(this->getLogger(), "Response size exceeded maximum limit");
  }

  RCLCPP_DEBUG(this->getLogger(), "recv (incomplete): %s", response.c_str());
  return response;
}

}  // namespace mg400_interface
