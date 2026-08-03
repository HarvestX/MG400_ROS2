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

#include "mg400_interface/tcp_interface/motion_tcp_interface.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>

namespace mg400_interface
{

MotionTcpInterface::MotionTcpInterface(const std::string & ip)
: MotionTcpInterface(ip, DEFAULT_PORT)
{}

MotionTcpInterface::MotionTcpInterface(const std::string & ip, uint16_t port)
{
  this->is_running_ = false;
  this->tcp_socket_ = std::make_shared<TcpSocketHandler>(ip, port);
}

MotionTcpInterface::~MotionTcpInterface()
{
  this->disConnect();
}

rclcpp::Logger MotionTcpInterface::getLogger()
{
  return rclcpp::get_logger("Motion Tcp Interface");
}

void MotionTcpInterface::init() noexcept
{
  try {
    this->is_running_ = true;
    this->thread_ = std::make_unique<std::thread>(&MotionTcpInterface::checkConnection, this);
  } catch (const TcpSocketException & err) {
    RCLCPP_ERROR(this->getLogger(), "%s", err.what());
  }
}

void MotionTcpInterface::checkConnection()
{
  using namespace std::chrono_literals; // NOLINT
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

bool MotionTcpInterface::isConnected() const
{
  return this->tcp_socket_->isConnected();
}

void MotionTcpInterface::disConnect()
{
  this->is_running_ = false;
  this->tcp_socket_->disConnect();
  if (this->thread_ && this->thread_->joinable()) {
    this->thread_->join();
  }
  this->thread_.reset();
  RCLCPP_INFO(this->getLogger(), "Close connection.");
}

void MotionTcpInterface::sendCommand(const std::string & cmd)
{
  this->tcp_socket_->send(cmd.data(), cmd.size());
}

std::string MotionTcpInterface::recvResponse(std::chrono::nanoseconds timeout)
{
  if (timeout <= std::chrono::nanoseconds::zero()) {
    this->tcp_socket_->disConnect();
    throw TcpSocketException("Motion TCP response timeout");
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::string response;
  response.reserve(256);
  std::array<char, RECEIVE_CHUNK_SIZE> chunk{};

  while (true) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      this->tcp_socket_->disConnect();
      throw TcpSocketException("Motion TCP response timeout");
    }

    const auto received = this->tcp_socket_->recvSome(
      chunk.data(), chunk.size(), deadline - now);
    if (received == 0) {
      this->tcp_socket_->disConnect();
      throw TcpSocketException("Motion TCP response timeout");
    }

    const auto end = chunk.begin() + static_cast<std::ptrdiff_t>(received);
    const auto terminator = std::find(chunk.begin(), end, ';');
    const auto frame_size = terminator == end ?
      received : static_cast<std::size_t>(std::distance(chunk.begin(), terminator)) + 1;

    if (response.size() + frame_size > MAX_RESPONSE_SIZE) {
      this->tcp_socket_->disConnect();
      throw TcpSocketException("Motion TCP response exceeded 4096 bytes");
    }
    response.append(chunk.data(), frame_size);

    if (terminator != end) {
      if (frame_size != received) {
        this->tcp_socket_->disConnect();
        throw TcpSocketException(
                "Motion TCP stream contained data after the response terminator");
      }
      RCLCPP_DEBUG(this->getLogger(), "recv: %s", response.c_str());
      return response;
    }

    if (response.size() >= MAX_RESPONSE_SIZE) {
      this->tcp_socket_->disConnect();
      throw TcpSocketException("Motion TCP response exceeded 4096 bytes");
    }
  }
}
}  // namespace mg400_interface
