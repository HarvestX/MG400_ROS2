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

#include "mg400_interface/tcp_interface/tcp_socket_handler.hpp"

namespace mg400_interface
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("TcpClient");

TcpSocketHandler::TcpSocketHandler(std::string ip, uint16_t port)
: fd_(-1),
  port_(port),
  ip_(std::move(ip))
{
  this->is_connected_.store(false);
}

TcpSocketHandler::~TcpSocketHandler()
{
  this->close();
}

void TcpSocketHandler::close()
{
  if (this->fd_ < 0) {
    // TcpClient not connected
    // Do nothing
    return;
  }

  ::close(this->fd_);
  this->is_connected_.store(false);
  this->fd_ = -1;
}

void TcpSocketHandler::connect(const std::chrono::nanoseconds & timeout)
{
  if (this->fd_ < 0) {
    this->fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (this->fd_ < 0) {
      throw TcpSocketException(this->toString() + std::string(" socket : ") + strerror(errno));
    }

    timeval tv = {0, 0};
    tv.tv_sec = timeout.count() / static_cast<int>(1e9);
    tv.tv_usec = (timeout.count() % static_cast<int>(1e9)) / static_cast<int>(1e3);
    if (::setsockopt(
        this->fd_, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<char *>(&tv), sizeof(tv)) < 0)
    {
      ::close(this->fd_);
      this->fd_ = -1;
      throw TcpSocketException(this->toString() + std::string(" socket : ") + strerror(errno));
    }
  }

  sockaddr_in addr = {};

  memset(&addr, 0, sizeof(addr));
  inet_pton(AF_INET, this->ip_.c_str(), &addr.sin_addr);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(this->port_);

  if (::connect(this->fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    ::close(this->fd_);
    this->fd_ = -1;
    if (errno == EINPROGRESS || errno == EAGAIN) {
      throw  TcpSocketException(this->toString() + std::string(" connect : timeout"));
    } else {
      throw  TcpSocketException(this->toString() + std::string(" connect : ") + strerror(errno));
    }
  }

  this->is_connected_.store(true);

  RCLCPP_INFO(LOGGER, "%s : connected successfully", this->toString().c_str());
}

void TcpSocketHandler::disConnect()
{
  if (this->is_connected_.load()) {
    ::close(this->fd_);
    this->is_connected_.store(false);
    this->fd_ = -1;
  }
}

bool TcpSocketHandler::isConnected() const
{
  return this->is_connected_.load();
}

void TcpSocketHandler::send(const void * buf, uint32_t len)
{
  if (!this->is_connected_.load()) {
    throw TcpSocketException("tcp is disconnected");
  }

  RCLCPP_INFO(LOGGER, "send : %s", (const char *)buf);

  const auto * tmp = (const uint8_t *)buf;
  while (len) {
    int err = static_cast<int>(::send(fd_, tmp, len, MSG_NOSIGNAL));
    if (err < 0) {
      this->disConnect();
      throw TcpSocketException(this->toString() + std::string(" ::send() ") + strerror(errno));
    }
    len -= err;
    tmp += err;
  }
}

bool TcpSocketHandler::recv(void * buf, uint32_t len, const std::chrono::nanoseconds & timeout)
{
  uint8_t * tmp = reinterpret_cast<uint8_t *>(buf);
  fd_set read_fds;
  timeval tv = {0, 0};

  while (len) {
    FD_ZERO(&read_fds);
    FD_SET(this->fd_, &read_fds);

    tv.tv_sec = timeout.count() / static_cast<int>(1e9);
    tv.tv_usec = (timeout.count() % static_cast<int>(1e9)) / static_cast<int>(1e3);
    int err = ::select(this->fd_ + 1, &read_fds, nullptr, nullptr, &tv);
    if (err < 0) {
      this->disConnect();
      throw TcpSocketException(this->toString() + std::string(" select() : ") + strerror(errno));
    } else if (err == 0) {
      return false;
    }
    err = static_cast<int>(::read(fd_, tmp, len));
    if (err < 0) {
      this->disConnect();
      throw TcpSocketException(this->toString() + std::string(" ::read() ") + strerror(errno));
    } else if (err == 0) {
      this->disConnect();
      throw TcpSocketException(this->toString() + std::string(" tcp server has disconnected."));
    }
    len -= err;
    tmp += err;
  }
  return true;
}

std::string TcpSocketHandler::toString()
{
  return this->ip_ + ":" + std::to_string(this->port_);
}

}  // namespace mg400_interface
