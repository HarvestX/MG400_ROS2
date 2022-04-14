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

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <cerrno>
#include <utility>
#include "mg400_control/mg400_interface/tcp_socket.hpp"

namespace mg400_control
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("TcpClient");

TcpClient::TcpClient(
  std::string ip,
  uint16_t port
)
: fd_(-1),
  port_(port),
  ip_(std::move(ip)),
  is_connected_(false)
{}

TcpClient::~TcpClient()
{
  this->close();
}

void TcpClient::close()
{
  if (this->fd_ < 0) {
    // TcpClient not connected
    // Do nothing
    return;
  }

  ::close(this->fd_);
  this->is_connected_ = false;
  this->fd_ = -1;
}

void TcpClient::connect()
{
  if (this->fd_ < 0) {
    this->fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (this->fd_ < 0) {
      throw TcpClientException(
              this->toString() + std::string(" socket : ") + strerror(errno));
    }
  }

  sockaddr_in addr = {};

  memset(&addr, 0, sizeof(addr));
  inet_pton(AF_INET, this->ip_.c_str(), &addr.sin_addr);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(this->port_);

  if (::connect(
      this->fd_,
      reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
  {
    throw  TcpClientException(
            this->toString() + std::string(" connect : ") + strerror(errno));
  }

  this->is_connected_ = true;

  RCLCPP_INFO(
    LOGGER,
    "%s : connected successfully", this->toString().c_str());
}

void TcpClient::disConnect()
{
  if (this->is_connected_) {
    this->fd_ = -1;
    this->is_connected_ = false;
    ::close(this->fd_);
  }
}

bool TcpClient::isConnected() const
{
  return this->is_connected_;
}

void TcpClient::send(const void * buf, uint32_t len)
{
  if (!this->is_connected_) {
    throw TcpClientException("tcp is disconnected");
  }

  RCLCPP_INFO(
    LOGGER,
    "send : %s",
    (const char *)buf);

  const auto * tmp = (const uint8_t *)buf;
  while (len) {
    int err = static_cast<int>(::send(fd_, tmp, len, MSG_NOSIGNAL));
    if (err < 0) {
      this->disConnect();
      throw TcpClientException(
              this->toString() + std::string(" ::send() ") + strerror(errno));
    }
    len -= err;
    tmp += err;
  }
}

bool TcpClient::recv(void * buf, uint32_t len, uint32_t timeout)
{
  uint8_t * tmp = reinterpret_cast<uint8_t *>(buf);
  fd_set read_fds;
  timeval tv = {0, 0};

  while (len) {
    FD_ZERO(&read_fds);
    FD_SET(this->fd_, &read_fds);

    tv.tv_sec = timeout / 1000;
    tv.tv_usec = (timeout % 1000) * 1000;
    int err = ::select(this->fd_ + 1, &read_fds, nullptr, nullptr, &tv);
    if (err < 0) {
      this->disConnect();
      throw TcpClientException(
              this->toString() + std::string(" select() : ") + strerror(errno));
    } else if (err == 0) {
      return false;
    }

    err = static_cast<int>(::read(fd_, tmp, len));
    if (err < 0) {
      this->disConnect();
      throw TcpClientException(
              this->toString() + std::string(" ::read() ") + strerror(errno));
    } else if (err == 0) {
      this->disConnect();
      throw TcpClientException(
              this->toString() + std::string(" tcp server has disconnected."));
    }
    len -= err;
    tmp += err;
  }
  return true;
}

std::string TcpClient::toString()
{
  return this->ip_ + ":" + std::to_string(this->port_);
}

}  // namespace mg400_control
