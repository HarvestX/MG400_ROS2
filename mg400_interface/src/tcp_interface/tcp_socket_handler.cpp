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
  this->disConnect();
}

void TcpSocketHandler::connect(const std::chrono::nanoseconds & timeout)
{
  std::lock_guard<std::mutex> lock(this->mutex_connect_);
  if (this->isConnected()) {
    return;
  }

  int fd = this->fd_.load();
  if (fd < 0) {
    fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
      throw TcpSocketException(this->toString() + std::string(" socket : ") + strerror(errno));
    }
    this->fd_.store(fd);

    timeval tv = {0, 0};
    tv.tv_sec = timeout.count() / static_cast<int>(1e9);
    tv.tv_usec = (timeout.count() % static_cast<int>(1e9)) / static_cast<int>(1e3);
    if (::setsockopt(
        fd, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<char *>(&tv), sizeof(tv)) < 0)
    {
      const int error_number = errno;
      this->closeSocket(fd);
      throw TcpSocketException(
              this->toString() + std::string(" socket : ") + strerror(error_number));
    }
  }

  sockaddr_in addr = {};

  memset(&addr, 0, sizeof(addr));
  inet_pton(AF_INET, this->ip_.c_str(), &addr.sin_addr);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(this->port_);

  if (::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    const int error_number = errno;
    this->closeSocket(fd);
    if (error_number == EINPROGRESS || error_number == EAGAIN) {
      throw  TcpSocketException(this->toString() + std::string(" connect : timeout"));
    } else {
      throw  TcpSocketException(
              this->toString() + std::string(" connect : ") + strerror(error_number));
    }
  }

  if (this->fd_.load() != fd) {
    throw TcpSocketException(this->toString() + std::string(" connect : cancelled"));
  }
  this->is_connected_.store(true);

  RCLCPP_INFO(LOGGER, "%s : connected successfully", this->toString().c_str());
}

void TcpSocketHandler::disConnect()
{
  this->is_connected_.store(false);
  const int fd = this->fd_.exchange(-1);
  if (fd >= 0) {
    ::shutdown(fd, SHUT_RDWR);
    ::close(fd);
  }
}

bool TcpSocketHandler::isConnected() const
{
  return this->is_connected_.load();
}

void TcpSocketHandler::send(const void * buf, uint32_t len)
{
  const int fd = this->fd_.load();
  if (!this->is_connected_.load() || fd < 0) {
    throw TcpSocketException("tcp is disconnected");
  }

  RCLCPP_DEBUG(LOGGER, "send : %.*s", static_cast<int>(len), (const char *)buf);

  const auto * tmp = (const uint8_t *)buf;
  while (len) {
    const int err = static_cast<int>(::send(fd, tmp, len, MSG_NOSIGNAL));
    if (err < 0 && errno == EINTR) {
      continue;
    }
    if (err <= 0) {
      const int error_number = err < 0 ? errno : EPIPE;
      this->closeSocket(fd);
      throw TcpSocketException(
              this->toString() + std::string(" ::send() ") + strerror(error_number));
    }
    len -= err;
    tmp += err;
  }
}

std::size_t TcpSocketHandler::recvSome(
  void * buf, std::size_t capacity, const std::chrono::nanoseconds & timeout)
{
  if (capacity == 0) {
    return 0;
  }

  const int fd = this->fd_.load();
  if (!this->is_connected_.load() || fd < 0) {
    throw TcpSocketException("tcp is disconnected");
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (true) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return 0;
    }
    const auto remaining = deadline - now;
    auto remaining_us = std::chrono::duration_cast<std::chrono::microseconds>(remaining);
    if (remaining_us <= std::chrono::microseconds::zero()) {
      remaining_us = std::chrono::microseconds(1);
    }

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);
    timeval tv = {0, 0};
    tv.tv_sec = remaining_us.count() / static_cast<int>(1e6);
    tv.tv_usec = remaining_us.count() % static_cast<int>(1e6);

    const int select_result = ::select(fd + 1, &read_fds, nullptr, nullptr, &tv);
    if (select_result < 0 && errno == EINTR) {
      continue;
    }
    if (select_result < 0) {
      const int error_number = errno;
      this->closeSocket(fd);
      throw TcpSocketException(
              this->toString() + std::string(" select() : ") + strerror(error_number));
    }
    if (select_result == 0 || FD_ISSET(fd, &read_fds) == 0) {
      return 0;
    }

    const auto read_result = ::read(fd, buf, capacity);
    if (read_result < 0 && errno == EINTR) {
      continue;
    }
    if (read_result < 0) {
      const int error_number = errno;
      if (error_number == EAGAIN || error_number == EWOULDBLOCK) {
        continue;
      }
      this->closeSocket(fd);
      throw TcpSocketException(
              this->toString() + std::string(" ::read() ") + strerror(error_number));
    }
    if (read_result == 0) {
      this->closeSocket(fd);
      throw TcpSocketException(this->toString() + std::string(" tcp server has disconnected."));
    }
    return static_cast<std::size_t>(read_result);
  }
}

bool TcpSocketHandler::recv(void * buf, uint32_t len, const std::chrono::nanoseconds & timeout)
{
  uint8_t * tmp = reinterpret_cast<uint8_t *>(buf);

  while (len) {
    const auto received = this->recvSome(tmp, len, timeout);
    if (received == 0) {
      return false;
    }
    len -= static_cast<uint32_t>(received);
    tmp += received;
  }
  return true;
}

bool TcpSocketHandler::recv(
  void * buf, uint32_t len, uint32_t & received_data_len,
  const std::chrono::nanoseconds & timeout)
{
  uint8_t * tmp = reinterpret_cast<uint8_t *>(buf);
  received_data_len = 0;

  while (len) {
    const auto received = this->recvSome(tmp, len, timeout);
    if (received == 0) {
      return false;
    }
    len -= static_cast<uint32_t>(received);
    tmp += received;
    received_data_len += static_cast<uint32_t>(received);
  }
  return true;
}

std::string TcpSocketHandler::toString()
{
  return this->ip_ + ":" + std::to_string(this->port_);
}

void TcpSocketHandler::closeSocket(int fd)
{
  int expected = fd;
  if (this->fd_.compare_exchange_strong(expected, -1)) {
    this->is_connected_.store(false);
    ::shutdown(fd, SHUT_RDWR);
    ::close(fd);
  }
}

}  // namespace mg400_interface
