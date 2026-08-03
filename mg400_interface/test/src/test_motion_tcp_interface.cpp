// Copyright 2026 HarvestX Inc.
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

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <gtest/gtest.h>

#include "mg400_interface/tcp_interface/motion_tcp_interface.hpp"

namespace
{

using namespace std::chrono_literals;  // NOLINT
using mg400_interface::MotionTcpInterface;
using mg400_interface::TcpSocketException;

class LoopbackServer
{
public:
  using Handler = std::function<void (int)>;

  explicit LoopbackServer(Handler handler)
  : listen_fd_(::socket(AF_INET, SOCK_STREAM, 0))
  {
    if (this->listen_fd_ < 0) {
      throw std::runtime_error("socket failed");
    }

    int reuse = 1;
    if (::setsockopt(
        this->listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
    {
      ::close(this->listen_fd_);
      throw std::runtime_error("setsockopt failed");
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    if (::bind(
        this->listen_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0 ||
      ::listen(this->listen_fd_, 2) < 0)
    {
      ::close(this->listen_fd_);
      throw std::runtime_error("bind or listen failed");
    }

    socklen_t length = sizeof(address);
    if (::getsockname(
        this->listen_fd_, reinterpret_cast<sockaddr *>(&address), &length) < 0)
    {
      ::close(this->listen_fd_);
      throw std::runtime_error("getsockname failed");
    }
    this->port_ = ntohs(address.sin_port);

    this->thread_ = std::thread(
      [this, handler = std::move(handler)]() {
        const int client = ::accept(this->listen_fd_, nullptr, nullptr);
        if (client < 0) {
          return;
        }
        this->client_fd_.store(client);
        try {
          handler(client);
        } catch (const std::exception & error) {
          std::lock_guard<std::mutex> lock(this->mutex_error_);
          this->error_ = error.what();
        }
        int expected = client;
        this->client_fd_.compare_exchange_strong(expected, -1);
        ::shutdown(client, SHUT_RDWR);
        ::close(client);
      });
  }

  ~LoopbackServer()
  {
    const int client = this->client_fd_.load();
    if (client >= 0) {
      ::shutdown(client, SHUT_RDWR);
    }
    ::shutdown(this->listen_fd_, SHUT_RDWR);
    ::close(this->listen_fd_);
    if (this->thread_.joinable()) {
      this->thread_.join();
    }
  }

  uint16_t port() const
  {
    return this->port_;
  }

  std::string error() const
  {
    std::lock_guard<std::mutex> lock(this->mutex_error_);
    return this->error_;
  }

  static std::string receiveCommand(int socket)
  {
    std::string command;
    int depth = 0;
    bool found_open_parenthesis = false;
    while (true) {
      char byte = 0;
      if (::recv(socket, &byte, 1, 0) <= 0) {
        throw std::runtime_error("connection closed while receiving command");
      }
      command.push_back(byte);
      if (byte == '(') {
        found_open_parenthesis = true;
        ++depth;
      } else if (byte == ')') {
        --depth;
        if (found_open_parenthesis && depth == 0) {
          return command;
        }
      }
    }
  }

  static void sendAll(int socket, const std::string & data)
  {
    std::size_t offset = 0;
    while (offset < data.size()) {
      const auto sent = ::send(
        socket, data.data() + offset, data.size() - offset, MSG_NOSIGNAL);
      if (sent <= 0) {
        throw std::runtime_error("send failed");
      }
      offset += static_cast<std::size_t>(sent);
    }
  }

private:
  int listen_fd_;
  std::atomic<int> client_fd_{-1};
  uint16_t port_{0};
  std::thread thread_;
  mutable std::mutex mutex_error_;
  std::string error_;
};

bool waitUntilConnected(MotionTcpInterface & interface)
{
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (interface.isConnected()) {
      return true;
    }
    std::this_thread::sleep_for(1ms);
  }
  return interface.isConnected();
}

template<typename Predicate>
bool waitUntil(Predicate predicate)
{
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(1ms);
  }
  return predicate();
}

TEST(MotionTcpInterface, ReassemblesResponseSplitAcrossReads)
{
  const std::string command = "ServoJ(1.000,2.000,3.000,4.000)";
  const std::string response = "0,{}," + command + ";";
  LoopbackServer server(
    [&](int client) {
      if (LoopbackServer::receiveCommand(client) != command) {
        throw std::runtime_error("unexpected command");
      }
      LoopbackServer::sendAll(client, response.substr(0, 9));
      std::this_thread::sleep_for(20ms);
      LoopbackServer::sendAll(client, response.substr(9));
    });

  MotionTcpInterface interface("127.0.0.1", server.port());
  interface.init();
  ASSERT_TRUE(waitUntilConnected(interface));
  interface.sendCommand(command);

  EXPECT_EQ(response, interface.recvResponse(500ms));
  EXPECT_TRUE(server.error().empty()) << server.error();
}

TEST(MotionTcpInterface, ReturnsAtTerminatorWithoutWaitingForPeerClose)
{
  const std::string command = "Sync()";
  LoopbackServer server(
    [&](int client) {
      static_cast<void>(LoopbackServer::receiveCommand(client));
      LoopbackServer::sendAll(client, "0,{}," + command + ";");
      std::this_thread::sleep_for(300ms);
    });

  MotionTcpInterface interface("127.0.0.1", server.port());
  interface.init();
  ASSERT_TRUE(waitUntilConnected(interface));
  interface.sendCommand(command);
  const auto start = std::chrono::steady_clock::now();

  EXPECT_EQ("0,{},Sync();", interface.recvResponse(500ms));
  EXPECT_LT(std::chrono::steady_clock::now() - start, 200ms);
}

TEST(MotionTcpInterface, TimeoutDisconnectsTheCurrentConnection)
{
  std::atomic<bool> peer_closed{false};
  LoopbackServer server(
    [&](int client) {
      static_cast<void>(LoopbackServer::receiveCommand(client));
      char byte = 0;
      peer_closed.store(::recv(client, &byte, 1, 0) == 0);
    });

  MotionTcpInterface interface("127.0.0.1", server.port());
  interface.init();
  ASSERT_TRUE(waitUntilConnected(interface));
  interface.sendCommand("Sync()");

  EXPECT_THROW(interface.recvResponse(20ms), TcpSocketException);
  EXPECT_TRUE(waitUntil([&]() {return peer_closed.load();}));
}

TEST(MotionTcpInterface, RejectsDataFollowingTerminatorAndDisconnects)
{
  std::atomic<bool> peer_closed{false};
  LoopbackServer server(
    [&](int client) {
      const auto command = LoopbackServer::receiveCommand(client);
      LoopbackServer::sendAll(client, "0,{}," + command + ";unexpected");
      char byte = 0;
      peer_closed.store(::recv(client, &byte, 1, 0) == 0);
    });

  MotionTcpInterface interface("127.0.0.1", server.port());
  interface.init();
  ASSERT_TRUE(waitUntilConnected(interface));
  interface.sendCommand("Sync()");

  EXPECT_THROW(interface.recvResponse(500ms), TcpSocketException);
  EXPECT_TRUE(waitUntil([&]() {return peer_closed.load();}));
}

TEST(MotionTcpInterface, PeerCloseRaisesAnException)
{
  LoopbackServer server(
    [](int client) {
      static_cast<void>(LoopbackServer::receiveCommand(client));
    });

  MotionTcpInterface interface("127.0.0.1", server.port());
  interface.init();
  ASSERT_TRUE(waitUntilConnected(interface));
  interface.sendCommand("Sync()");

  EXPECT_THROW(interface.recvResponse(500ms), TcpSocketException);
}

TEST(MotionTcpInterface, OversizedResponseRaisesAnException)
{
  LoopbackServer server(
    [](int client) {
      static_cast<void>(LoopbackServer::receiveCommand(client));
      LoopbackServer::sendAll(client, std::string(4096, 'x'));
    });

  MotionTcpInterface interface("127.0.0.1", server.port());
  interface.init();
  ASSERT_TRUE(waitUntilConnected(interface));
  interface.sendCommand("Sync()");

  EXPECT_THROW(interface.recvResponse(500ms), TcpSocketException);
}

}  // namespace
