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

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
#include <future>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "mg400_interface/tcp_interface/motion_tcp_interface.hpp"

namespace
{
using namespace std::chrono_literals;  // NOLINT
using mg400_interface::MotionCommandQueueFullException;
using mg400_interface::MotionResponse;
using mg400_interface::MotionResponseResult;
using mg400_interface::MotionTcpInterface;
using mg400_interface::MotionTcpInterfaceOptions;

class LoopbackServer
{
public:
  using ConnectionHandler = std::function<void (int, size_t)>;

  explicit LoopbackServer(size_t connection_count, ConnectionHandler handler)
  : listen_fd_(::socket(AF_INET, SOCK_STREAM, 0)),
    client_fd_(-1),
    port_(0),
    accepted_count_(0)
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
        this->listen_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0)
    {
      ::close(this->listen_fd_);
      throw std::runtime_error("bind failed");
    }
    if (::listen(this->listen_fd_, 4) < 0) {
      ::close(this->listen_fd_);
      throw std::runtime_error("listen failed");
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
      [this, connection_count, handler]() {
        for (size_t index = 0; index < connection_count; ++index) {
          const int client = ::accept(this->listen_fd_, nullptr, nullptr);
          if (client < 0) {
            return;
          }
          this->client_fd_.store(client);
          {
            std::lock_guard<std::mutex> lock(this->mutex_accepted_);
            ++this->accepted_count_;
          }
          this->cv_accepted_.notify_all();

          try {
            handler(client, index);
          } catch (const std::exception & error) {
            std::lock_guard<std::mutex> lock(this->mutex_error_);
            this->error_ = error.what();
          }

          int expected = client;
          this->client_fd_.compare_exchange_strong(expected, -1);
          ::shutdown(client, SHUT_RDWR);
          ::close(client);
        }
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

  bool waitForAcceptedCount(size_t count, std::chrono::nanoseconds timeout)
  {
    std::unique_lock<std::mutex> lock(this->mutex_accepted_);
    return this->cv_accepted_.wait_for(
      lock, timeout, [this, count]() {return this->accepted_count_ >= count;});
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
    bool found_open_bracket = false;
    while (true) {
      char byte = 0;
      const auto received = ::recv(socket, &byte, 1, 0);
      if (received <= 0) {
        throw std::runtime_error("connection closed while reading command");
      }
      command.push_back(byte);
      if (byte == '(') {
        found_open_bracket = true;
        ++depth;
      } else if (byte == ')') {
        --depth;
        if (found_open_bracket && depth == 0) {
          return command;
        }
      }
    }
  }

  static void sendAll(int socket, const std::string & data)
  {
    size_t offset = 0;
    while (offset < data.size()) {
      const auto sent = ::send(
        socket, data.data() + offset, data.size() - offset, MSG_NOSIGNAL);
      if (sent <= 0) {
        throw std::runtime_error("send failed");
      }
      offset += static_cast<size_t>(sent);
    }
  }

private:
  int listen_fd_;
  std::atomic<int> client_fd_;
  uint16_t port_;
  std::thread thread_;
  mutable std::mutex mutex_error_;
  std::string error_;
  std::mutex mutex_accepted_;
  std::condition_variable cv_accepted_;
  size_t accepted_count_;
};

MotionTcpInterfaceOptions testOptions()
{
  MotionTcpInterfaceOptions options;
  options.command_timeout = 500ms;
  options.receive_poll_interval = 2ms;
  options.reconnect_interval = 5ms;
  options.connect_timeout = 100ms;
  options.pending_capacity = 32;
  options.completed_capacity = 64;
  options.maximum_response_length = 256;
  return options;
}

MotionResponse waitForResponse(MotionTcpInterface & interface)
{
  MotionResponse response;
  EXPECT_TRUE(interface.waitForResponse(response, 2s));
  return response;
}

bool waitForCompletedCount(
  MotionTcpInterface & interface, size_t expected, std::chrono::nanoseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (interface.getCompletedResponseCount() >= expected) {
      return true;
    }
    std::this_thread::yield();
  }
  return interface.getCompletedResponseCount() >= expected;
}

bool waitForDroppedCount(
  MotionTcpInterface & interface, uint64_t expected, std::chrono::nanoseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (interface.getDroppedCompletedResponseCount() >= expected) {
      return true;
    }
    std::this_thread::yield();
  }
  return interface.getDroppedCompletedResponseCount() >= expected;
}

TEST(MotionTcpInterface, ReassemblesResponseSplitAcrossRecvCalls)
{
  std::mutex mutex_fragment;
  std::condition_variable cv_fragment;
  bool first_fragment_sent = false;
  bool send_second_fragment = false;
  LoopbackServer server(
    1, [&](int client, size_t) {
      EXPECT_EQ(LoopbackServer::receiveCommand(client), "MovJ(1,2,3,4)");
      LoopbackServer::sendAll(client, "0,{12},Mov");
      {
        std::lock_guard<std::mutex> lock(mutex_fragment);
        first_fragment_sent = true;
      }
      cv_fragment.notify_all();
      std::unique_lock<std::mutex> lock(mutex_fragment);
      cv_fragment.wait_for(lock, 2s, [&]() {return send_second_fragment;});
      LoopbackServer::sendAll(client, "J(1,2,3,4);");
    });

  MotionTcpInterface interface("127.0.0.1", server.port(), testOptions());
  interface.init();
  ASSERT_TRUE(interface.waitUntilConnected(2s));
  interface.sendCommand("MovJ(1,2,3,4)");

  {
    std::unique_lock<std::mutex> lock(mutex_fragment);
    ASSERT_TRUE(cv_fragment.wait_for(lock, 2s, [&]() {return first_fragment_sent;}));
  }
  MotionResponse incomplete;
  EXPECT_FALSE(interface.waitForResponse(incomplete, 20ms));
  {
    std::lock_guard<std::mutex> lock(mutex_fragment);
    send_second_fragment = true;
  }
  cv_fragment.notify_all();

  const auto response = waitForResponse(interface);
  EXPECT_EQ(response.result, MotionResponseResult::SUCCESS);
  EXPECT_EQ(response.raw_response, "0,{12},MovJ(1,2,3,4);");
  EXPECT_EQ(response.command, "MovJ(1,2,3,4)");
  EXPECT_EQ(response.error_code, 0);
  EXPECT_EQ(interface.getPendingCommandCount(), 0U);
}

TEST(MotionTcpInterface, SplitsCombinedResponsesAndCorrelatesFifo)
{
  LoopbackServer server(
    1, [](int client, size_t) {
      const auto first = LoopbackServer::receiveCommand(client);
      const auto second = LoopbackServer::receiveCommand(client);
      LoopbackServer::sendAll(
        client, "0,{1}," + first + ";0,{2}," + second + ";");
    });

  MotionTcpInterface interface("127.0.0.1", server.port(), testOptions());
  interface.init();
  ASSERT_TRUE(interface.waitUntilConnected(2s));
  interface.sendCommand("MovJ(1,2,3,4)");
  interface.sendCommand("MovL(4,3,2,1)");

  const auto first = waitForResponse(interface);
  const auto second = waitForResponse(interface);
  EXPECT_EQ(first.command, "MovJ(1,2,3,4)");
  EXPECT_EQ(second.command, "MovL(4,3,2,1)");
  EXPECT_LT(first.sequence_id, second.sequence_id);
  EXPECT_EQ(first.result, MotionResponseResult::SUCCESS);
  EXPECT_EQ(second.result, MotionResponseResult::SUCCESS);
  EXPECT_EQ(interface.getPendingCommandCount(), 0U);
}

TEST(MotionTcpInterface, ParsesControllerErrorAndPublishesLatestError)
{
  LoopbackServer server(
    1, [](int client, size_t) {
      const auto command = LoopbackServer::receiveCommand(client);
      LoopbackServer::sendAll(client, "-10000,{}," + command + ";");
    });

  MotionTcpInterface interface("127.0.0.1", server.port(), testOptions());
  interface.init();
  ASSERT_TRUE(interface.waitUntilConnected(2s));
  interface.sendCommand("MovJ(1,2,3,4)");

  const auto response = waitForResponse(interface);
  EXPECT_EQ(response.result, MotionResponseResult::CONTROLLER_ERROR);
  EXPECT_EQ(response.error_code, -10000);
  MotionResponse latest_error;
  ASSERT_TRUE(interface.getLatestError(latest_error));
  EXPECT_EQ(latest_error.sequence_id, response.sequence_id);
}

TEST(MotionTcpInterface, CommandNameMismatchIsParseError)
{
  LoopbackServer server(
    1, [](int client, size_t) {
      static_cast<void>(LoopbackServer::receiveCommand(client));
      LoopbackServer::sendAll(client, "0,{},MovL(4,3,2,1);");
    });

  MotionTcpInterface interface("127.0.0.1", server.port(), testOptions());
  interface.init();
  ASSERT_TRUE(interface.waitUntilConnected(2s));
  interface.sendCommand("MovJ(1,2,3,4)");

  const auto response = waitForResponse(interface);
  EXPECT_EQ(response.result, MotionResponseResult::PARSE_ERROR);
  EXPECT_EQ(response.command, "MovJ(1,2,3,4)");
}

TEST(MotionTcpInterface, TimeoutCompletesCommandAndResetsConnection)
{
  auto options = testOptions();
  options.command_timeout = 30ms;
  LoopbackServer server(
    1, [](int client, size_t) {
      static_cast<void>(LoopbackServer::receiveCommand(client));
      char byte = 0;
      static_cast<void>(::recv(client, &byte, 1, 0));
    });

  MotionTcpInterface interface("127.0.0.1", server.port(), options);
  interface.init();
  ASSERT_TRUE(interface.waitUntilConnected(2s));
  interface.sendCommand("MovJ(1,2,3,4)");

  const auto response = waitForResponse(interface);
  EXPECT_EQ(response.result, MotionResponseResult::TIMEOUT);
  EXPECT_EQ(interface.getPendingCommandCount(), 0U);
}

TEST(MotionTcpInterface, DisconnectCompletesAllPendingCommands)
{
  LoopbackServer server(
    1, [](int client, size_t) {
      static_cast<void>(LoopbackServer::receiveCommand(client));
      static_cast<void>(LoopbackServer::receiveCommand(client));
    });

  MotionTcpInterface interface("127.0.0.1", server.port(), testOptions());
  interface.init();
  ASSERT_TRUE(interface.waitUntilConnected(2s));
  interface.sendCommand("MovJ(1,2,3,4)");
  interface.sendCommand("MovL(4,3,2,1)");

  EXPECT_EQ(waitForResponse(interface).result, MotionResponseResult::DISCONNECTED);
  EXPECT_EQ(waitForResponse(interface).result, MotionResponseResult::DISCONNECTED);
  EXPECT_EQ(interface.getPendingCommandCount(), 0U);
}

TEST(MotionTcpInterface, ReconnectDoesNotCorrelateOldPendingCommand)
{
  LoopbackServer server(
    2, [](int client, size_t index) {
      const auto command = LoopbackServer::receiveCommand(client);
      if (index == 1) {
        LoopbackServer::sendAll(client, "0,{}," + command + ";");
      }
    });

  MotionTcpInterface interface("127.0.0.1", server.port(), testOptions());
  interface.init();
  ASSERT_TRUE(interface.waitUntilConnected(2s));
  interface.sendCommand("MovJ(1,2,3,4)");
  const auto old_response = waitForResponse(interface);
  ASSERT_EQ(old_response.result, MotionResponseResult::DISCONNECTED);

  ASSERT_TRUE(server.waitForAcceptedCount(2, 2s));
  ASSERT_TRUE(interface.waitUntilConnected(2s));
  interface.sendCommand("MovL(4,3,2,1)");
  const auto new_response = waitForResponse(interface);
  EXPECT_EQ(new_response.result, MotionResponseResult::SUCCESS);
  EXPECT_EQ(new_response.command, "MovL(4,3,2,1)");
  EXPECT_GT(new_response.sequence_id, old_response.sequence_id);
}

TEST(MotionTcpInterface, RejectsCommandWhenPendingQueueIsFull)
{
  auto options = testOptions();
  options.pending_capacity = 2;
  options.completed_capacity = 4;
  LoopbackServer server(
    1, [](int client, size_t) {
      static_cast<void>(LoopbackServer::receiveCommand(client));
      static_cast<void>(LoopbackServer::receiveCommand(client));
      char byte = 0;
      static_cast<void>(::recv(client, &byte, 1, 0));
    });

  MotionTcpInterface interface("127.0.0.1", server.port(), options);
  interface.init();
  ASSERT_TRUE(interface.waitUntilConnected(2s));
  interface.sendCommand("MovJ(1,2,3,4)");
  interface.sendCommand("MovL(4,3,2,1)");
  EXPECT_THROW(interface.sendCommand("Sync()"), MotionCommandQueueFullException);
  EXPECT_EQ(interface.getPendingCommandCount(), 2U);
}

TEST(MotionTcpInterface, CompletedQueueOverflowIsBoundedAndObservable)
{
  auto options = testOptions();
  options.pending_capacity = 2;
  options.completed_capacity = 2;
  LoopbackServer server(
    1, [](int client, size_t) {
      for (size_t index = 0; index < 3; ++index) {
        const auto command = LoopbackServer::receiveCommand(client);
        LoopbackServer::sendAll(client, "0,{}," + command + ";");
      }
    });

  MotionTcpInterface interface("127.0.0.1", server.port(), options);
  interface.init();
  ASSERT_TRUE(interface.waitUntilConnected(2s));
  interface.sendCommand("MovJ(1,2,3,4)");
  ASSERT_TRUE(waitForCompletedCount(interface, 1, 2s));
  interface.sendCommand("MovL(4,3,2,1)");
  ASSERT_TRUE(waitForCompletedCount(interface, 2, 2s));
  interface.sendCommand("Sync()");
  ASSERT_TRUE(waitForDroppedCount(interface, 1, 2s));
  EXPECT_EQ(interface.getCompletedResponseCount(), 2U);
  EXPECT_EQ(interface.getDroppedCompletedResponseCount(), 1U);
  EXPECT_EQ(waitForResponse(interface).command, "MovL(4,3,2,1)");
  EXPECT_EQ(waitForResponse(interface).command, "Sync()");
}

TEST(MotionTcpInterface, SustainedResponsesKeepPendingQueueBounded)
{
  constexpr size_t command_count = 200;
  LoopbackServer server(
    1, [](int client, size_t) {
      for (size_t index = 0; index < command_count; ++index) {
        const auto command = LoopbackServer::receiveCommand(client);
        LoopbackServer::sendAll(client, "0,{}," + command + ";");
      }
    });

  MotionTcpInterface interface("127.0.0.1", server.port(), testOptions());
  interface.init();
  ASSERT_TRUE(interface.waitUntilConnected(2s));
  for (size_t index = 0; index < command_count; ++index) {
    const auto command = "MovJ(" + std::to_string(index) + ",0,0,0)";
    interface.sendCommand(command);
    const auto response = waitForResponse(interface);
    ASSERT_EQ(response.result, MotionResponseResult::SUCCESS);
    ASSERT_EQ(response.command, command);
  }
  EXPECT_EQ(interface.getPendingCommandCount(), 0U);
  EXPECT_EQ(interface.getCompletedResponseCount(), 0U);
}

TEST(MotionTcpInterface, OversizedUnterminatedResponseFailsPendingCommand)
{
  auto options = testOptions();
  options.maximum_response_length = 16;
  LoopbackServer server(
    1, [](int client, size_t) {
      static_cast<void>(LoopbackServer::receiveCommand(client));
      LoopbackServer::sendAll(client, "0,{},MovJ(12345678901234567890");
    });

  MotionTcpInterface interface("127.0.0.1", server.port(), options);
  interface.init();
  ASSERT_TRUE(interface.waitUntilConnected(2s));
  interface.sendCommand("MovJ(1,2,3,4)");
  EXPECT_EQ(waitForResponse(interface).result, MotionResponseResult::PARSE_ERROR);
  EXPECT_EQ(interface.getPendingCommandCount(), 0U);
}

TEST(MotionTcpInterface, DisconnectJoinsReceiverWithoutDeadlock)
{
  LoopbackServer server(
    1, [](int client, size_t) {
      char byte = 0;
      static_cast<void>(::recv(client, &byte, 1, 0));
    });

  MotionTcpInterface interface("127.0.0.1", server.port(), testOptions());
  interface.init();
  ASSERT_TRUE(interface.waitUntilConnected(2s));
  auto waiter = std::async(
    std::launch::async, [&interface]() {
      MotionResponse response;
      return interface.waitForResponse(response, 5s);
    });
  auto disconnect = std::async(std::launch::async, [&interface]() {interface.disConnect();});
  EXPECT_EQ(disconnect.wait_for(1s), std::future_status::ready);
  disconnect.get();
  EXPECT_EQ(waiter.wait_for(1s), std::future_status::ready);
  EXPECT_FALSE(waiter.get());
}

}  // namespace
