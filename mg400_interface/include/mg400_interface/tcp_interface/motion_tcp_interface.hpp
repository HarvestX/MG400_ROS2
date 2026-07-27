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

#ifndef __MG400_INTERFACE_TCP_INTERFACE_MOTION_TCP_INTERFACE_HPP__
#define __MG400_INTERFACE_TCP_INTERFACE_MOTION_TCP_INTERFACE_HPP__

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "mg400_interface/tcp_interface/tcp_socket_handler.hpp"

namespace mg400_interface
{

enum class MotionResponseResult
{
  SUCCESS,
  CONTROLLER_ERROR,
  TIMEOUT,
  DISCONNECTED,
  PARSE_ERROR
};

struct MotionResponse
{
  static constexpr int ERROR_CODE_UNAVAILABLE = std::numeric_limits<int>::min();

  uint64_t sequence_id = 0;
  std::string command;
  std::string raw_response;
  int error_code = ERROR_CODE_UNAVAILABLE;
  std::chrono::steady_clock::time_point sent_at;
  std::chrono::steady_clock::time_point received_at;
  MotionResponseResult result = MotionResponseResult::PARSE_ERROR;
};

struct MotionTcpInterfaceOptions
{
  std::chrono::nanoseconds command_timeout = std::chrono::seconds(1);
  std::chrono::nanoseconds receive_poll_interval = std::chrono::milliseconds(10);
  std::chrono::nanoseconds reconnect_interval = std::chrono::milliseconds(100);
  std::chrono::nanoseconds connect_timeout = std::chrono::milliseconds(500);
  size_t pending_capacity = 256;
  size_t completed_capacity = 512;
  size_t maximum_response_length = 4096;
};

class MotionCommandQueueFullException : public std::runtime_error
{
public:
  explicit MotionCommandQueueFullException(const std::string & message)
  : std::runtime_error(message)
  {}
};

class MotionTcpInterfaceBase
{
public:
  MotionTcpInterfaceBase() {}
  virtual ~MotionTcpInterfaceBase() = default;
  virtual void sendCommand(const std::string &) = 0;

  // Defaults preserve compatibility with lightweight mocks and third-party
  // implementations that only provide the original send-only API.
  virtual bool tryTakeResponse(MotionResponse &) {return false;}
  virtual bool waitForResponse(MotionResponse &, const std::chrono::nanoseconds &) {return false;}
  virtual bool getLatestResponse(MotionResponse &) const {return false;}
  virtual bool getLatestError(MotionResponse &) const {return false;}
  virtual size_t getPendingCommandCount() const {return 0;}
  virtual uint64_t getDroppedCompletedResponseCount() const {return 0;}
};

class MotionTcpInterface : public MotionTcpInterfaceBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MotionTcpInterface)
  RCLCPP_UNIQUE_PTR_DEFINITIONS(MotionTcpInterface)

private:
  struct PendingCommand
  {
    uint64_t sequence_id;
    uint64_t connection_generation;
    std::string command;
    std::string command_name;
    std::chrono::steady_clock::time_point sent_at;
    std::chrono::steady_clock::time_point timeout_at;
  };

  static constexpr uint16_t DEFAULT_PORT = 30003;
  static constexpr size_t RECEIVE_CHUNK_SIZE = 1024;

  const MotionTcpInterfaceOptions options_;
  std::atomic<bool> is_running_;
  std::unique_ptr<std::thread> thread_;
  std::shared_ptr<TcpSocketHandler> tcp_socket_;

  mutable std::mutex mutex_lifecycle_;
  mutable std::mutex mutex_send_;
  mutable std::mutex mutex_state_;
  std::condition_variable cv_stop_;
  std::condition_variable cv_response_;
  std::condition_variable cv_connection_;
  std::mutex mutex_stop_;

  uint64_t next_sequence_id_;
  uint64_t connection_generation_;
  std::deque<PendingCommand> pending_commands_;
  std::deque<MotionResponse> completed_responses_;
  MotionResponse latest_response_;
  MotionResponse latest_error_;
  bool has_latest_response_;
  bool has_latest_error_;
  uint64_t dropped_completed_response_count_;

public:
  MotionTcpInterface() = delete;
  explicit MotionTcpInterface(const std::string &);
  MotionTcpInterface(
    const std::string &, uint16_t, const MotionTcpInterfaceOptions & = MotionTcpInterfaceOptions());
  ~MotionTcpInterface();
  void init() noexcept;

  static rclcpp::Logger getLogger();
  bool isConnected() const;
  bool waitUntilConnected(const std::chrono::nanoseconds &);
  void sendCommand(const std::string &) override;
  bool tryTakeResponse(MotionResponse &) override;
  bool waitForResponse(MotionResponse &, const std::chrono::nanoseconds &) override;
  bool getLatestResponse(MotionResponse &) const override;
  bool getLatestError(MotionResponse &) const override;
  size_t getPendingCommandCount() const override;
  uint64_t getDroppedCompletedResponseCount() const override;
  size_t getCompletedResponseCount() const;
  void disConnect();

private:
  void receiveLoop();
  bool connectSocket();
  void waitBeforeReconnect();
  bool processReceivedBytes(std::string &, const char *, size_t);
  bool processResponseFrame(const std::string &);
  bool expireTimedOutCommands();
  void handleConnectionLoss(const std::string &);
  void handleProtocolFailure(const std::string &);
  void completeFrontAsProtocolError(const std::string &);
  void failAllPending(MotionResponseResult, const std::chrono::steady_clock::time_point &);
  void addCompletedResponse(const MotionResponse &);
  static std::string takeCommandName(const std::string &);
  static bool commandNamesEqual(const std::string &, const std::string &);
};
}  // namespace mg400_interface
#endif
