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
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "mg400_interface/joint_handler.hpp"

#include "mg400_interface/tcp_interface/realtime_data.hpp"
#include "mg400_interface/tcp_interface/tcp_socket_handler.hpp"

namespace mg400_interface
{
class MotionTcpInterfaceBase
{
public:
  MotionTcpInterfaceBase() {}
  virtual ~MotionTcpInterfaceBase() = default;
  virtual void sendCommand(const std::string &) = 0;
  virtual std::string recvResponse(std::chrono::nanoseconds) = 0;
};

class MotionTcpInterface : public MotionTcpInterfaceBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MotionTcpInterface)
  RCLCPP_UNIQUE_PTR_DEFINITIONS(MotionTcpInterface)

private:
  static constexpr uint16_t DEFAULT_PORT = 30003;
  static constexpr std::size_t MAX_RESPONSE_SIZE = 4096;
  static constexpr std::size_t RECEIVE_CHUNK_SIZE = 512;

  std::atomic<bool> is_running_;
  std::unique_ptr<std::thread> thread_;
  std::shared_ptr<TcpSocketHandler> tcp_socket_;

public:
  MotionTcpInterface() = delete;
  explicit MotionTcpInterface(const std::string &);
  MotionTcpInterface(const std::string &, uint16_t);
  ~MotionTcpInterface() override;
  void init() noexcept;

  static rclcpp::Logger getLogger();
  bool isConnected() const;
  void sendCommand(const std::string &) override;
  std::string recvResponse(std::chrono::nanoseconds) override;
  void disConnect();

private:
  void checkConnection();
};
}  // namespace mg400_interface
#endif
