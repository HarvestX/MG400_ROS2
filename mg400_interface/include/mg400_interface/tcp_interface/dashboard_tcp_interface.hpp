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

#pragma once

#include <cstdlib>

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mg400_interface/tcp_interface/tcp_socket_handler.hpp"

namespace mg400_interface
{
using namespace std::chrono_literals;  // NOLINT
class DashboardTcpInterfaceBase
{
public:
  DashboardTcpInterfaceBase() {}
  virtual void sendCommand(const std::string &) = 0;
  virtual std::string recvResponse() = 0;
};

class DashboardTcpInterface : public DashboardTcpInterfaceBase
{
public:
  using UniquePtr = std::unique_ptr<DashboardTcpInterface>;

private:
  const uint16_t PORT_ = 29999;

  std::atomic<bool> is_running_;
  std::mutex mutex_;
  std::unique_ptr<std::thread> thread_;
  std::shared_ptr<TcpSocketHandler> tcp_socket_;

public:
  DashboardTcpInterface() = delete;
  explicit DashboardTcpInterface(const std::string &);
  ~DashboardTcpInterface();
  void init() noexcept;

  static rclcpp::Logger getLogger();
  bool isConnected();
  void sendCommand(const std::string &) override;
  std::string recvResponse(void) override;
  void disConnect();

private:
  void checkConnection();
};
}  // namespace mg400_interface
