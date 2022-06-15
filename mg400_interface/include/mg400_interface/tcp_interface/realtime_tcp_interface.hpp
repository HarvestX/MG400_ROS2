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

#include "mg400_interface/joint_handler.hpp"
#include "mg400_interface/tcp_interface/realtime_data.hpp"
#include "mg400_interface/tcp_interface/tcp_socket_handler.hpp"

namespace mg400_interface
{
#pragma pack(push, 1)
class RealtimeTcpInterface
{
private:
  const uint16_t PORT_ = 30003;
  const int CONNECTION_TRIAL_ = 3;

  std::mutex mutex_;
  std::array<double, 6> current_joints_;
  RealTimeData rt_data_;
  std::atomic<bool> is_running_;
  std::unique_ptr<std::thread> thread_;
  std::shared_ptr<TcpSocketHandler> tcp_socket_;

public:
  explicit RealtimeTcpInterface(const std::string &);
  ~RealtimeTcpInterface();
  void init() noexcept;

  static rclcpp::Logger getLogger();
  void getCurrentJointStates(std::array<double, 6> &);
  RealTimeData getRealtimeData();
  bool isConnected();

  // DOBOT MG400 Official Command ---------------------------------------------
  void movJ(
    const double, const double, const double,
    const double, const double, const double);

  void moveJog(const std::string &);
  // End DOBOT MG400 Official Command -----------------------------------------

private:
  void recvData();
  void sendCommand(const std::string &);
};
}  // namespace mg400_interface
