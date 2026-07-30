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

#ifndef __MG400_INTERFACE_TCP_INTERFACE_REALTIME_FEEDBACK_TCP_INTERFACE_HPP__
#define __MG400_INTERFACE_TCP_INTERFACE_REALTIME_FEEDBACK_TCP_INTERFACE_HPP__


#include <atomic>
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mg400_interface/joint_handler.hpp"
#include "mg400_interface/tcp_interface/realtime_data.hpp"
#include "mg400_interface/tcp_interface/realtime_data_snapshot.hpp"
#include "mg400_interface/tcp_interface/tcp_socket_handler.hpp"


namespace mg400_interface
{

class RealtimeFeedbackTcpInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RealtimeFeedbackTcpInterface)
  RCLCPP_UNIQUE_PTR_DEFINITIONS(RealtimeFeedbackTcpInterface)
  using RealtimeDataCallback = std::function<void (const RealTimeData &)>;

  const std::string frame_id_prefix;

private:
  using Pose = geometry_msgs::msg::Pose;
  const uint16_t PORT_ = 30004;

  mutable std::mutex mutex_realtime_data_;
  std::mutex mutex_realtime_data_callback_;
  RealtimeDataSnapshot realtime_data_snapshot_;
  RealtimeDataCallback realtime_data_callback_;
  std::atomic<bool> is_running_;
  std::unique_ptr<std::thread> thread_;
  TcpSocketHandler::SharedPtr tcp_socket_;

public:
  RealtimeFeedbackTcpInterface() = delete;
  explicit RealtimeFeedbackTcpInterface(
    const std::string &, const std::string & = "");
  ~RealtimeFeedbackTcpInterface();
  void init() noexcept;

  static rclcpp::Logger getLogger();
  bool isConnected();
  bool isActive() const;

  bool getCurrentJointStates(std::array<double, 4> &) const;
  bool getCurrentEndPose(Pose &) const;
  RealtimeDataSnapshot getLatestRealtimeData() const;
  bool getRobotMode(uint64_t &) const;
  bool isRobotMode(const uint64_t &) const;
  void setRealtimeDataCallback(RealtimeDataCallback callback);
  void disConnect();

private:
  void beginConnectionEpoch();
  void invalidateRealtimeData();
  void recvData();
};
}  // namespace mg400_interface
#endif
