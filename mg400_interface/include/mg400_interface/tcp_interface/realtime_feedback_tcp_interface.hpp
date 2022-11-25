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


#include <string>
#include <memory>

#include <mg400_msgs/msg/end_pose.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mg400_interface/joint_handler.hpp"
#include "mg400_interface/tcp_interface/realtime_data.hpp"
#include "mg400_interface/tcp_interface/tcp_socket_handler.hpp"


namespace mg400_interface
{

class RealtimeFeedbackTcpInterface
{
public:
  using SharedPtr = std::shared_ptr<RealtimeFeedbackTcpInterface>;
  const std::string frame_id_prefix;

private:
  const uint16_t PORT_ = 30004;

  std::mutex mutex_;
  std::array<double, 4> current_joints_;
  RealTimeData rt_data_;
  std::atomic<bool> is_running_;
  std::unique_ptr<std::thread> thread_;
  std::shared_ptr<TcpSocketHandler> tcp_socket_;

public:
  RealtimeFeedbackTcpInterface() = delete;
  explicit RealtimeFeedbackTcpInterface(
    const std::string &, const std::string & = "");
  ~RealtimeFeedbackTcpInterface();
  void init() noexcept;

  static rclcpp::Logger getLogger();
  bool isConnected();

  void getCurrentJointStates(std::array<double, 4> &);
  void getCurrentEndPose(
    mg400_msgs::msg::EndPose &, const bool && = true);
  RealTimeData getRealtimeData();
  uint64_t getRobotMode();
  bool isRobotMode(const uint64_t &) const;
  void disConnect();

private:
  void recvData();
};
}  // namespace mg400_interface
