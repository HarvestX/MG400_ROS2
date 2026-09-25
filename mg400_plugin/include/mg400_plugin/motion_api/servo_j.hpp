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

#ifndef MG400_PLUGIN_MOTION_API_SERVO_J_HPP_
#define MG400_PLUGIN_MOTION_API_SERVO_J_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <mg400_common/mg400_ik_util.hpp>
#include <mg400_interface/mg400_interface.hpp>
#include <mg400_msgs/msg/servo_j.hpp>
#include <mg400_msgs/srv/servo_j_session.hpp>
#include <mg400_plugin_base/api_plugin_base.hpp>

namespace mg400_plugin
{

/** Manages one explicit ServoJ stream as a Motion API plugin.
 *
 * The session overlays the feedback-derived state so existing motion plugins
 * reject new goals while the stream is active.
 */
class ServoJ final : public mg400_plugin_base::MotionApiPluginBase
{
public:
  ServoJ() = default;
  ~ServoJ() override;

  void configure(
    const mg400_interface::MotionCommander::SharedPtr,
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr,
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr,
    const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr,
    const rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr,
    const mg400_interface::MG400Interface::SharedPtr) override;

  void activate() override;
  void deactivate() override;

private:
  using Setpoint = mg400_msgs::msg::ServoJ;
  using Session = mg400_msgs::srv::ServoJSession;
  using Clock = std::chrono::steady_clock;

  void onSession(const Session::Request::SharedPtr, Session::Response::SharedPtr);
  void onSetpoint(const Setpoint::SharedPtr);
  void onSendTimer();
  void clearSessionLocked();

  mg400_common::MG400IKUtil ik_util_;
  mg400_interface::MotionCommander::ServoJFormat wire_format_;
  double default_t_;
  double aheadtime_;
  double gain_;

  rclcpp::Subscription<Setpoint>::SharedPtr setpoint_sub_;
  rclcpp::Service<Session>::SharedPtr session_srv_;
  rclcpp::TimerBase::SharedPtr send_timer_;

  std::mutex mutex_;
  bool available_{false};
  bool stopping_{false};
  bool have_new_target_{false};
  bool stale_warned_{false};
  uint64_t session_id_{0};
  uint64_t next_session_id_{0};
  std::array<double, 4> target_{};
  double target_t_{0.1};
  Clock::time_point last_receive_{};
  Clock::time_point last_send_{};
};

}  // namespace mg400_plugin

#endif  // MG400_PLUGIN_MOTION_API_SERVO_J_HPP_
