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

#ifndef MG400_NODE__SERVO_J_CONTROLLER_HPP_
#define MG400_NODE__SERVO_J_CONTROLLER_HPP_

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
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace mg400_node
{

/** Manages one explicit ServoJ stream without changing the motion plugin API.
 *
 * The session overlays the feedback-derived state so existing motion plugins
 * reject new goals while the stream is active.
 */
class ServoJController
{
public:
  ServoJController(
    rclcpp_lifecycle::LifecycleNode & node,
    mg400_interface::MG400Interface::SharedPtr interface,
    const std::string & wire_format, double default_t, double aheadtime, double gain);
  ~ServoJController();

  void activate();
  void deactivate();

private:
  using ServoJ = mg400_msgs::msg::ServoJ;
  using Session = mg400_msgs::srv::ServoJSession;
  using Clock = std::chrono::steady_clock;

  void onSession(const Session::Request::SharedPtr, Session::Response::SharedPtr);
  void onSetpoint(const ServoJ::SharedPtr);
  void onSendTimer();
  void clearSessionLocked();

  rclcpp_lifecycle::LifecycleNode & node_;
  mg400_interface::MG400Interface::SharedPtr interface_;
  mg400_common::MG400IKUtil ik_util_;
  mg400_interface::MotionCommander::ServoJFormat wire_format_;
  double default_t_;
  double aheadtime_;
  double gain_;

  rclcpp::Subscription<ServoJ>::SharedPtr setpoint_sub_;
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

}  // namespace mg400_node

#endif  // MG400_NODE__SERVO_J_CONTROLLER_HPP_
