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

#ifndef MG400_NODE__SERVO_CONTROL_ROS_INTERFACE_HPP_
#define MG400_NODE__SERVO_CONTROL_ROS_INTERFACE_HPP_

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <mg400_msgs/msg/control_state.hpp>
#include <mg400_msgs/msg/servo_error.hpp>
#include <mg400_msgs/msg/servo_j.hpp>
#include <mg400_msgs/srv/enable_servo_j.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "mg400_interface/control_state_manager.hpp"
#include "mg400_interface/servo_control_session.hpp"

namespace mg400_node
{

/// ROS endpoint and message-conversion layer for ServoControlSession.
///
/// Periodic transmission, the steady-clock watchdog, response monitoring, and
/// the stop operation deliberately remain in ServoControlSession.
class ServoControlRosInterface
{
public:
  using Manager = mg400_interface::ControlStateManager;
  using Session = mg400_interface::ServoControlSession;
  using OperationalErrorState = mg400_interface::ServoOperationalErrorState;
  using SafetyViolationState = mg400_interface::ServoSafetyViolationState;
  using EnableServoJ = mg400_msgs::srv::EnableServoJ;
  struct StopResult
  {
    bool success;
    bool stale_session;
    std::string message;
  };

  ServoControlRosInterface(
    rclcpp_lifecycle::LifecycleNode & node,
    Manager::SharedPtr manager,
    SafetyViolationState::SharedPtr safety_violation_state,
    OperationalErrorState::SharedPtr operational_error_state);
  ~ServoControlRosInterface();

  ServoControlRosInterface(const ServoControlRosInterface &) = delete;
  ServoControlRosInterface & operator=(const ServoControlRosInterface &) = delete;

  /// Install one connection-epoch Session and enable its ROS API.
  void installSession(std::shared_ptr<Session> session);

  /// Enable or reject new service starts and target callbacks.
  void setLifecycleActive(bool active);

  /// Stop the active lease for a normal Lifecycle transition.
  StopResult stopForLifecycle(const std::string & reason);

  /// Detach a stopped or stale Session. It is destroyed before this call returns.
  void clearSession();

  /// Retire a Session whose lease was already revoked by connection/mode status.
  bool retireSessionIfLeaseLost(const std::string & reason);

  bool hasSession() const;
  bool hasActiveServoLease() const;

  SafetyViolationState::SharedPtr getSafetyViolationStateShared() const noexcept;
  OperationalErrorState::SharedPtr getOperationalErrorStateShared() const noexcept;

  void publishControlState(bool force = false);
  void publishServoError();

  static mg400_msgs::msg::ServoError makeServoErrorMessage(
    const SafetyViolationState::Snapshot & safety,
    const OperationalErrorState::Snapshot & operational);

  // Public handlers keep the ROS contract testable without sockets.
  void handleEnableServoJ(
    const std::shared_ptr<EnableServoJ::Request> request,
    std::shared_ptr<EnableServoJ::Response> response);
  void handleServoJTarget(const mg400_msgs::msg::ServoJ::SharedPtr message);

private:
  rclcpp_lifecycle::LifecycleNode & node_;
  Manager::SharedPtr manager_;

  mutable std::mutex operation_mutex_;
  std::condition_variable operation_cv_;
  bool control_transition_in_progress_;
  mutable std::mutex session_pointer_mutex_;
  std::shared_ptr<Session> session_;
  bool lifecycle_active_;
  SafetyViolationState::SharedPtr safety_violation_state_;
  OperationalErrorState::SharedPtr operational_error_state_;
  std::unique_ptr<SafetyViolationState::CallbackHandle> safety_violation_callback_;
  std::unique_ptr<OperationalErrorState::CallbackHandle> operational_error_callback_;

  mutable std::mutex publication_mutex_;
  std::optional<Manager::State> last_published_state_;

  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Service<EnableServoJ>::SharedPtr enable_servo_j_service_;
  rclcpp::Subscription<mg400_msgs::msg::ServoJ>::SharedPtr servo_j_subscriber_;
  rclcpp::Publisher<mg400_msgs::msg::ControlState>::SharedPtr control_state_publisher_;
  rclcpp::Publisher<mg400_msgs::msg::ServoError>::SharedPtr servo_error_publisher_;

  void publishControlStateLocked(bool force);
  void rejectRosTarget(const std::string & reason);
};

}  // namespace mg400_node

#endif  // MG400_NODE__SERVO_CONTROL_ROS_INTERFACE_HPP_
