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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mg400_msgs/msg/control_state.hpp>
#include <mg400_msgs/msg/servo_j.hpp>
#include <mg400_msgs/msg/servo_p.hpp>
#include <mg400_msgs/srv/change_control_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
  using ChangeControlState = mg400_msgs::srv::ChangeControlState;
  using TransformPoseFunction = std::function<bool (
        const geometry_msgs::msg::PoseStamped &,
        const std::string &,
        geometry_msgs::msg::PoseStamped &,
        std::string &)>;

  struct Options
  {
    std::chrono::nanoseconds diagnostics_period = std::chrono::milliseconds(100);
    std::string target_frame = "mg400_origin_link";
    std::string hardware_id = "mg400";
  };

  struct StopResult
  {
    bool success;
    bool stale_session;
    std::string message;
  };

  ServoControlRosInterface(
    rclcpp_lifecycle::LifecycleNode & node,
    Manager::SharedPtr manager,
    const Options & options,
    TransformPoseFunction transform_pose = TransformPoseFunction());
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
  void clearSession(const std::string & reason);

  /// Retire a Session whose lease was already revoked by connection/mode status.
  bool retireSessionIfLeaseLost(const std::string & reason);

  bool hasSession() const;

  void publishControlState(bool force = false);
  void publishDiagnostics();

  // Public handlers keep the ROS contract testable without sockets.
  void handleChangeControlState(
    const std::shared_ptr<ChangeControlState::Request> request,
    std::shared_ptr<ChangeControlState::Response> response);
  void handleServoJTarget(const mg400_msgs::msg::ServoJ::SharedPtr message);
  void handleServoPTarget(const mg400_msgs::msg::ServoP::SharedPtr message);

  std::uint64_t getRosRejectedTargetCount() const noexcept;
  std::string getLastRosRejection() const;

  static bool quaternionToYaw(
    const geometry_msgs::msg::Quaternion & quaternion,
    double & yaw,
    std::string & reason);

private:
  rclcpp_lifecycle::LifecycleNode & node_;
  Manager::SharedPtr manager_;
  Options options_;

  mutable std::mutex operation_mutex_;
  mutable std::mutex session_pointer_mutex_;
  std::shared_ptr<Session> session_;
  bool lifecycle_active_;
  std::optional<Session::Snapshot> retired_snapshot_;
  std::string retired_reason_;

  mutable std::mutex diagnostic_mutex_;
  std::atomic<std::uint64_t> ros_rejected_target_count_;
  std::string last_ros_rejection_;
  std::optional<Manager::State> last_published_state_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  TransformPoseFunction transform_pose_;

  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Service<ChangeControlState>::SharedPtr change_control_state_service_;
  rclcpp::Subscription<mg400_msgs::msg::ServoJ>::SharedPtr servo_j_subscriber_;
  rclcpp::Subscription<mg400_msgs::msg::ServoP>::SharedPtr servo_p_subscriber_;
  rclcpp::Publisher<mg400_msgs::msg::ControlState>::SharedPtr control_state_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  void publishControlStateLocked(bool force);
  void rejectRosTarget(const std::string & reason);
  bool defaultTransformPose(
    const geometry_msgs::msg::PoseStamped & input,
    const std::string & target_frame,
    geometry_msgs::msg::PoseStamped & output,
    std::string & reason);
  static bool finitePose(
    const geometry_msgs::msg::Pose & pose,
    std::string & reason);
  static const char * sessionStateName(Session::State state) noexcept;
  static const char * stopCauseName(Session::StopCause cause) noexcept;
  static const char * responseResultName(
    mg400_interface::MotionResponseResult result) noexcept;
};

}  // namespace mg400_node

#endif  // MG400_NODE__SERVO_CONTROL_ROS_INTERFACE_HPP_
