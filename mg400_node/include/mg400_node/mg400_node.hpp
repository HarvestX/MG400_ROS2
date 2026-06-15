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

#ifndef __MG400_NODE_MG400_NODE_HPP__
#define __MG400_NODE_MG400_NODE_HPP__

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <mg400_msgs/msg/error_id.hpp>
#include <mg400_msgs/msg/realtime_feedback.hpp>
#include <mg400_msgs/msg/robot_mode.hpp>
#include <mg400_msgs/msg/servo_j.hpp>
#include <mg400_msgs/msg/servo_p.hpp>
#include <mg400_msgs/srv/servo_mode.hpp>
#include <std_msgs/msg/bool.hpp>
#include <mg400_plugin_base/api_loader_base.hpp>
#include <mg400_plugin_base/api_plugin_base.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mg400_node
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using State = rclcpp_lifecycle::State;

class MG400Node : public rclcpp_lifecycle::LifecycleNode
{
private:
  enum class ServoCommandType
  {
    NONE,
    SERVO_J,
    SERVO_P
  };

  const std::vector<std::string> default_dashboard_api_plugins_ = {
    "mg400_plugin::ClearError",
    "mg400_plugin::DisableRobot",
    "mg400_plugin::EmergencyStop",
    "mg400_plugin::EnableRobot",
    "mg400_plugin::GetErrorID",
    "mg400_plugin::InverseSolution",
    "mg400_plugin::PayLoad",
    "mg400_plugin::PositiveSolution",
    "mg400_plugin::ResetRobot",
    "mg400_plugin::SetCollisionLevel",
    "mg400_plugin::SpeedFactor",
    "mg400_plugin::ToolDI",
    "mg400_plugin::ToolDOExecute",
  };

  const std::vector<std::string> default_motion_api_plugins_ = {
    "mg400_plugin::CommandQueue",
    "mg400_plugin::JointMovJ",
    "mg400_plugin::MoveJog",
    "mg400_plugin::MovJ",
    "mg400_plugin::MovJIO",
    "mg400_plugin::MovL",
    "mg400_plugin::MovLIO",
  };

  std::string ip_address_;
  mg400_interface::MG400Interface::SharedPtr interface_;
  mg400_plugin_base::DashboardApiLoader::SharedPtr dashboard_api_loader_;
  mg400_plugin_base::MotionApiLoader::SharedPtr motion_api_loader_;

  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  rclcpp::TimerBase::SharedPtr realtime_feedback_timer_;
  rclcpp::TimerBase::SharedPtr robot_mode_timer_;
  rclcpp::TimerBase::SharedPtr error_timer_;
  rclcpp::TimerBase::SharedPtr interface_check_timer_;
  rclcpp::TimerBase::SharedPtr connect_timer_;
  rclcpp::TimerBase::SharedPtr servo_command_timeout_timer_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<mg400_msgs::msg::RealtimeFeedback>::SharedPtr realtime_feedback_pub_;
  rclcpp::Publisher<mg400_msgs::msg::RobotMode>::SharedPtr robot_mode_pub_;
  rclcpp::Publisher<mg400_msgs::msg::ErrorID>::SharedPtr error_id_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mg400_connected_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr servo_mode_enabled_pub_;

  rclcpp::Subscription<mg400_msgs::msg::ServoJ>::SharedPtr servo_j_sub_;
  rclcpp::Subscription<mg400_msgs::msg::ServoP>::SharedPtr servo_p_sub_;
  rclcpp::Service<mg400_msgs::srv::ServoMode>::SharedPtr servo_mode_srv_;

  bool connection_interrupted_;
  int servo_command_timeout_ms_;

  std::mutex servo_mode_mutex_;
  bool servo_mode_active_;
  ServoCommandType servo_command_type_;
  rclcpp::Time last_valid_servo_command_stamp_;
  bool has_realtime_feedback_;
  uint64_t latest_robot_mode_;
  uint8_t latest_enable_status_;
  uint8_t latest_error_status_;

  rclcpp::TimerBase::SharedPtr autoconfigure_timer_;
  std::atomic<bool> autoconfigure_executed_{false};

public:
  MG400Node() = delete;
  explicit MG400Node(const rclcpp::NodeOptions &);
  ~MG400Node() override;

  void onInit();
  void onJointStateTimer();
  void onRealtimeFeedbackTimer();
  void onRobotModeTimer();
  void onErrorTimer();
  void onInterfaceCheckTimer();
  void onServoCommandTimeoutTimer();

private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & /*previous_state*/) override;

  void runTimer();
  void cancelTimer();

  void handleAutoConfigure();
  void publishRealtimeFeedback(const mg400_interface::RealTimeData &);
  void updateServoModeRealtimeFeedback(const mg400_interface::RealTimeData &);
  void onServoModeService(
    const std::shared_ptr<mg400_msgs::srv::ServoMode::Request>,
    std::shared_ptr<mg400_msgs::srv::ServoMode::Response>);
  void onServoJ(const mg400_msgs::msg::ServoJ::SharedPtr);
  void onServoP(const mg400_msgs::msg::ServoP::SharedPtr);
  bool enterServoMode(std::string &, int32_t &);
  void exitServoMode(const std::string &, bool = false);
  bool checkDashboardErrors(std::string &);
  void publishServoModeEnabled(const bool);
  void warnInvalidServoCommand(const std::string &, const std::string &);
  void warnUnusedServoOptions(const std::string &, double, double, double);
  std::string getServoOriginFrame() const;
};
}  // namespace mg400_node


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mg400_node::MG400Node)
#endif
