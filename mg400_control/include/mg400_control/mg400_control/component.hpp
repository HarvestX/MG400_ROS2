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

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <mg400_msgs/srv/enable_robot.hpp>
#include <mg400_msgs/srv/disable_robot.hpp>
#include <mg400_msgs/srv/clear_error.hpp>
#include <mg400_msgs/srv/reset_robot.hpp>
#include <mg400_msgs/srv/speed_factor.hpp>
#include <mg400_msgs/srv/user.hpp>
#include <mg400_msgs/srv/tool.hpp>
#include <mg400_msgs/srv/robot_mode.hpp>
#include <mg400_msgs/srv/payload.hpp>
#include <mg400_msgs/srv/do.hpp>
#include <mg400_msgs/srv/do_execute.hpp>
#include <mg400_msgs/srv/tool_do.hpp>
#include <mg400_msgs/srv/tool_do_execute.hpp>
#include <mg400_msgs/srv/ao.hpp>
#include <mg400_msgs/srv/ao_execute.hpp>
#include <mg400_msgs/srv/acc_j.hpp>
#include <mg400_msgs/srv/acc_l.hpp>
#include <mg400_msgs/srv/speed_j.hpp>
#include <mg400_msgs/srv/speed_l.hpp>
#include <mg400_msgs/srv/arch.hpp>
#include <mg400_msgs/srv/cp.hpp>
#include <mg400_msgs/srv/lim_z.hpp>
#include <mg400_msgs/srv/set_arm_orientation.hpp>
#include <mg400_msgs/srv/power_on.hpp>
#include <mg400_msgs/srv/run_script.hpp>
#include <mg400_msgs/srv/stop_script.hpp>
#include <mg400_msgs/srv/pause_script.hpp>
#include <mg400_msgs/srv/continue_script.hpp>
#include <mg400_msgs/srv/set_safe_skin.hpp>
#include <mg400_msgs/srv/set_obstacle_avoid.hpp>
#include <mg400_msgs/srv/set_collision_level.hpp>
#include <mg400_msgs/srv/emergency_stop.hpp>

#include <mg400_msgs/srv/mov_j.hpp>
#include <mg400_msgs/srv/mov_l.hpp>
#include <mg400_msgs/srv/jump.hpp>
#include <mg400_msgs/srv/arc.hpp>
#include <mg400_msgs/srv/sync.hpp>
#include <mg400_msgs/srv/circle.hpp>
#include <mg400_msgs/srv/servo_j.hpp>
#include <mg400_msgs/srv/start_trace.hpp>
#include <mg400_msgs/srv/start_path.hpp>
#include <mg400_msgs/srv/start_fc_trace.hpp>
#include <mg400_msgs/srv/move_jog.hpp>
#include <mg400_msgs/srv/servo_p.hpp>
#include <mg400_msgs/srv/rel_mov_j.hpp>
#include <mg400_msgs/srv/rel_mov_l.hpp>
#include <mg400_msgs/srv/joint_mov_j.hpp>

#include <mg400_msgs/msg/robot_status.hpp>

#include "mg400_control/mg400_interface/commander.hpp"


namespace mg400_control
{
class Component : public rclcpp::Node
{
private:
  double goal_[6];
  const std::string prefix_;

  std::unique_ptr<mg400_interface::Commander> commander_;

  rclcpp::TimerBase::SharedPtr js_timer_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  rclcpp::Service<mg400_msgs::srv::EnableRobot>::SharedPtr enable_robot_srv_;
  rclcpp::Service<mg400_msgs::srv::DisableRobot>::SharedPtr disable_robot_srv_;
  rclcpp::Service<mg400_msgs::srv::ClearError>::SharedPtr clear_error_srv_;
  rclcpp::Service<mg400_msgs::srv::ResetRobot>::SharedPtr reset_robot_srv_;
  rclcpp::Service<mg400_msgs::srv::SpeedFactor>::SharedPtr speed_factor_srv_;
  rclcpp::Service<mg400_msgs::srv::User>::SharedPtr user_srv_;
  rclcpp::Service<mg400_msgs::srv::Tool>::SharedPtr tool_srv_;
  rclcpp::Service<mg400_msgs::srv::RobotMode>::SharedPtr robot_mode_srv_;
  rclcpp::Service<mg400_msgs::srv::Payload>::SharedPtr payload_srv_;
  rclcpp::Service<mg400_msgs::srv::DO>::SharedPtr do_srv_;
  rclcpp::Service<mg400_msgs::srv::DOExecute>::SharedPtr do_execute_srv_;
  rclcpp::Service<mg400_msgs::srv::ToolDO>::SharedPtr tool_do_srv_;
  rclcpp::Service<mg400_msgs::srv::ToolDOExecute>::SharedPtr
    tool_do_execute_srv_;
  rclcpp::Service<mg400_msgs::srv::AO>::SharedPtr ao_srv_;
  rclcpp::Service<mg400_msgs::srv::AOExecute>::SharedPtr ao_execute_srv_;
  rclcpp::Service<mg400_msgs::srv::AccJ>::SharedPtr acc_j_srv_;
  rclcpp::Service<mg400_msgs::srv::AccL>::SharedPtr acc_l_srv_;
  rclcpp::Service<mg400_msgs::srv::SpeedJ>::SharedPtr speed_j_srv_;
  rclcpp::Service<mg400_msgs::srv::SpeedL>::SharedPtr speed_l_srv_;
  rclcpp::Service<mg400_msgs::srv::Arch>::SharedPtr arch_srv_;
  rclcpp::Service<mg400_msgs::srv::CP>::SharedPtr cp_srv_;
  rclcpp::Service<mg400_msgs::srv::LimZ>::SharedPtr lim_z_srv_;
  rclcpp::Service<mg400_msgs::srv::SetArmOrientation>::SharedPtr
    set_arm_orientation_srv_;
  rclcpp::Service<mg400_msgs::srv::PowerOn>::SharedPtr power_on_srv_;
  rclcpp::Service<mg400_msgs::srv::RunScript>::SharedPtr run_script_srv_;
  rclcpp::Service<mg400_msgs::srv::StopScript>::SharedPtr stop_script_srv_;
  rclcpp::Service<mg400_msgs::srv::PauseScript>::SharedPtr pause_script_srv_;
  rclcpp::Service<mg400_msgs::srv::ContinueScript>::SharedPtr
    continue_script_srv_;
  rclcpp::Service<mg400_msgs::srv::SetSafeSkin>::SharedPtr set_safe_skin_srv_;
  rclcpp::Service<mg400_msgs::srv::SetObstacleAvoid>::SharedPtr
    set_obstacle_avoid_srv_;
  rclcpp::Service<mg400_msgs::srv::SetCollisionLevel>::SharedPtr
    set_collistion_level_srv_;
  rclcpp::Service<mg400_msgs::srv::EmergencyStop>::SharedPtr
    emergency_stop_srv_;

  rclcpp::Service<mg400_msgs::srv::MovJ>::SharedPtr mov_j_srv_;
  rclcpp::Service<mg400_msgs::srv::MovL>::SharedPtr mov_l_srv_;
  rclcpp::Service<mg400_msgs::srv::Jump>::SharedPtr jump_srv_;
  rclcpp::Service<mg400_msgs::srv::Arc>::SharedPtr arc_srv_;
  rclcpp::Service<mg400_msgs::srv::Sync>::SharedPtr sync_srv_;
  rclcpp::Service<mg400_msgs::srv::Circle>::SharedPtr circle_srv_;
  rclcpp::Service<mg400_msgs::srv::ServoJ>::SharedPtr servo_j_srv_;
  rclcpp::Service<mg400_msgs::srv::StartTrace>::SharedPtr start_trace_srv_;
  rclcpp::Service<mg400_msgs::srv::StartPath>::SharedPtr start_path_srv_;
  rclcpp::Service<mg400_msgs::srv::MoveJog>::SharedPtr move_jog_srv_;
  rclcpp::Service<mg400_msgs::srv::ServoP>::SharedPtr servo_p_srv_;
  rclcpp::Service<mg400_msgs::srv::RelMovJ>::SharedPtr rel_mov_j_srv_;
  rclcpp::Service<mg400_msgs::srv::RelMovL>::SharedPtr rel_mov_l_srv_;
  rclcpp::Service<mg400_msgs::srv::JointMovJ>::SharedPtr joint_mov_j_srv_;

public:
  explicit Component(
    const rclcpp::NodeOptions &);
  ~Component();

  void init();

  void getJointState(double *);

  void getToolVectorActual(double *);

  bool isEnabled() const;

  bool isConnected() const;

protected:
  void publishJointState();

  void enableRobot(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::EnableRobot::Request::SharedPtr,
    mg400_msgs::srv::EnableRobot::Response::SharedPtr);
  void disableRobot(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::DisableRobot::Request::SharedPtr,
    mg400_msgs::srv::DisableRobot::Response::SharedPtr);
  void clearError(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::ClearError::Request::SharedPtr,
    mg400_msgs::srv::ClearError::Response::SharedPtr);
  void resetRobot(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::ResetRobot::Request::SharedPtr,
    mg400_msgs::srv::ResetRobot::Response::SharedPtr);
  void speedFactor(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::SpeedFactor::Request::SharedPtr,
    mg400_msgs::srv::SpeedFactor::Response::SharedPtr);
  void user(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::User::Request::SharedPtr,
    mg400_msgs::srv::User::Response::SharedPtr);
  void tool(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::Tool::Request::SharedPtr,
    mg400_msgs::srv::Tool::Response::SharedPtr);
  void robotMode(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::RobotMode::Request::SharedPtr,
    mg400_msgs::srv::RobotMode::Response::SharedPtr);
  void payload(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::Payload::Request::SharedPtr,
    mg400_msgs::srv::Payload::Response::SharedPtr);
  void dO(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::DO::Request::SharedPtr,
    mg400_msgs::srv::DO::Response::SharedPtr);
  void dOExecute(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::DOExecute::Request::SharedPtr,
    mg400_msgs::srv::DOExecute::Response::SharedPtr);
  void toolDO(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::ToolDO::Request::SharedPtr,
    mg400_msgs::srv::ToolDO::Response::SharedPtr);
  void toolDOExecute(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::ToolDOExecute::Request::SharedPtr,
    mg400_msgs::srv::ToolDOExecute::Response::SharedPtr);
  void aO(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::AO::Request::SharedPtr,
    mg400_msgs::srv::AO::Response::SharedPtr);
  void aOExecute(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::AOExecute::Request::SharedPtr,
    mg400_msgs::srv::AOExecute::Response::SharedPtr);
  void accJ(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::AccJ::Request::SharedPtr,
    mg400_msgs::srv::AccJ::Response::SharedPtr);
  void accL(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::AccL::Request::SharedPtr,
    mg400_msgs::srv::AccL::Response::SharedPtr);
  void speedJ(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::SpeedJ::Request::SharedPtr,
    mg400_msgs::srv::SpeedJ::Response::SharedPtr);
  void speedL(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::SpeedL::Request::SharedPtr,
    mg400_msgs::srv::SpeedL::Response::SharedPtr);
  void arch(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::Arch::Request::SharedPtr,
    mg400_msgs::srv::Arch::Response::SharedPtr);
  void cp(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::CP::Request::SharedPtr,
    mg400_msgs::srv::CP::Response::SharedPtr);
  void limZ(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::LimZ::Request::SharedPtr,
    mg400_msgs::srv::LimZ::Response::SharedPtr);
  void setArmOrientation(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::SetArmOrientation::Request::SharedPtr,
    mg400_msgs::srv::SetArmOrientation::Response::SharedPtr);
  void powerOn(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::PowerOn::Request::SharedPtr,
    mg400_msgs::srv::PowerOn::Response::SharedPtr);
  void runScript(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::RunScript::Request::SharedPtr,
    mg400_msgs::srv::RunScript::Response::SharedPtr);
  void stopScript(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::StopScript::Request::SharedPtr,
    mg400_msgs::srv::StopScript::Response::SharedPtr);
  void pauseScript(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::PauseScript::Request::SharedPtr,
    mg400_msgs::srv::PauseScript::Response::SharedPtr);
  void continueScript(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::ContinueScript::Request::SharedPtr,
    mg400_msgs::srv::ContinueScript::Response::SharedPtr);
  void setSafeSkin(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::SetSafeSkin::Request::SharedPtr,
    mg400_msgs::srv::SetSafeSkin::Response::SharedPtr);
  void setObstacleAvoid(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::SetObstacleAvoid::Request::SharedPtr,
    mg400_msgs::srv::SetObstacleAvoid::Response::SharedPtr);
  void setCollisionLevel(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::SetCollisionLevel::Request::SharedPtr,
    mg400_msgs::srv::SetCollisionLevel::Response::SharedPtr);
  void emergencyStop(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::EmergencyStop::Request::SharedPtr,
    mg400_msgs::srv::EmergencyStop::Response::SharedPtr);

  void movJ(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::MovJ::Request::SharedPtr,
    mg400_msgs::srv::MovJ::Response::SharedPtr);
  void movL(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::MovL::Request::SharedPtr,
    mg400_msgs::srv::MovL::Response::SharedPtr);
  void jointMovJ(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::JointMovJ::Request::SharedPtr,
    mg400_msgs::srv::JointMovJ::Response::SharedPtr);
  void jump(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::Jump::Request::SharedPtr,
    mg400_msgs::srv::Jump::Response::SharedPtr);
  void relMovJ(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::RelMovJ::Request::SharedPtr,
    mg400_msgs::srv::RelMovJ::Response::SharedPtr);
  void relMovL(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::RelMovL::Request::SharedPtr,
    mg400_msgs::srv::RelMovL::Response::SharedPtr);
  void arc(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::Arc::Request::SharedPtr,
    mg400_msgs::srv::Arc::Response::SharedPtr);
  void circle(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::Circle::Request::SharedPtr,
    mg400_msgs::srv::Circle::Response::SharedPtr);
  void servoJ(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::ServoJ::Request::SharedPtr,
    mg400_msgs::srv::ServoJ::Response::SharedPtr);
  void servoP(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::ServoP::Request::SharedPtr,
    mg400_msgs::srv::ServoP::Response::SharedPtr);
  void sync(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::Sync::Request::SharedPtr,
    mg400_msgs::srv::Sync::Response::SharedPtr);
  void startTrace(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::StartTrace::Request::SharedPtr,
    mg400_msgs::srv::StartTrace::Response::SharedPtr);
  void startPath(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::StartPath::Request::SharedPtr,
    mg400_msgs::srv::StartPath::Response::SharedPtr);
  void startFCTrace(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::StartFCTrace::Request::SharedPtr,
    mg400_msgs::srv::StartFCTrace::Response::SharedPtr);
  void moveJog(
    const std::shared_ptr<rmw_request_id_t>,
    const mg400_msgs::srv::MoveJog::Request::SharedPtr,
    mg400_msgs::srv::MoveJog::Response::SharedPtr);
};
}  // namespace mg400_control
