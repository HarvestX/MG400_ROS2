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

#include "mg400_controller/mg400_interface/commander.hpp"


namespace mg400_interface
{
using namespace mg400_msgs::srv;
class MG400ServiceServer
{

private:
  double goal_[6];
  uint32_t index_;
  rclcpp::TimerBase::UniquePtr timer_;
  rclcpp::TimerBase::UniquePtr mov_j_timer_;
  double trajectory_duration_;
  std::unique_ptr<Commander> commander_;

public:
  MG400ServiceServer();
  ~MG400ServiceServer();

  void init();

  void getJonitState(double *);

  void getToolVectorActual(double *);

  bool isEnabled() const;

  bool isConnected() const;

protected:
  void enableRobot(
    const std::shared_ptr<rmw_request_id_t>,
    const EnableRobot::Request::SharedPtr,
    EnableRobot::Response::SharedPtr
  );
  void disableRobot(
    const std::shared_ptr<rmw_request_id_t>,
    const DisableRobot::Request::SharedPtr,
    DisableRobot::Response::SharedPtr
  );
  void clearError(
    const std::shared_ptr<rmw_request_id_t>,
    const ClearError::Request::SharedPtr,
    ClearError::Response::SharedPtr
  );
  void resetRobot(
    const std::shared_ptr<rmw_request_id_t>,
    const ResetRobot::Request::SharedPtr,
    ResetRobot::Response::SharedPtr
  );
  void speedFactor(
    const std::shared_ptr<rmw_request_id_t>,
    const SpeedFactor::Request::SharedPtr,
    SpeedFactor::Response::SharedPtr
  );
  void user(
    const std::shared_ptr<rmw_request_id_t>,
    const User::Request::SharedPtr,
    User::Response::SharedPtr
  );
  void tool(
    const std::shared_ptr<rmw_request_id_t>,
    const Tool::Request::SharedPtr,
    Tool::Response::SharedPtr
  );
  void robotMode(
    const std::shared_ptr<rmw_request_id_t>,
    const RobotMode::Request::SharedPtr,
    RobotMode::Response::SharedPtr
  );
  void payload(
    const std::shared_ptr<rmw_request_id_t>,
    const Payload::Request::SharedPtr,
    Payload::Response::SharedPtr
  );
  void DO(
    const std::shared_ptr<rmw_request_id_t>,
    const DO::Request::SharedPtr,
    DO::Response::SharedPtr
  );
  void DOExecute(
    const std::shared_ptr<rmw_request_id_t>,
    const DOExecute::Request::SharedPtr,
    DOExecute::Response::SharedPtr
  );
  void toolDO(
    const std::shared_ptr<rmw_request_id_t>,
    const ToolDO::Request::SharedPtr,
    ToolDO::Response::SharedPtr
  );
  void toolDOExecute(
    const std::shared_ptr<rmw_request_id_t>,
    const ToolDOExecute::Request::SharedPtr,
    ToolDOExecute::Response::SharedPtr
  );
  void AO(
    const std::shared_ptr<rmw_request_id_t>,
    const AO::Request::SharedPtr,
    AO::Response::SharedPtr
  );
  void AOExecute(
    const std::shared_ptr<rmw_request_id_t>,
    const AOExecute::Request::SharedPtr,
    AOExecute::Response::SharedPtr
  );
  void accJ(
    const std::shared_ptr<rmw_request_id_t>,
    const AccJ::Request::SharedPtr,
    AccJ::Response::SharedPtr
  );
  void accL(
    const std::shared_ptr<rmw_request_id_t>,
    const AccL::Request::SharedPtr,
    AccL::Response::SharedPtr
  );
  void speedJ(
    const std::shared_ptr<rmw_request_id_t>,
    const SpeedJ::Request::SharedPtr,
    SpeedJ::Response::SharedPtr
  );
  void speedL(
    const std::shared_ptr<rmw_request_id_t>,
    const SpeedL::Request::SharedPtr,
    SpeedL::Response::SharedPtr
  );
  void arch(
    const std::shared_ptr<rmw_request_id_t>,
    const Arch::Request::SharedPtr,
    Arch::Response::SharedPtr
  );
  void cp(
    const std::shared_ptr<rmw_request_id_t>,
    const CP::Request::SharedPtr,
    CP::Response::SharedPtr
  );
  void limZ(
    const std::shared_ptr<rmw_request_id_t>,
    const LimZ::Request::SharedPtr,
    LimZ::Response::SharedPtr
  );
  void setArmOrientation(
    const std::shared_ptr<rmw_request_id_t>,
    const SetArmOrientation::Request::SharedPtr,
    SetArmOrientation::Response::SharedPtr
  );
  void powerOn(
    const std::shared_ptr<rmw_request_id_t>,
    const PowerOn::Request::SharedPtr,
    PowerOn::Response::SharedPtr
  );
  void runScript(
    const std::shared_ptr<rmw_request_id_t>,
    const RunScript::Request::SharedPtr,
    RunScript::Response::SharedPtr
  );
  void stopScript(
    const std::shared_ptr<rmw_request_id_t>,
    const StopScript::Request::SharedPtr,
    StopScript::Response::SharedPtr
  );
  void pauseScript(
    const std::shared_ptr<rmw_request_id_t>,
    const PauseScript::Request::SharedPtr,
    PauseScript::Response::SharedPtr
  );
  void continueScript(
    const std::shared_ptr<rmw_request_id_t>,
    const ContinueScript::Request::SharedPtr,
    ContinueScript::Response::SharedPtr
  );
  void setSafeSkin(
    const std::shared_ptr<rmw_request_id_t>,
    const SetSafeSkin::Request::SharedPtr,
    SetSafeSkin::Response::SharedPtr
  );
  void setObstacleAvoid(
    const std::shared_ptr<rmw_request_id_t>,
    const SetObstacleAvoid::Request::SharedPtr,
    SetObstacleAvoid::Response::SharedPtr
  );
  void setCollisionLevel(
    const std::shared_ptr<rmw_request_id_t>,
    const SetCollisionLevel::Request::SharedPtr,
    SetCollisionLevel::Response::SharedPtr
  );
  void emergencyStop(
    const std::shared_ptr<rmw_request_id_t>,
    const EmergencyStop::Request::SharedPtr,
    EmergencyStop::Response::SharedPtr
  );

  void movJ(
    const std::shared_ptr<rmw_request_id_t>,
    const MovJ::Request::SharedPtr,
    MovJ::Response::SharedPtr
  );
  void movL(
    const std::shared_ptr<rmw_request_id_t>,
    const MovL::Request::SharedPtr,
    MovL::Response::SharedPtr
  );
  void jointMovJ(
    const std::shared_ptr<rmw_request_id_t>,
    const JointMovJ::Request::SharedPtr,
    JointMovJ::Response::SharedPtr
  );
  void jump(
    const std::shared_ptr<rmw_request_id_t>,
    const Jump::Request::SharedPtr,
    Jump::Response::SharedPtr
  );
  void relMovJ(
    const std::shared_ptr<rmw_request_id_t>,
    const RelMovJ::Request::SharedPtr,
    RelMovJ::Response::SharedPtr
  );
  void relMovL(
    const std::shared_ptr<rmw_request_id_t>,
    const RelMovL::Request::SharedPtr,
    RelMovL::Response::SharedPtr
  );
  void arc(
    const std::shared_ptr<rmw_request_id_t>,
    const Arc::Request::SharedPtr,
    Arc::Response::SharedPtr
  );
  void circle(
    const std::shared_ptr<rmw_request_id_t>,
    const Circle::Request::SharedPtr,
    Circle::Response::SharedPtr
  );
  void servoJ(
    const std::shared_ptr<rmw_request_id_t>,
    const ServoJ::Request::SharedPtr,
    ServoJ::Response::SharedPtr
  );
  void servoP(
    const std::shared_ptr<rmw_request_id_t>,
    const ServoP::Request::SharedPtr,
    ServoP::Response::SharedPtr
  );
  void sync(
    const std::shared_ptr<rmw_request_id_t>,
    const Sync::Request::SharedPtr,
    Sync::Response::SharedPtr
  );
  void startTrace(
    const std::shared_ptr<rmw_request_id_t>,
    const StartTrace::Request::SharedPtr,
    StartTrace::Response::SharedPtr
  );
  void startPath(
    const std::shared_ptr<rmw_request_id_t>,
    const StartPath::Request::SharedPtr,
    StartPath::Response::SharedPtr
  );
  void startFCTrace(
    const std::shared_ptr<rmw_request_id_t>,
    const StartFCTrace::Request::SharedPtr,
    StartFCTrace::Response::SharedPtr
  );
  void moveJog(
    const std::shared_ptr<rmw_request_id_t>,
    const MoveJog::Request::SharedPtr,
    MoveJog::Response::SharedPtr
  );
};
} // namespace mg400_interface
