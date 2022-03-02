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


#include "mg400_controller/mg400_interface/component.hpp"

namespace mg400_interface
{

Component::Component(
  const rclcpp::NodeOptions & options
)
: Node("mg400_interface", "mg400_interface", options)
{
  this->enable_robot_srv = this->create_service<EnableRobot>(
    "enable_robot",
    std::bind(
      &Component::enableRobot, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->disable_robot_srv_ = this->create_service<DisableRobot>(
    "disable_robot",
    std::bind(
      &Component::disableRobot, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->clear_error_srv_ = this->create_service<ClearError>(
    "clear_error",
    std::bind(
      &Component::clearError, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->reset_robot_srv_ = this->create_service<ResetRobot>(
    "reset_robot",
    std::bind(
      &Component::resetRobot, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->speed_factor_srv_ = this->create_service<SpeedFactor>(
    "speed_factor",
    std::bind(
      &Component::speedFactor, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->user_srv_ = this->create_service<User>(
    "user",
    std::bind(
      &Component::user, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->tool_srv_ = this->create_service<Tool>(
    "tool",
    std::bind(
      &Component::tool, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->robot_mode_srv_ = this->create_service<RobotMode>(
    "robot_mode",
    std::bind(
      &Component::robotMode, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->payload_srv_ = this->create_service<Payload>(
    "payload",
    std::bind(
      &Component::payload, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->do_srv_ = this->create_service<DO>(
    "DO",
    std::bind(
      &Component::dO, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->do_execute_srv_ = this->create_service<DOExecute>(
    "DO_execute",
    std::bind(
      &Component::dOExecute, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->tool_do_srv_ = this->create_service<ToolDO>(
    "tool_DO",
    std::bind(
      &Component::toolDO, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->tool_do_execute_srv_ = this->create_service<ToolDOExecute>(
    "tool_DO_execute",
    std::bind(
      &Component::toolDOExecute, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->ao_srv_ = this->create_service<AO>(
    "AO",
    std::bind(
      &Component::aO, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->ao_execute_srv_ = this->create_service<AOExecute>(
    "AO_execute",
    std::bind(
      &Component::aOExecute, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->acc_j_srv_ = this->create_service<AccJ>(
    "acc_j",
    std::bind(
      &Component::accJ, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->acc_l_srv_ = this->create_service<AccL>(
    "acc_l",
    std::bind(
      &Component::accL, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->speed_j_srv_ = this->create_service<SpeedJ>(
    "speed_j",
    std::bind(
      &Component::speedJ, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->speed_l_srv_ = this->create_service<SpeedL>(
    "speed_l",
    std::bind(
      &Component::speedL, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->arch_srv_ = this->create_service<Arch>(
    "arch",
    std::bind(
      &Component::arch, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->cp_srv_ = this->create_service<CP>(
    "cp",
    std::bind(
      &Component::cp, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->lim_z_srv_ = this->create_service<LimZ>(
    "lim_z",
    std::bind(
      &Component::limZ, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->set_arm_orientation_srv_ = this->create_service<SetArmOrientation>(
    "set_arm_orientation",
    std::bind(
      &Component::setArmOrientation, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->power_on_srv_ = this->create_service<PowerOn>(
    "power_on",
    std::bind(
      &Component::powerOn, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->run_script_srv_ = this->create_service<RunScript>(
    "run_script",
    std::bind(
      &Component::runScript, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->stop_script_srv_ = this->create_service<StopScript>(
    "stop_script",
    std::bind(
      &Component::stopScript, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->pause_script_srv_ = this->create_service<PauseScript>(
    "pause_script",
    std::bind(
      &Component::pauseScript, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->continue_script_srv_ = this->create_service<ContinueScript>(
    "continue_script",
    std::bind(
      &Component::continueScript, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->set_safe_skin_srv_ = this->create_service<SetSafeSkin>(
    "set_safe_skin",
    std::bind(
      &Component::setSafeSkin, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->set_obstacle_avoid_srv_ = this->create_service<SetObstacleAvoid>(
    "set_obstacle_avoid",
    std::bind(
      &Component::setObstacleAvoid, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->set_collistion_level_srv_ = this->create_service<SetCollisionLevel>(
    "set_collision_level",
    std::bind(
      &Component::setCollisionLevel, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->emergency_stop_srv_ = this->create_service<EmergencyStop>(
    "emergency_stop",
    std::bind(
      &Component::emergencyStop, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );

  this->mov_j_srv_ = this->create_service<MovJ>(
    "mov_j",
    std::bind(
      &Component::movJ, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->mov_l_srv_ = this->create_service<MovL>(
    "mov_l",
    std::bind(
      &Component::movL, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->jump_srv_ = this->create_service<Jump>(
    "jump",
    std::bind(
      &Component::jump, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->arc_srv_ = this->create_service<Arc>(
    "arc",
    std::bind(
      &Component::arc, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->sync_srv_ = this->create_service<Sync>(
    "sync",
    std::bind(
      &Component::sync, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->circle_srv_ = this->create_service<Circle>(
    "circle",
    std::bind(
      &Component::circle, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->servo_j_srv_ = this->create_service<ServoJ>(
    "servo_j",
    std::bind(
      &Component::servoJ, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->start_trace_srv_ = this->create_service<StartTrace>(
    "start_trace",
    std::bind(
      &Component::startTrace, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->start_path_srv_ = this->create_service<StartPath>(
    "start_path",
    std::bind(
      &Component::startPath, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->move_jog_srv_ = this->create_service<MoveJog>(
    "move_jog",
    std::bind(
      &Component::moveJog, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->servo_p_srv_ = this->create_service<ServoP>(
    "servo_p",
    std::bind(
      &Component::servoP, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->rel_mov_j_srv_ = this->create_service<RelMovJ>(
    "rel_mov_j",
    std::bind(
      &Component::relMovJ, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->rel_mov_l_srv_ = this->create_service<RelMovL>(
    "rel_mov_l",
    std::bind(
      &Component::relMovL, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );
  this->joint_mov_j_srv_ = this->create_service<JointMovJ>(
    "joint_mov_j",
    std::bind(
      &Component::jointMovJ, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    )
  );

  RCLCPP_INFO(
    this->get_logger(),
    "Ready"
  );
}

Component::~Component()
{

}

void Component::getJonitState(double * js)
{
  this->commander_->getCurrentJointStates(js);
}

bool Component::isEnabled() const
{
  return this->commander_->isEnabled();
}

bool Component::isConnected() const
{
  return this->commander_->isConnected();
}

void Component::getToolVectorActual(double * val)
{
  this->commander_->getToolVectorActual(val);
}

// dashboard
void Component::enableRobot(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const EnableRobot::Request::SharedPtr request,
  EnableRobot::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  (void)request; // for linter
  try {
    this->commander_->enableRobot();
    response->res = 0;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    this->commander_->clearError();
    response->res = -1;
  }
}

void Component::disableRobot(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const DisableRobot::Request::SharedPtr request,
  DisableRobot::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  (void)request; // for linter
  try {
    this->commander_->disableRobot();
    response->res = 0;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    this->commander_->clearError();
    response->res = -1;
  }
}

void Component::clearError(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const ClearError::Request::SharedPtr request,
  ClearError::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  (void)request; // for linter
  try {
    this->commander_->clearError();
    response->res = 0;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    this->commander_->clearError();
    response->res = -1;
  }
}

void Component::resetRobot(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const ResetRobot::Request::SharedPtr request,
  ResetRobot::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  (void)request; // for linter
  try {
    this->commander_->resetRobot();
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::speedFactor(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const SpeedFactor::Request::SharedPtr request,
  SpeedFactor::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    this->commander_->speedFactor(request->ratio);
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::user(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const User::Request::SharedPtr request,
  User::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "User(%d)", request->index);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::tool(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const Tool::Request::SharedPtr request,
  Tool::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "Tool(%d)", request->index);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::robotMode(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const RobotMode::Request::SharedPtr request,
  RobotMode::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  (void)request; // for linter
  try {
    const char * cmd = "RobotMode()";
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::payload(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const Payload::Request::SharedPtr request,
  Payload::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "PayLoad(%0.3f, %0.3f)", request->weight, request->inertia);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::dO(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const DO::Request::SharedPtr request,
  DO::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "DO(%d, %d)", request->index, request->status);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::dOExecute(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const DOExecute::Request::SharedPtr request,
  DOExecute::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "DOExecute(%d, %d)", request->index, request->status);
    commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::toolDO(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const ToolDO::Request::SharedPtr request,
  ToolDO::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "ToolDO(%d, %d)", request->index, request->status);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::toolDOExecute(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const ToolDOExecute::Request::SharedPtr request,
  ToolDOExecute::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "ToolDOExecute(%d, %d)", request->index, request->status);
    commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::aO(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const AO::Request::SharedPtr request,
  AO::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "AO(%d, %d)", request->index, request->status);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::aOExecute(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const AOExecute::Request::SharedPtr request,
  AOExecute::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "AOExecute(%d, %0.3f)", request->index, static_cast<float>(request->value));
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::accJ(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const AccJ::Request::SharedPtr request,
  AccJ::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "AccJ(%d)", request->r);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::accL(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const AccL::Request::SharedPtr request,
  AccL::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "AccL(%d)", request->r);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::speedJ(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const SpeedJ::Request::SharedPtr request,
  SpeedJ::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "SpeedJ(%d)", request->r);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::speedL(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const SpeedL::Request::SharedPtr request,
  SpeedL::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "SpeedL(%d)", request->r);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::arch(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const Arch::Request::SharedPtr request,
  Arch::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "Arch(%d)", request->index);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::cp(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const CP::Request::SharedPtr request,
  CP::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "CP(%d)", request->r);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::limZ(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const LimZ::Request::SharedPtr request,
  LimZ::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "LimZ(%d)", request->value);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::setArmOrientation(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const SetArmOrientation::Request::SharedPtr request,
  SetArmOrientation::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(
      cmd,
      "SetArmOrientation(%d,%d,%d,%d)",
      request->lor_r,
      request->uor_d,
      request->for_n,
      request->config_6
    );
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::powerOn(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const PowerOn::Request::SharedPtr request,
  PowerOn::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  (void)request; // for linter
  try {
    const char * cmd = "PowerOn()";
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::runScript(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const RunScript::Request::SharedPtr request,
  RunScript::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "RunScript(%s)", request->project_name.c_str());
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::stopScript(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const StopScript::Request::SharedPtr request,
  StopScript::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  (void)request; // for linter
  try {
    const char * cmd = "StopScript()";
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::pauseScript(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const PauseScript::Request::SharedPtr request,
  PauseScript::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  (void)request; // for linter
  try {
    const char * cmd = "PauseScript()";
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::continueScript(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const ContinueScript::Request::SharedPtr request,
  ContinueScript::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  (void)request; // for linter
  try {
    const char * cmd = "ContinueScript()";
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::setSafeSkin(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const SetSafeSkin::Request::SharedPtr request,
  SetSafeSkin::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "SetSafeSkin(%d)", request->status);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::setObstacleAvoid(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const SetObstacleAvoid::Request::SharedPtr request,
  SetObstacleAvoid::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "SetObstacleAvoid(%d)", request->status);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::setCollisionLevel(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const SetCollisionLevel::Request::SharedPtr request,
  SetCollisionLevel::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "SetCollisionLevel(%d)", request->level);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::emergencyStop(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const EmergencyStop::Request::SharedPtr request,
  EmergencyStop::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  (void)request; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "EmergencyStop()");
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

// real time

void Component::movJ(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const MovJ::Request::SharedPtr request,
  MovJ::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    this->commander_->movJ(
      request->x, request->y, request->z,
      request->a, request->b, request->c
    );
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::movL(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const MovL::Request::SharedPtr request,
  MovL::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    this->commander_->movL(
      request->x, request->y, request->z,
      request->a, request->b, request->c
    );
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::servoJ(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const ServoJ::Request::SharedPtr request,
  ServoJ::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    this->commander_->servoJ(
      request->j1, request->j2, request->j3,
      request->j4, request->j5, request->j6
    );
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::arc(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const Arc::Request::SharedPtr request,
  Arc::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(
      cmd,
      "Arc(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)",
      request->x1, request->y1, request->z1,
      request->rx1, request->ry1, request->rz1,
      request->x2, request->y2, request->z2,
      request->rx2, request->ry2, request->rz2
    );
    this->commander_->realSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::circle(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const Circle::Request::SharedPtr request,
  Circle::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(
      cmd, "Circle(%d, %0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)",
      request->count,
      request->x1, request->y1, request->z1,
      request->rx1, request->ry1, request->rz1,
      request->x2, request->y2, request->z2,
      request->rx2, request->ry2, request->rz2
    );
    this->commander_->realSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::jump(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const Jump::Request::SharedPtr request,
  Jump::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(
      cmd,
      "Jump(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)",
      request->offset1, request->offset2, request->offset3,
      request->offset4, request->offset5, request->offset6
    );
    this->commander_->realSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::servoP(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const ServoP::Request::SharedPtr request,
  ServoP::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    this->commander_->servoP(
      request->x, request->y, request->z,
      request->a, request->b, request->c
    );
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::relMovJ(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const RelMovJ::Request::SharedPtr request,
  RelMovJ::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    this->commander_->relMovJ(
      request->offset1, request->offset2, request->offset3,
      request->offset4, request->offset5, request->offset6
    );
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::relMovL(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const RelMovL::Request::SharedPtr request,
  RelMovL::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    this->commander_->relMovL(
      request->x, request->y, request->z
    );
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::jointMovJ(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const JointMovJ::Request::SharedPtr request,
  JointMovJ::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    this->commander_->jointMovJ(
      request->j1, request->j2, request->j3,
      request->j4, request->j5, request->j6
    );
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::sync(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const Sync::Request::SharedPtr request,
  Sync::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  (void)request; // for linter
  try {
    const char * cmd = "Sync()";
    this->commander_->realSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::startTrace(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const StartTrace::Request::SharedPtr request,
  StartTrace::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "StartTrace(%s)", request->trace_name.c_str());
    this->commander_->realSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::startPath(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const StartPath::Request::SharedPtr request,
  StartPath::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(
      cmd, "StartPath(%s,%d,%d)",
      request->trace_name.c_str(),
      request->const_val,
      request->cart
    );
    this->commander_->realSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::startFCTrace(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const StartFCTrace::Request::SharedPtr request,
  StartFCTrace::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "StartFCTrace(%s)", request->trace_name.c_str());
    this->commander_->realSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}

void Component::moveJog(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const MoveJog::Request::SharedPtr request,
  MoveJog::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    this->commander_->moveJog(request->axis_id);
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what()
    );
    response->res = -1;
  }
}
} // namespace mg400_interface
