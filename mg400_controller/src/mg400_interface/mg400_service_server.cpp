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


#include "mg400_controller/mg400_interface/mg400_service_server.hpp"

namespace mg400_interface
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("MG400ServiceServer");
void MG400ServiceServer::getJonitState(double * js)
{
  this->commander_->getCurrentJointStates(js);
}

bool MG400ServiceServer::isEnabled() const
{
  return this->commander_->isEnabled();
}

bool MG400ServiceServer::isConnected() const
{
  return this->commander_->isConnected();
}

void MG400ServiceServer::getToolVectorActual(double * val)
{
  this->commander_->getToolVectorActual(val);
}

// dashboard
void MG400ServiceServer::enableRobot(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const EnableRobot::Request::SharedPtr request,
  EnableRobot::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    this->commander_->enableRobot();
    response->res = 0;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      LOGGER,
      e.what()
    );
    this->commander_->clearError();
    response->res = -1;
  }
}

void MG400ServiceServer::disableRobot(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const DisableRobot::Request::SharedPtr request,
  DisableRobot::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    this->commander_->disableRobot();
    response->res = 0;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      LOGGER,
      e.what()
    );
    this->commander_->clearError();
    response->res = -1;
  }
}

void MG400ServiceServer::clearError(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const ClearError::Request::SharedPtr request,
  ClearError::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    this->commander_->clearError();
    response->res = 0;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      LOGGER,
      e.what()
    );
    this->commander_->clearError();
    response->res = -1;
  }
}

void MG400ServiceServer::resetRobot(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const ResetRobot::Request::SharedPtr request,
  ResetRobot::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    this->commander_->resetRobot();
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::speedFactor(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::user(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::tool(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::robotMode(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const RobotMode::Request::SharedPtr request,
  RobotMode::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    const char * cmd = "RobotMode()";
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::payload(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::DO(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::DOExecute(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::toolDO(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::toolDOExecute(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::AO(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::AOExecute(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::accJ(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::accL(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::speedJ(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::speedL(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::arch(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::cp(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::limZ(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::setArmOrientation(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::powerOn(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const PowerOn::Request::SharedPtr request,
  PowerOn::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    const char * cmd = "PowerOn()";
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::runScript(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::stopScript(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const StopScript::Request::SharedPtr request,
  StopScript::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    const char * cmd = "StopScript()";
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::pauseScript(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const PauseScript::Request::SharedPtr request,
  PauseScript::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    const char * cmd = "PauseScript()";
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::continueScript(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const ContinueScript::Request::SharedPtr request,
  ContinueScript::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    const char * cmd = "ContinueScript()";
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::setSafeSkin(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::setObstacleAvoid(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::setCollisionLevel(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::emergencyStop(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const EmergencyStop::Request::SharedPtr request,
  EmergencyStop::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    char cmd[100];
    sprintf(cmd, "EmergencyStop()");
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

// real time

void MG400ServiceServer::movJ(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::movL(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::servoJ(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::jump(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::arc(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::servoP(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::relMovJ(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::relMovL(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::jointMovJ(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::sync(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const Sync::Request::SharedPtr request,
  Sync::Response::SharedPtr response
)
{
  (void)request_header; // for linter
  try {
    const char * cmd = "Sync()";
    this->commander_->realSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const TcpClientException & e) {
    RCLCPP_ERROR(
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::startTrace(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::startPath(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::startFCTrace(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}

void MG400ServiceServer::moveJog(
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
      LOGGER,
      e.what()
    );
    response->res = -1;
  }
}
} // namespace mg400_interface
