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


#include "mg400_control/convert.hpp"
#include "mg400_control/mg400_control/component.hpp"

namespace mg400_control
{
using namespace std::chrono_literals;

Component::Component(
  const rclcpp::NodeOptions & options
)
: Node("mg400_control", options)
{
  std::string ip;
  this->declare_parameter<std::string>("ip_address", "192.168.1.6");
  this->get_parameter("ip_address", ip);
  RCLCPP_INFO(
    this->get_logger(),
    "IP address: %s", ip.c_str());

  commander_ = std::make_unique<mg400_interface::Commander>(ip);
  commander_->init();

  this->joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
  this->js_timer_ = this->create_wall_timer(
    100ms,
    std::bind(&Component::publishJointState, this));

  this->enable_robot_srv_ = this->create_service<mg400_msgs::srv::EnableRobot>(
    "enable_robot",
    std::bind(
      &Component::enableRobot, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->disable_robot_srv_ = this->create_service<mg400_msgs::srv::DisableRobot>(
    "disable_robot",
    std::bind(
      &Component::disableRobot, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->clear_error_srv_ = this->create_service<mg400_msgs::srv::ClearError>(
    "clear_error",
    std::bind(
      &Component::clearError, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->get_error_id_srv_ = this->create_service<mg400_msgs::srv::GetErrorID>(
    "get_error_id",
    std::bind(
      &Component::getErrorID, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->reset_robot_srv_ = this->create_service<mg400_msgs::srv::ResetRobot>(
    "reset_robot",
    std::bind(
      &Component::resetRobot, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->speed_factor_srv_ = this->create_service<mg400_msgs::srv::SpeedFactor>(
    "speed_factor",
    std::bind(
      &Component::speedFactor, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->user_srv_ = this->create_service<mg400_msgs::srv::User>(
    "user",
    std::bind(
      &Component::user, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->tool_srv_ = this->create_service<mg400_msgs::srv::Tool>(
    "tool",
    std::bind(
      &Component::tool, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->robot_mode_srv_ = this->create_service<mg400_msgs::srv::RobotMode>(
    "robot_mode",
    std::bind(
      &Component::robotMode, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->payload_srv_ = this->create_service<mg400_msgs::srv::Payload>(
    "payload",
    std::bind(
      &Component::payload, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->do_srv_ = this->create_service<mg400_msgs::srv::DO>(
    "DO",
    std::bind(
      &Component::dO, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->do_execute_srv_ = this->create_service<mg400_msgs::srv::DOExecute>(
    "DO_execute",
    std::bind(
      &Component::dOExecute, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->tool_do_srv_ = this->create_service<mg400_msgs::srv::ToolDO>(
    "tool_DO",
    std::bind(
      &Component::toolDO, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->tool_do_execute_srv_ =
    this->create_service<mg400_msgs::srv::ToolDOExecute>(
    "tool_DO_execute",
    std::bind(
      &Component::toolDOExecute, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->ao_srv_ = this->create_service<mg400_msgs::srv::AO>(
    "AO",
    std::bind(
      &Component::aO, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->ao_execute_srv_ = this->create_service<mg400_msgs::srv::AOExecute>(
    "AO_execute",
    std::bind(
      &Component::aOExecute, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->acc_j_srv_ = this->create_service<mg400_msgs::srv::AccJ>(
    "acc_j",
    std::bind(
      &Component::accJ, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->acc_l_srv_ = this->create_service<mg400_msgs::srv::AccL>(
    "acc_l",
    std::bind(
      &Component::accL, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->speed_j_srv_ = this->create_service<mg400_msgs::srv::SpeedJ>(
    "speed_j",
    std::bind(
      &Component::speedJ, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->speed_l_srv_ = this->create_service<mg400_msgs::srv::SpeedL>(
    "speed_l",
    std::bind(
      &Component::speedL, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->arch_srv_ = this->create_service<mg400_msgs::srv::Arch>(
    "arch",
    std::bind(
      &Component::arch, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->cp_srv_ = this->create_service<mg400_msgs::srv::CP>(
    "cp",
    std::bind(
      &Component::cp, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->lim_z_srv_ = this->create_service<mg400_msgs::srv::LimZ>(
    "lim_z",
    std::bind(
      &Component::limZ, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->set_arm_orientation_srv_ =
    this->create_service<mg400_msgs::srv::SetArmOrientation>(
    "set_arm_orientation",
    std::bind(
      &Component::setArmOrientation, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->power_on_srv_ = this->create_service<mg400_msgs::srv::PowerOn>(
    "power_on",
    std::bind(
      &Component::powerOn, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->run_script_srv_ = this->create_service<mg400_msgs::srv::RunScript>(
    "run_script",
    std::bind(
      &Component::runScript, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->stop_script_srv_ = this->create_service<mg400_msgs::srv::StopScript>(
    "stop_script",
    std::bind(
      &Component::stopScript, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->pause_script_srv_ = this->create_service<mg400_msgs::srv::PauseScript>(
    "pause_script",
    std::bind(
      &Component::pauseScript, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->continue_script_srv_ =
    this->create_service<mg400_msgs::srv::ContinueScript>(
    "continue_script",
    std::bind(
      &Component::continueScript, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->set_safe_skin_srv_ = this->create_service<mg400_msgs::srv::SetSafeSkin>(
    "set_safe_skin",
    std::bind(
      &Component::setSafeSkin, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->set_obstacle_avoid_srv_ =
    this->create_service<mg400_msgs::srv::SetObstacleAvoid>(
    "set_obstacle_avoid",
    std::bind(
      &Component::setObstacleAvoid, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->set_collistion_level_srv_ =
    this->create_service<mg400_msgs::srv::SetCollisionLevel>(
    "set_collision_level",
    std::bind(
      &Component::setCollisionLevel, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->emergency_stop_srv_ =
    this->create_service<mg400_msgs::srv::EmergencyStop>(
    "emergency_stop",
    std::bind(
      &Component::emergencyStop, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  this->mov_j_srv_ = this->create_service<mg400_msgs::srv::MovJ>(
    "mov_j",
    std::bind(
      &Component::movJ, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->mov_l_srv_ = this->create_service<mg400_msgs::srv::MovL>(
    "mov_l",
    std::bind(
      &Component::movL, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->jump_srv_ = this->create_service<mg400_msgs::srv::Jump>(
    "jump",
    std::bind(
      &Component::jump, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->arc_srv_ = this->create_service<mg400_msgs::srv::Arc>(
    "arc",
    std::bind(
      &Component::arc, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->sync_srv_ = this->create_service<mg400_msgs::srv::Sync>(
    "sync",
    std::bind(
      &Component::sync, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->circle_srv_ = this->create_service<mg400_msgs::srv::Circle>(
    "circle",
    std::bind(
      &Component::circle, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->servo_j_srv_ = this->create_service<mg400_msgs::srv::ServoJ>(
    "servo_j",
    std::bind(
      &Component::servoJ, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->start_trace_srv_ = this->create_service<mg400_msgs::srv::StartTrace>(
    "start_trace",
    std::bind(
      &Component::startTrace, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->start_path_srv_ = this->create_service<mg400_msgs::srv::StartPath>(
    "start_path",
    std::bind(
      &Component::startPath, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->move_jog_srv_ = this->create_service<mg400_msgs::srv::MoveJog>(
    "move_jog",
    std::bind(
      &Component::moveJog, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->servo_p_srv_ = this->create_service<mg400_msgs::srv::ServoP>(
    "servo_p",
    std::bind(
      &Component::servoP, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->rel_mov_j_srv_ = this->create_service<mg400_msgs::srv::RelMovJ>(
    "rel_mov_j",
    std::bind(
      &Component::relMovJ, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->rel_mov_l_srv_ = this->create_service<mg400_msgs::srv::RelMovL>(
    "rel_mov_l",
    std::bind(
      &Component::relMovL, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->joint_mov_j_srv_ = this->create_service<mg400_msgs::srv::JointMovJ>(
    "joint_mov_j",
    std::bind(
      &Component::jointMovJ, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  RCLCPP_INFO(
    this->get_logger(),
    "Ready");
}

Component::~Component()
{}

void Component::getJointState(double * js)
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

void Component::publishJointState()
{
  double joint_state[6];
  this->getJointState(joint_state);

  this->joint_state_pub_->publish(
    convert::toJointState(
      joint_state[0], joint_state[1],
      joint_state[2], joint_state[3]));
}

// dashboard
void Component::enableRobot(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::EnableRobot::Request::SharedPtr request,
  mg400_msgs::srv::EnableRobot::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  (void)request;  // for linter
  try {
    this->commander_->enableRobot();
    response->res = 0;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    this->commander_->clearError();
    response->res = -1;
  }
}

void Component::disableRobot(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::DisableRobot::Request::SharedPtr request,
  mg400_msgs::srv::DisableRobot::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  (void)request;  // for linter
  try {
    this->commander_->disableRobot();
    response->res = 0;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    this->commander_->clearError();
    response->res = -1;
  }
}

void Component::clearError(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::ClearError::Request::SharedPtr request,
  mg400_msgs::srv::ClearError::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  (void)request;  // for linter
  try {
    this->commander_->clearError();
    response->res = 0;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    this->commander_->clearError();
    response->res = -1;
  }
}

void Component::getErrorID(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::GetErrorID::Request::SharedPtr request,
  mg400_msgs::srv::GetErrorID::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  (void)request;  // for linter
  try {
    this->commander_->getErrorID();
    response->res = 0;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    this->commander_->clearError();
    response->res = -1;
  }
}

void Component::resetRobot(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::ResetRobot::Request::SharedPtr request,
  mg400_msgs::srv::ResetRobot::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  (void)request;  // for linter
  try {
    this->commander_->resetRobot();
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::speedFactor(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::SpeedFactor::Request::SharedPtr request,
  mg400_msgs::srv::SpeedFactor::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    this->commander_->speedFactor(request->ratio);
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::user(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::User::Request::SharedPtr request,
  mg400_msgs::srv::User::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(cmd, sizeof(cmd), "User(%d)", request->index);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::tool(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::Tool::Request::SharedPtr request,
  mg400_msgs::srv::Tool::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(cmd, sizeof(cmd), "Tool(%d)", request->index);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::robotMode(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::RobotMode::Request::SharedPtr request,
  mg400_msgs::srv::RobotMode::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  (void)request;  // for linter
  try {
    const char * cmd = "RobotMode()";
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::payload(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::Payload::Request::SharedPtr request,
  mg400_msgs::srv::Payload::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(
      cmd, sizeof(cmd),
      "PayLoad(%0.3f, %0.3f)", request->weight, request->inertia);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::dO(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::DO::Request::SharedPtr request,
  mg400_msgs::srv::DO::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(
      cmd, sizeof(cmd),
      "DO(%d, %d)", request->index, request->status);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::dOExecute(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::DOExecute::Request::SharedPtr request,
  mg400_msgs::srv::DOExecute::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(
      cmd, sizeof(cmd),
      "DOExecute(%d, %d)", request->index, request->status);
    commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::toolDO(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::ToolDO::Request::SharedPtr request,
  mg400_msgs::srv::ToolDO::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(
      cmd, sizeof(cmd),
      "ToolDO(%d, %d)", request->index, request->status);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::toolDOExecute(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::ToolDOExecute::Request::SharedPtr request,
  mg400_msgs::srv::ToolDOExecute::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(
      cmd, sizeof(cmd),
      "ToolDOExecute(%d, %d)", request->index, request->status);
    commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::aO(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::AO::Request::SharedPtr request,
  mg400_msgs::srv::AO::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(
      cmd, sizeof(cmd),
      "AO(%d, %d)", request->index, request->status);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::aOExecute(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::AOExecute::Request::SharedPtr request,
  mg400_msgs::srv::AOExecute::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(
      cmd, sizeof(cmd), "AOExecute(%d, %0.3f)", request->index,
      static_cast<float>(request->value));
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::accJ(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::AccJ::Request::SharedPtr request,
  mg400_msgs::srv::AccJ::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(cmd, sizeof(cmd), "AccJ(%d)", request->r);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::accL(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::AccL::Request::SharedPtr request,
  mg400_msgs::srv::AccL::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(cmd, sizeof(cmd), "AccL(%d)", request->r);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::speedJ(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::SpeedJ::Request::SharedPtr request,
  mg400_msgs::srv::SpeedJ::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(cmd, sizeof(cmd), "SpeedJ(%d)", request->r);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::speedL(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::SpeedL::Request::SharedPtr request,
  mg400_msgs::srv::SpeedL::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(cmd, sizeof(cmd), "SpeedL(%d)", request->r);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::arch(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::Arch::Request::SharedPtr request,
  mg400_msgs::srv::Arch::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(cmd, sizeof(cmd), "Arch(%d)", request->index);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::cp(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::CP::Request::SharedPtr request,
  mg400_msgs::srv::CP::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(cmd, sizeof(cmd), "CP(%d)", request->r);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::limZ(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::LimZ::Request::SharedPtr request,
  mg400_msgs::srv::LimZ::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(cmd, sizeof(cmd), "LimZ(%d)", request->value);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::setArmOrientation(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::SetArmOrientation::Request::SharedPtr request,
  mg400_msgs::srv::SetArmOrientation::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(
      cmd, sizeof(cmd),
      "SetArmOrientation(%d,%d,%d,%d)",
      request->lor_r, request->uor_d,
      request->for_n, request->config_6);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::powerOn(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::PowerOn::Request::SharedPtr request,
  mg400_msgs::srv::PowerOn::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  (void)request;  // for linter
  try {
    const char * cmd = "PowerOn()";
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::runScript(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::RunScript::Request::SharedPtr request,
  mg400_msgs::srv::RunScript::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(
      cmd, sizeof(cmd),
      "RunScript(%s)", request->project_name.c_str());
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::stopScript(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::StopScript::Request::SharedPtr request,
  mg400_msgs::srv::StopScript::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  (void)request;  // for linter
  try {
    const char * cmd = "StopScript()";
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::pauseScript(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::PauseScript::Request::SharedPtr request,
  mg400_msgs::srv::PauseScript::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  (void)request;  // for linter
  try {
    const char * cmd = "PauseScript()";
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::continueScript(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::ContinueScript::Request::SharedPtr request,
  mg400_msgs::srv::ContinueScript::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  (void)request;  // for linter
  try {
    const char * cmd = "ContinueScript()";
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::setSafeSkin(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::SetSafeSkin::Request::SharedPtr request,
  mg400_msgs::srv::SetSafeSkin::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(cmd, sizeof(cmd), "SetSafeSkin(%d)", request->status);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::setObstacleAvoid(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::SetObstacleAvoid::Request::SharedPtr request,
  mg400_msgs::srv::SetObstacleAvoid::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(cmd, sizeof(cmd), "SetObstacleAvoid(%d)", request->status);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::setCollisionLevel(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::SetCollisionLevel::Request::SharedPtr request,
  mg400_msgs::srv::SetCollisionLevel::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(cmd, sizeof(cmd), "SetCollisionLevel(%d)", request->level);
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::emergencyStop(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::EmergencyStop::Request::SharedPtr request,
  mg400_msgs::srv::EmergencyStop::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  (void)request;  // for linter
  try {
    char cmd[100];
    snprintf(cmd, sizeof(cmd), "EmergencyStop()");
    this->commander_->dashSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

// real time

void Component::movJ(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::MovJ::Request::SharedPtr request,
  mg400_msgs::srv::MovJ::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    this->commander_->movJ(
      request->x, request->y, request->z,
      request->a, request->b, request->c);
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::movL(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::MovL::Request::SharedPtr request,
  mg400_msgs::srv::MovL::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    this->commander_->movL(
      request->x, request->y, request->z,
      request->a, request->b, request->c);
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::servoJ(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::ServoJ::Request::SharedPtr request,
  mg400_msgs::srv::ServoJ::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    this->commander_->servoJ(
      request->j1, request->j2, request->j3,
      request->j4, request->j5, request->j6);
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::arc(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::Arc::Request::SharedPtr request,
  mg400_msgs::srv::Arc::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    this->commander_->arc(
      request->x1, request->y1, request->z1,
      request->rx1, request->ry1, request->rz1,
      request->x2, request->y2, request->z2,
      request->rx2, request->ry2, request->rz2);
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::circle(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::Circle::Request::SharedPtr request,
  mg400_msgs::srv::Circle::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    this->commander_->circle(
      request->x1, request->y1, request->z1,
      request->rx1, request->ry1, request->rz1,
      request->x2, request->y2, request->z2,
      request->rx2, request->ry2, request->rz2);
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::jump(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::Jump::Request::SharedPtr request,
  mg400_msgs::srv::Jump::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(
      cmd,
      sizeof(cmd),
      "Jump(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)",
      request->offset1, request->offset2, request->offset3,
      request->offset4, request->offset5, request->offset6);
    this->commander_->realSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::servoP(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::ServoP::Request::SharedPtr request,
  mg400_msgs::srv::ServoP::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    this->commander_->servoP(
      request->x, request->y, request->z,
      request->a, request->b, request->c);
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::relMovJ(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::RelMovJ::Request::SharedPtr request,
  mg400_msgs::srv::RelMovJ::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    this->commander_->relMovJ(
      request->offset1, request->offset2, request->offset3,
      request->offset4, request->offset5, request->offset6);
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::relMovL(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::RelMovL::Request::SharedPtr request,
  mg400_msgs::srv::RelMovL::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    this->commander_->relMovL(
      request->x, request->y, request->z);
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::jointMovJ(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::JointMovJ::Request::SharedPtr request,
  mg400_msgs::srv::JointMovJ::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    this->commander_->jointMovJ(
      request->j1, request->j2, request->j3,
      request->j4, request->j5, request->j6);
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::sync(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::Sync::Request::SharedPtr request,
  mg400_msgs::srv::Sync::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  (void)request;  // for linter
  try {
    const char * cmd = "Sync()";
    this->commander_->realSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::startTrace(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::StartTrace::Request::SharedPtr request,
  mg400_msgs::srv::StartTrace::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(cmd, sizeof(cmd), "StartTrace(%s)", request->trace_name.c_str());
    this->commander_->realSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::startPath(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::StartPath::Request::SharedPtr request,
  mg400_msgs::srv::StartPath::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(
      cmd, sizeof(cmd), "StartPath(%s,%d,%d)",
      request->trace_name.c_str(),
      request->const_val,
      request->cart);
    this->commander_->realSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::startFCTrace(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::StartFCTrace::Request::SharedPtr request,
  mg400_msgs::srv::StartFCTrace::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    char cmd[100];
    snprintf(cmd, sizeof(cmd), "StartFCTrace(%s)", request->trace_name.c_str());
    this->commander_->realSendCmd(cmd, strlen(cmd));
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}

void Component::moveJog(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const mg400_msgs::srv::MoveJog::Request::SharedPtr request,
  mg400_msgs::srv::MoveJog::Response::SharedPtr response
)
{
  (void)request_header;  // for linter
  try {
    this->commander_->moveJog(request->axis_id);
    response->res = 0;
  } catch (const mg400_interface::TcpClientException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    response->res = -1;
  }
}
}  // namespace mg400_control
