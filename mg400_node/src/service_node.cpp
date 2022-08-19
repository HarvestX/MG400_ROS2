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


#include "mg400_node/service_node.hpp"


namespace mg400_node
{
ServiceNode::ServiceNode(const rclcpp::NodeOptions & options)
: Node("service_node", options),
  prefix_(this->declare_parameter("prefix", "")),
  error_msg_generator_(
    std::make_unique<mg400_interface::ErrorMsgGenerator>(
      "alarm_controller.json"))
{
  this->declare_parameter<std::string>(
    "service_level",
    "service_level_1");
  this->get_parameter("service_level", this->service_level);
  this->service_level_map["service_level_1"] = 1;
  this->service_level_map["service_level_2"] = 2;
  this->service_level_map["service_level_3"] = 3;

  const std::string ip_address =
    this->declare_parameter<std::string>("ip_address", "192.168.1.6");

  this->db_tcp_if_ =
    std::make_unique<mg400_interface::DashboardTcpInterface>(ip_address);
  this->mt_tcp_if_ =
    std::make_unique<mg400_interface::MotionTcpInterface>(ip_address);
  this->rt_tcp_if_ =
    std::make_unique<mg400_interface::RealtimeFeedbackTcpInterface>(ip_address);

  this->initTcpIf();

  this->db_commander_ =
    std::make_unique<mg400_interface::DashboardCommander>(
    this->db_tcp_if_.get());
  this->mt_commander_ =
    std::make_unique<mg400_interface::MotionCommander>(
    this->mt_tcp_if_.get());

  // ROS Interfaces
  this->joint_state_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
  using namespace std::chrono_literals;
  this->js_timer_ = this->create_wall_timer(
    10ms, std::bind(&ServiceNode::onJsTimer, this));
  this->error_timer_ = this->create_wall_timer(
    500ms, std::bind(&ServiceNode::onErrorTimer, this));

  // Service Initialization
  switch (this->get_service_level(this->service_level)) {
    case 3:
      /* service_level_3 */
      this->tool_do_execute_srv_ =
        this->create_service<mg400_srv::ToolDOExecute>(
        "tool_do_execute",
        std::bind(
          &ServiceNode::toolDOExecute, this,
          std::placeholders::_1, std::placeholders::_2));
    // fall through
    case 2:
      /* service_level_2 */
      this->reset_robot_srv_ =
        this->create_service<mg400_srv::ResetRobot>(
        "reset_robot",
        std::bind(
          &ServiceNode::resetRobot, this,
          std::placeholders::_1, std::placeholders::_2));

      this->speed_factor_srv_ =
        this->create_service<mg400_srv::SpeedFactor>(
        "speed_factor",
        std::bind(
          &ServiceNode::speedFactor, this,
          std::placeholders::_1, std::placeholders::_2));

      // this->speed_j_srv_ =
      //   this->create_service<mg400_srv::SpeedJ>(
      //   "speed_j",
      //   std::bind(
      //     &ServiceNode::speedJ, this,
      //     std::placeholders::_1, std::placeholders::_2));

      // this->speed_l_srv_ =
      //   this->create_service<mg400_srv::SpeedL>(
      //   "speed_l",
      //   std::bind(
      //     &ServiceNode::speedL, this,
      //     std::placeholders::_1, std::placeholders::_2));

      // this->acc_j_srv_ =
      //   this->create_service<mg400_srv::AccJ>(
      //   "acc_j",
      //   std::bind(
      //     &ServiceNode::accJ, this,
      //     std::placeholders::_1, std::placeholders::_2));

      // this->acc_l_srv_ =
      //   this->create_service<mg400_srv::AccL>(
      //   "acc_l",
      //   std::bind(
      //     &ServiceNode::accL, this,
      //     std::placeholders::_1, std::placeholders::_2));

      this->joint_mov_j_srv_ =
        this->create_service<mg400_srv::JointMovJ>(
        "joint_mov_j",
        std::bind(
          &ServiceNode::jointMovJ, this,
          std::placeholders::_1, std::placeholders::_2));

      this->mov_l_srv_ =
        this->create_service<mg400_srv::MovL>(
        "mov_l",
        std::bind(
          &ServiceNode::movL, this,
          std::placeholders::_1, std::placeholders::_2));
    // fall through
    case 1:
      /* service_level_1 */
      this->clear_error_srv_ =
        this->create_service<mg400_srv::ClearError>(
        "clear_error",
        std::bind(
          &ServiceNode::clearError, this,
          std::placeholders::_1, std::placeholders::_2));

      this->disable_robot_srv_ =
        this->create_service<mg400_srv::DisableRobot>(
        "disable_robot",
        std::bind(
          &ServiceNode::disableRobot, this,
          std::placeholders::_1, std::placeholders::_2));

      this->enable_robot_srv_ =
        this->create_service<mg400_srv::EnableRobot>(
        "enable_robot",
        std::bind(
          &ServiceNode::enableRobot, this,
          std::placeholders::_1, std::placeholders::_2));

      this->move_jog_srv_ =
        this->create_service<mg400_srv::MoveJog>(
        "move_jog",
        std::bind(
          &ServiceNode::moveJog, this,
          std::placeholders::_1, std::placeholders::_2));

      this->mov_j_srv_ =
        this->create_service<mg400_srv::MovJ>(
        "mov_j",
        std::bind(
          &ServiceNode::movJ, this,
          std::placeholders::_1, std::placeholders::_2));
      break;
  }
  // END Ros Interfaces


  // Robot Initialization
  this->db_commander_->clearError();
}

ServiceNode::~ServiceNode()
{}

uint8_t ServiceNode::get_service_level(std::string key)
{
  auto found = this->service_level_map.find(key);
  if (found == end(this->service_level_map)) {
    return 0;
  }
  return found->second;
}

void ServiceNode::initTcpIf()
{
  using namespace std::chrono_literals;
  this->db_tcp_if_->init();
  this->rt_tcp_if_->init();
  this->mt_tcp_if_->init();

  const auto start = this->get_clock()->now();
  while (!this->db_tcp_if_->isConnected() ||
    !this->rt_tcp_if_->isConnected() ||
    !this->mt_tcp_if_->isConnected())
  {
    if (this->get_clock()->now() - start > rclcpp::Duration(3s)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Could not connect DOBOT MG400.");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Waiting for the connection...");
    rclcpp::sleep_for(1s);
  }
}

void ServiceNode::onJsTimer()
{
  std::array<double, 6> joint_states;
  this->rt_tcp_if_->getCurrentJointStates(joint_states);

  this->joint_state_pub_->publish(
    mg400_interface::getJointState(
      joint_states[0], joint_states[1], joint_states[2], joint_states[3],
      this->prefix_));
}

void ServiceNode::onErrorTimer()
{
  if (this->rt_tcp_if_->getRobotMode() != mg400_interface::RobotMode::ERROR) {
    return;
  }

  std::stringstream ss;
  const auto joints_error_ids = this->db_commander_->getErrorId();
  for (size_t i = 0; i < joints_error_ids.size(); ++i) {
    if (joints_error_ids.at(i).empty()) {
      continue;
    }
    ss << "Joint" << (i + 1) << ":" << std::endl;
    for (auto error_id : joints_error_ids.at(i)) {
      const auto message = this->error_msg_generator_->get(error_id);
      ss << "\t" << message << std::endl;
    }
  }
  RCLCPP_ERROR(
    this->get_logger(),
    ss.str().c_str());
  this->db_commander_->clearError();
}

void ServiceNode::clearError(
  const mg400_srv::ClearError::Request::SharedPtr,
  mg400_srv::ClearError::Response::SharedPtr response
)
{
  response->result = this->db_commander_->clearError();
}

void ServiceNode::resetRobot(
  const mg400_srv::ResetRobot::Request::SharedPtr,
  mg400_srv::ResetRobot::Response::SharedPtr response
)
{
  response->result = this->db_commander_->resetRobot();
}

void ServiceNode::disableRobot(
  const mg400_srv::DisableRobot::Request::SharedPtr,
  mg400_srv::DisableRobot::Response::SharedPtr response
)
{
  response->result = this->db_commander_->disableRobot();
}

void ServiceNode::enableRobot(
  const mg400_srv::EnableRobot::Request::SharedPtr,
  mg400_srv::EnableRobot::Response::SharedPtr response
)
{
  response->result = this->db_commander_->enableRobot();
}

void ServiceNode::toolDOExecute(
  const mg400_srv::ToolDOExecute::Request::SharedPtr request,
  mg400_srv::ToolDOExecute::Response::SharedPtr response
)
{
  response->result = this->db_commander_->toolDOExecute(
    request->index,
    request->status);
}

void ServiceNode::speedFactor(
  const mg400_srv::SpeedFactor::Request::SharedPtr request,
  mg400_srv::SpeedFactor::Response::SharedPtr response
)
{
  response->result = this->db_commander_->speedFactor(request->ratio);
}

//TODO: Uncomment after command implementation.
// void ServiceNode::speedJ(
//   const mg400_srv::SpeedJ::Request::SharedPtr request,
//   mg400_srv::SpeedJ::Response::SharedPtr response
// )
// {
//   response->result = this->db_commander_->speedJ(request->r);
// }

// void ServiceNode::speedL(
//   const mg400_srv::SpeedL::Request::SharedPtr request,
//   mg400_srv::SpeedL::Response::SharedPtr response
// )
// {
//   response->result = this->db_commander_->speedL(request->r);
// }

// void ServiceNode::accJ(
//   const mg400_srv::AccJ::Request::SharedPtr request,
//   mg400_srv::AccJ::Response::SharedPtr response
// )
// {
//   response->result = this->db_commander_->accJ(request->r);
// }

// void ServiceNode::accL(
//   const mg400_srv::AccL::Request::SharedPtr request,
//   mg400_srv::AccL::Response::SharedPtr response
// )
// {
//   response->result = this->db_commander_->accL(request->r);
// }
// end (Uncomment after command implementation.)

void ServiceNode::jointMovJ(
  const mg400_srv::JointMovJ::Request::SharedPtr request,
  mg400_srv::JointMovJ::Response::SharedPtr)
{
  this->mt_commander_->jointMovJ(
    request->j1, request->j2, request->j3,
    request->j4, request->j5, request->j6);
}

void ServiceNode::moveJog(
  const mg400_srv::MoveJog::Request::SharedPtr request,
  mg400_srv::MoveJog::Response::SharedPtr)
{
  this->mt_commander_->moveJog(request->axis_id);
}

void ServiceNode::movJ(
  const mg400_srv::MovJ::Request::SharedPtr request,
  mg400_srv::MovJ::Response::SharedPtr
)
{
  this->mt_commander_->movJ(
    request->x, request->y, request->z,
    request->rx, request->ry, request->rz);
}

void ServiceNode::movL(
  const mg400_srv::MovL::Request::SharedPtr request,
  mg400_srv::MovL::Response::SharedPtr)
{
  this->mt_commander_->movL(
    request->x, request->y, request->z,
    request->rx, request->ry, request->rz);
}
}  // namespace mg400_node
