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
  prefix_(this->declare_parameter("prefix", ""))
{
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

  // Service Initialization
  this->clear_error_srv_ =
    this->create_service<mg400_msgs::srv::ClearError>(
    "clear_error",
    std::bind(
      &ServiceNode::clearError, this,
      std::placeholders::_1, std::placeholders::_2));

  this->disable_robot_srv_ =
    this->create_service<mg400_msgs::srv::DisableRobot>(
    "disable_robot",
    std::bind(
      &ServiceNode::disableRobot, this,
      std::placeholders::_1, std::placeholders::_2));

  this->enable_robot_srv_ =
    this->create_service<mg400_msgs::srv::EnableRobot>(
    "enable_robot",
    std::bind(
      &ServiceNode::enableRobot, this,
      std::placeholders::_1, std::placeholders::_2));

  /**
  this->get_error_id_srv_ =
    this->create_service<mg400_msgs::srv::GetErrorID>(
    "get_error_id",
    std::bind(
      &ServiceNode::getErrorId, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  **/

  this->move_jog_srv_ =
    this->create_service<mg400_msgs::srv::MoveJog>(
    "move_jog",
    std::bind(
      &ServiceNode::moveJog, this,
      std::placeholders::_1, std::placeholders::_2));

  this->mov_j_srv_ =
    this->create_service<mg400_msgs::srv::MovJ>(
    "mov_j",
    std::bind(
      &ServiceNode::movJ, this,
      std::placeholders::_1, std::placeholders::_2));

  this->mov_l_srv_ =
    this->create_service<mg400_msgs::srv::MovL>(
    "mov_l",
    std::bind(
      &ServiceNode::movL, this,
      std::placeholders::_1, std::placeholders::_2));
  // END Ros Interfaces


  // Robot Initialization
  this->db_commander_->clearError();
}

ServiceNode::~ServiceNode()
{}

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

void ServiceNode::clearError(
  const mg400_msgs::srv::ClearError::Request::SharedPtr,
  mg400_msgs::srv::ClearError::Response::SharedPtr response
)
{
  response->result = this->db_commander_->clearError();
}

void ServiceNode::disableRobot(
  const mg400_msgs::srv::DisableRobot::Request::SharedPtr,
  mg400_msgs::srv::DisableRobot::Response::SharedPtr response
)
{
  response->result = this->db_commander_->disableRobot();
}

void ServiceNode::enableRobot(
  const mg400_msgs::srv::EnableRobot::Request::SharedPtr,
  mg400_msgs::srv::EnableRobot::Response::SharedPtr response
)
{
  response->result = this->db_commander_->enableRobot();
}

void ServiceNode::moveJog(
  const mg400_msgs::srv::MoveJog::Request::SharedPtr request,
  mg400_msgs::srv::MoveJog::Response::SharedPtr)
{
  this->mt_commander_->moveJog(request->axis_id);
}

void ServiceNode::movJ(
  const mg400_msgs::srv::MovJ::Request::SharedPtr request,
  mg400_msgs::srv::MovJ::Response::SharedPtr
)
{
  this->mt_commander_->movJ(
    request->x, request->y, request->z,
    request->rx, request->ry, request->rz);
}

void ServiceNode::movL(
  const mg400_msgs::srv::MovL::Request::SharedPtr request,
  mg400_msgs::srv::MovL::Response::SharedPtr)
{
  this->mt_commander_->movL(
    request->x, request->y, request->z,
    request->rx, request->ry, request->rz);
}
}  // namespace mg400_node
