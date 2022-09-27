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


#include "mg400_node/joint_handler_node.hpp"


namespace mg400_node
{
JointHandlerNode::JointHandlerNode(const rclcpp::NodeOptions & options)
: Node("joint_handler_node", options),
  prefix_(this->declare_parameter("prefix", "")),
  error_msg_generator_(
    std::make_unique<mg400_interface::ErrorMsgGenerator>(
      "alarm_controller.json"))
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
    rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
  using namespace std::chrono_literals;
  this->js_timer_ = this->create_wall_timer(
    500ms, std::bind(&JointHandlerNode::onJsTimer, this));
  this->error_timer_ = this->create_wall_timer(
    500ms, std::bind(&JointHandlerNode::onErrorTimer, this));

  this->joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_commands", rclcpp::SensorDataQoS().keep_last(1).best_effort(),
    std::bind(&JointHandlerNode::onJoint, this, std::placeholders::_1));

  this->enable_robot_srv_ =
    this->create_service<mg400_msgs::srv::EnableRobot>(
    "enable_robot",
    std::bind(
      &JointHandlerNode::enableRobot, this,
      std::placeholders::_1, std::placeholders::_2));
}

JointHandlerNode::~JointHandlerNode()
{}

void JointHandlerNode::onJoint(
  sensor_msgs::msg::JointState::ConstSharedPtr joint_msg)
{
  callJointMovJ(
    joint_msg->position.at(0),
    joint_msg->position.at(1),
    joint_msg->position.at(6),
    joint_msg->position.at(7),
    0.0,
    0.0
  );
}

void JointHandlerNode::initTcpIf()
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

void JointHandlerNode::onJsTimer()
{
  std::array<double, 6> joint_states;
  this->rt_tcp_if_->getCurrentJointStates(joint_states);

  this->joint_state_pub_->publish(
    mg400_interface::getJointState(
      joint_states[0], joint_states[1], joint_states[2], joint_states[3],
      this->prefix_));
}

void JointHandlerNode::onErrorTimer()
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
void JointHandlerNode::enableRobot(
  const mg400_msgs::srv::EnableRobot::Request::SharedPtr,
  mg400_msgs::srv::EnableRobot::Response::SharedPtr response
)
{
  try {
    this->db_commander_->enableRobot();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "%s", e.what());
    response->result = false;
  }
  response->result = true;
}

void JointHandlerNode::callJointMovJ(
  const double j1, const double j2, const double j3,
  const double j4, const double j5, const double j6)
{
  this->mt_commander_->jointMovJ(
    j1, j2, j3,
    j4, j5, j6);
}
}  // namespace mg400_node
