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


#include "mg400_joy/mg400_joy_interface_node.hpp"

namespace mg400_joy
{
MG400JoyInterfaceNode::MG400JoyInterfaceNode(const rclcpp::NodeOptions & node_options)
: Node("mg400_joy", node_options)
{
  const std::string hw_name =
    this->declare_parameter<std::string>(
    "hw_type", p9n_interface::HW_NAME::DUALSENSE);

  try {
    this->hw_type_ = p9n_interface::getHwType(hw_name);
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
    RCLCPP_ERROR(
      this->get_logger(), "Please select hardware from %s.",
      p9n_interface::getAllHwName().c_str());
    rclcpp::shutdown();
    return;
  }

  this->p9n_if_ =
    std::make_unique<p9n_interface::PlayStationInterface>(this->hw_type_);

  this->callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  this->callback_group_executor_.add_callback_group(
    this->callback_group_, this->get_node_base_interface());

  this->mg400_reset_robot_clnt_ =
    this->create_client<mg400_msgs::srv::ResetRobot>(
    "reset_robot", rmw_qos_profile_default, this->callback_group_);
  this->mg400_move_jog_clnt_ =
    this->create_client<mg400_msgs::srv::MoveJog>(
    "move_jog", rmw_qos_profile_default, this->callback_group_);
  this->mg400_enable_robot_clnt_ =
    this->create_client<mg400_msgs::srv::EnableRobot>(
    "enable_robot", rmw_qos_profile_default, this->callback_group_);
  this->mg400_disable_robot_clnt_ =
    this->create_client<mg400_msgs::srv::DisableRobot>(
    "disable_robot", rmw_qos_profile_default, this->callback_group_);

  this->rm_sub_ = this->create_subscription<RobotMode>(
    "robot_mode", rclcpp::SensorDataQoS().keep_last(1),
    [&](const mg400_msgs::msg::RobotMode::ConstSharedPtr msg) {
      this->current_robot_mode_ = msg;
    });

  this->joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&MG400JoyInterfaceNode::onJoy, this, std::placeholders::_1));

  this->current_robot_mode_ = std::make_shared<const RobotMode>();
}


void MG400JoyInterfaceNode::onJoy(const sensor_msgs::msg::Joy::ConstSharedPtr joy_msg)
{
  this->p9n_if_->setJoyMsg(joy_msg);

  if (this->p9n_if_->pressedStart()) {
    if (this->current_robot_mode_->robot_mode == RobotMode::ENABLE) {
      this->callDisableRobot();
    } else if (this->current_robot_mode_->robot_mode == RobotMode::DISABLED) {
      this->callEnableRobot();
    } else {
      RCLCPP_WARN(this->get_logger(), "Please restart robot");
    }
    rclcpp::sleep_for(1s);
    return;
  }

  if (this->p9n_if_->pressedPS()) {
    this->callResetRobot();
    rclcpp::sleep_for(1s);
    return;
  }

  if (this->p9n_if_->pressedR1()) {
    // Swap jog mode
    if (this->current_jog_mode_ == JogMode::JOINT) {
      this->current_jog_mode_ = JogMode::LINEAR;
      RCLCPP_INFO(this->get_logger(), "JogMode: Linear");
    } else {
      this->current_jog_mode_ = JogMode::JOINT;
      RCLCPP_INFO(this->get_logger(), "JogMode: Joint");
    }
    rclcpp::sleep_for(1s);
    return;
  }

  static bool jog_active = false;
  static std::string jog_mode = mg400_msgs::msg::MoveJog::STOP;
  if (this->p9n_if_->isTiltedStickL() || this->p9n_if_->isTiltedStickR()) {
    if (this->tiltedStick2JogAxis(jog_mode)) {
      this->callMoveJog(jog_mode);
      jog_active = true;
    }
  } else if (jog_active) {
    // Stop jog
    this->callMoveJog("");
    jog_active = false;
  }
}

bool MG400JoyInterfaceNode::isEnabled()
{
  if (this->current_robot_mode_->robot_mode != RobotMode::ENABLE) {
    RCLCPP_WARN(this->get_logger(), "Please enable robot. Press Start.");
    rclcpp::sleep_for(1s);
    return false;
  }
  return true;
}

void MG400JoyInterfaceNode::callResetRobot()
{
  auto req = std::make_shared<mg400_msgs::srv::ResetRobot::Request>();
  if (!this->mg400_reset_robot_clnt_->wait_for_service(1s)) {
    RCLCPP_ERROR(
      this->get_logger(), "\"%s\" is not ready",
      this->mg400_reset_robot_clnt_->get_service_name());
    return;
  }

  auto future_result = this->mg400_reset_robot_clnt_->async_send_request(req);

  if (this->callback_group_executor_.spin_until_future_complete(future_result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "\"%s\" service client: async_send_request failed",
      this->mg400_reset_robot_clnt_->get_service_name());
    return;
  }

  const auto wrapped_result = future_result.get();
  if (!wrapped_result->result) {
    RCLCPP_ERROR(
      this->get_logger(),
      "\"%s\" service client: failed",
      this->mg400_reset_robot_clnt_->get_service_name());
  }
}

void MG400JoyInterfaceNode::callEnableRobot()
{
  if (this->current_robot_mode_->robot_mode == RobotMode::ENABLE) {
    RCLCPP_WARN(
      this->get_logger(),
      "robot already enabled");
    return;
  }

  if (!this->mg400_enable_robot_clnt_->wait_for_service(1s)) {
    RCLCPP_ERROR(
      this->get_logger(), "\"%s\" is not ready",
      this->mg400_enable_robot_clnt_->get_service_name());
    return;
  }

  auto req = std::make_shared<mg400_msgs::srv::EnableRobot::Request>();

  auto future_result = this->mg400_enable_robot_clnt_->async_send_request(req);

  if (this->callback_group_executor_.spin_until_future_complete(future_result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "\"%s\" service client: async_send_request failed",
      this->mg400_enable_robot_clnt_->get_service_name());
    return;
  }

  const auto wrapped_result = future_result.get();
  if (!wrapped_result->result) {
    RCLCPP_ERROR(
      this->get_logger(),
      "\"%s\" service client: failed",
      this->mg400_enable_robot_clnt_->get_service_name());
  }
}

void MG400JoyInterfaceNode::callDisableRobot()
{
  if (this->current_robot_mode_->robot_mode == RobotMode::DISABLED) {
    RCLCPP_WARN(
      this->get_logger(),
      "robot already disabled");
    return;
  }

  if (!this->mg400_disable_robot_clnt_->wait_for_service(1s)) {
    RCLCPP_ERROR(
      this->get_logger(), "\"%s\" is not ready",
      this->mg400_disable_robot_clnt_->get_service_name());
    return;
  }

  auto req = std::make_shared<mg400_msgs::srv::DisableRobot::Request>();

  auto future_result = this->mg400_disable_robot_clnt_->async_send_request(req);

  if (this->callback_group_executor_.spin_until_future_complete(future_result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "\"%s\" service client: async_send_request failed",
      this->mg400_disable_robot_clnt_->get_service_name());
    return;
  }

  const auto wrapped_result = future_result.get();
  if (!wrapped_result->result) {
    RCLCPP_ERROR(
      this->get_logger(),
      "\"%s\" service client: failed",
      this->mg400_disable_robot_clnt_->get_service_name());
  }
}

void MG400JoyInterfaceNode::callMoveJog(const std::string & jog_mode)
{
  if (!this->mg400_move_jog_clnt_->wait_for_service(1s)) {
    RCLCPP_ERROR(
      this->get_logger(), "\"%s\" is not ready",
      this->mg400_move_jog_clnt_->get_service_name());
    return;
  }
  const auto & robot_mode = this->current_robot_mode_->robot_mode;
  if (robot_mode != RobotMode::ENABLE &&
    robot_mode != RobotMode::JOG &&
    robot_mode != RobotMode::RUNNING)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Current state is not jog acceptable");
    rclcpp::sleep_for(1s);
    return;
  }

  auto req = std::make_shared<mg400_msgs::srv::MoveJog::Request>();
  req->jog.jog_mode = jog_mode;

  auto future_result = this->mg400_move_jog_clnt_->async_send_request(req);

  if (this->callback_group_executor_.spin_until_future_complete(future_result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "\"%s\" service client: async_send_request failed",
      this->mg400_move_jog_clnt_->get_service_name());
    return;
  }
}

bool MG400JoyInterfaceNode::tiltedStick2JogAxis(std::string & jog_mode) const
{
  std::array<float, 4> tilted_values = {
    std::fabs(this->p9n_if_->tiltedStickLX()),
    std::fabs(this->p9n_if_->tiltedStickLY()),
    std::fabs(this->p9n_if_->tiltedStickRX()),
    std::fabs(this->p9n_if_->tiltedStickRY())
  };

  size_t max_idx = std::distance(
    tilted_values.begin(),
    std::max_element(tilted_values.begin(), tilted_values.end()));

  using MoveJog = mg400_msgs::msg::MoveJog;
  switch (max_idx) {
    case 0:
      {
        if (this->current_jog_mode_ == JogMode::JOINT) {
          jog_mode = MoveJog::J1_POSITIVE;
          if (this->p9n_if_->tiltedStickLX() < 0.0) {
            jog_mode = MoveJog::J1_NEGATIVE;
          }
        } else if (this->current_jog_mode_ == JogMode::LINEAR) {
          jog_mode = MoveJog::X_POSITIVE;
          if (this->p9n_if_->tiltedStickLX() < 0.0) {
            jog_mode = MoveJog::X_NEGATIVE;
          }
        }
        break;
      }
    case 1:
      {
        if (this->current_jog_mode_ == JogMode::JOINT) {
          jog_mode = MoveJog::J2_POSITIVE;
          if (this->p9n_if_->tiltedStickLY() < 0.0) {
            jog_mode = MoveJog::J2_NEGATIVE;
          }
        } else if (this->current_jog_mode_ == JogMode::LINEAR) {
          jog_mode = MoveJog::Y_NEGATIVE;
          if (this->p9n_if_->tiltedStickLY() < 0.0) {
            jog_mode = MoveJog::Y_POSITIVE;
          }
        }
        break;
      }
    case 2:
      {
        jog_mode = MoveJog::J4_POSITIVE;
        if (this->p9n_if_->tiltedStickRX() < 0.0) {
          jog_mode = MoveJog::J4_NEGATIVE;
        }
        break;
      }
    case 3:
      {
        if (this->current_jog_mode_ == JogMode::JOINT) {
          jog_mode = MoveJog::J3_NEGATIVE;
          if (this->p9n_if_->tiltedStickRY() < 0.0) {
            jog_mode = MoveJog::J3_POSITIVE;
          }
        } else if (this->current_jog_mode_ == JogMode::LINEAR) {
          jog_mode = MoveJog::Z_POSITIVE;
          if (this->p9n_if_->tiltedStickRY() < 0.0) {
            jog_mode = MoveJog::Z_NEGATIVE;
          }
        }
        break;
      }
    default:
      return false;
  }
  return true;
}
}  // namespace mg400_joy
