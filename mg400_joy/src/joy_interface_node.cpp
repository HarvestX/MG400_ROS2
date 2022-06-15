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


#include "mg400_joy/joy_interface_node.hpp"

namespace mg400_joy
{
JoyInterfaceNode::JoyInterfaceNode(const rclcpp::NodeOptions & node_options)
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

  this->mg400_clear_error_clnt_ =
    this->create_client<mg400_msgs::srv::ClearError>("clear_error");
  this->mg400_move_jog_clnt_ =
    this->create_client<mg400_msgs::srv::MoveJog>("move_jog");
  this->mg400_enable_robot_clnt_ =
    this->create_client<mg400_msgs::srv::EnableRobot>("enable_robot");
  this->mg400_disable_robot_clnt_ =
    this->create_client<mg400_msgs::srv::DisableRobot>("disable_robot");

  this->joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&JoyInterfaceNode::onJoy, this, std::placeholders::_1));

  this->current_robot_state_ = ROBOT_STATE::DISABLED;
  this->current_service_state_ = SERVICE_STATE::DONE;

  RCLCPP_INFO(
    this->get_logger(),
    "Ready.");
}

void JoyInterfaceNode::onJoy(sensor_msgs::msg::Joy::ConstSharedPtr joy_msg)
{
  if (this->current_service_state_ == SERVICE_STATE::IN_PROGRESS) {
    return;
  }

  this->p9n_if_->setJoyMsg(joy_msg);

  if (this->p9n_if_->pressedCircle()) {
    this->callClearError();
    using namespace std::chrono_literals;
    rclcpp::sleep_for(500ms);
  }

  if (this->p9n_if_->pressedStart()) {
    if (this->current_robot_state_ == ROBOT_STATE::ENABLED) {
      this->callDisableRobot();
    } else if (this->current_robot_state_ == ROBOT_STATE::DISABLED) {
      this->callEnableRobot();
    }
    using namespace std::chrono_literals;
    rclcpp::sleep_for(500ms);
  }

  if (this->isStickTilted()) {
    auto axis_id = this->tiltedStick2JogAxis();
    this->callMoveJog(axis_id);
    this->current_robot_state_ = ROBOT_STATE::MOVING;
  } else if (this->current_robot_state_ == ROBOT_STATE::MOVING) {
    // Stop action
    this->callMoveJog("");
    this->current_robot_state_ = ROBOT_STATE::ENABLED;
  }
}

void JoyInterfaceNode::callClearError()
{
  auto req = std::make_shared<mg400_msgs::srv::ClearError::Request>();
  using namespace std::literals::chrono_literals;
  while (!this->mg400_clear_error_clnt_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for the service. Exiting.");
      return;
    }
  }
  using ServiceResponseFuture =
    rclcpp::Client<mg400_msgs::srv::ClearError>::SharedFuture;
  auto req_callback = [this](ServiceResponseFuture future)
    {
      auto result = future.get();
      this->current_service_state_ = SERVICE_STATE::DONE;
    };
  auto future_result =
    this->mg400_clear_error_clnt_->async_send_request(req, req_callback);
  this->current_service_state_ = SERVICE_STATE::IN_PROGRESS;

  // For compiler warning
  (void) future_result;
}

void JoyInterfaceNode::callEnableRobot()
{
  if (this->current_robot_state_ == ROBOT_STATE::ENABLED) {
    RCLCPP_INFO(this->get_logger(), "Robot already enabled");
    return;
  }

  auto req = std::make_shared<mg400_msgs::srv::EnableRobot::Request>();
  using namespace std::chrono_literals;
  while (!this->mg400_enable_robot_clnt_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for service. Exiting.");
      return;
    }
  }

  using ServiceResponseFuture =
    rclcpp::Client<mg400_msgs::srv::EnableRobot>::SharedFuture;
  auto req_callback = [this](ServiceResponseFuture future) {
      auto result = future.get();
      if (result->res == 0) {
        this->current_robot_state_ = ROBOT_STATE::ENABLED;
      }
      this->current_service_state_ = SERVICE_STATE::DONE;
    };
  auto future_result =
    this->mg400_enable_robot_clnt_->async_send_request(req, req_callback);
  this->current_service_state_ = SERVICE_STATE::IN_PROGRESS;

  // For compiler warning
  (void) future_result;
}

void JoyInterfaceNode::callDisableRobot()
{
  if (this->current_robot_state_ == ROBOT_STATE::DISABLED) {
    RCLCPP_INFO(this->get_logger(), "Robot already diabled");
    return;
  }

  auto req = std::make_shared<mg400_msgs::srv::DisableRobot::Request>();
  using namespace std::chrono_literals;
  while (!this->mg400_disable_robot_clnt_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for service. Exiting.");
      return;
    }
  }

  using ServiceResponseFuture =
    rclcpp::Client<mg400_msgs::srv::DisableRobot>::SharedFuture;
  auto req_callback = [this](ServiceResponseFuture future) {
      auto result = future.get();
      if (result->res == 0) {
        this->current_robot_state_ = ROBOT_STATE::DISABLED;
      }
      this->current_service_state_ = SERVICE_STATE::DONE;
    };
  auto future_result =
    this->mg400_disable_robot_clnt_->async_send_request(req, req_callback);
  this->current_service_state_ = SERVICE_STATE::IN_PROGRESS;

  // For compiler warning
  (void)future_result;
}

void JoyInterfaceNode::callMoveJog(const std::string & axis_id)
{
  if (this->current_robot_state_ == ROBOT_STATE::DISABLED) {
    RCLCPP_INFO(this->get_logger(), "Please enable robot. Press Start.");
    return;
  }

  auto req = std::make_shared<mg400_msgs::srv::MoveJog::Request>();
  req->axis_id = axis_id;
  using namespace std::chrono_literals;
  while (!this->mg400_move_jog_clnt_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for service. Exiting.");
      return;
    }
  }

  using ServiceResponseFuture =
    rclcpp::Client<mg400_msgs::srv::MoveJog>::SharedFuture;
  auto req_callback = [this](ServiceResponseFuture future) {
      auto result = future.get();
      if (result->res != 0) {
        RCLCPP_ERROR(
          this->get_logger(),
          "MoveJog client received invalid response.");
      }
      this->current_service_state_ = SERVICE_STATE::DONE;
    };
  auto future_result =
    this->mg400_move_jog_clnt_->async_send_request(req, req_callback);
  this->current_service_state_ = SERVICE_STATE::IN_PROGRESS;

  // For compiler warning
  (void)future_result;
}

bool JoyInterfaceNode::isStickTilted() const
{
  bool res = false;
  if (
    std::fabs(this->p9n_if_->tiltedStickLX()) > this->TILT_THRESHOLD_ ||
    std::fabs(this->p9n_if_->tiltedStickLY()) > this->TILT_THRESHOLD_ ||
    std::fabs(this->p9n_if_->tiltedStickRX()) > this->TILT_THRESHOLD_ ||
    std::fabs(this->p9n_if_->tiltedStickRY()) > this->TILT_THRESHOLD_)
  {
    res = true;
  }
  return res;
}

std::string JoyInterfaceNode::tiltedStick2JogAxis() const
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

  mg400_interface::JogMode mode = mg400_interface::JogMode::STOP;
  switch (max_idx) {
    case 0:
      {
        mode = mg400_interface::JogMode::J1_NEGATIVE;
        if (this->p9n_if_->tiltedStickLX() < 0.0) {
          mode = mg400_interface::JogMode::J1_POSITIVE;
        }
        break;

      }
    case 1:
      {
        mode = mg400_interface::JogMode::J2_NEGATIVE;
        if (this->p9n_if_->tiltedStickLY() < 0.0) {
          mode = mg400_interface::JogMode::J2_POSITIVE;
        }
        break;
      }
    case 2:
      {
        mode = mg400_interface::JogMode::J4_NEGATIVE;
        if (this->p9n_if_->tiltedStickRX() < 0.0) {
          mode = mg400_interface::JogMode::J4_POSITIVE;
        }
        break;
      }
    case 3:
      {
        mode = mg400_interface::JogMode::J3_NEGATIVE;
        if (this->p9n_if_->tiltedStickRY() < 0.0) {
          mode = mg400_interface::JogMode::J3_POSITIVE;
        }
        break;
      }
    default:
      break;
  }
  return mg400_interface::getAxisIdStr(mode);
}
}  // namespace mg400_joy
