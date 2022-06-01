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

#include "mg400_control/mg400_joy/component.hpp"

namespace mg400_joy
{
JoyComponent::JoyComponent(const rclcpp::NodeOptions & node_options)
: Node("mg400_joy", node_options)
{
  this->button_ = std::make_unique<JoyButton>();
  this->joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy",
    rclcpp::SensorDataQoS(),
    std::bind(&JoyComponent::joyCallback, this, std::placeholders::_1));

  this->clear_error_clnt_ = this->create_client<mg400_msgs::srv::ClearError>("clear_error");
  this->move_jog_clnt_ = this->create_client<mg400_msgs::srv::MoveJog>("move_jog");
  this->enable_robot_clnt_ = this->create_client<mg400_msgs::srv::EnableRobot>("enable_robot");
  this->disable_robot_clnt_ = this->create_client<mg400_msgs::srv::DisableRobot>("disable_robot");

  this->robot_state = 0;

  RCLCPP_INFO(
    this->get_logger(),
    "Ready.");
}

void JoyComponent::joyCallback(const sensor_msgs::msg::Joy::UniquePtr joy_msg)
{
  this->updateButton(joy_msg);

  this->displayInfo();

  this->joyClearError();

  this->joyEnableDisable();

  this->joyMoveJog();

  // TODO: Call jog service here
  // You can access the state of each button as follows
  // this->button_->circle;
}

void JoyComponent::joyClearError()
{
  if (this->button_->circle) {
    auto request = std::make_shared<mg400_msgs::srv::ClearError::Request>();
    using namespace std::literals::chrono_literals;
    while (!clear_error_clnt_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          rclcpp::get_logger(
            "rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    using ServiceResponseFuture =
      rclcpp::Client<mg400_msgs::srv::ClearError>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "Result: %d", result->res);
      };
    auto future_result = clear_error_clnt_->async_send_request(request, response_received_callback);
    RCLCPP_INFO(
      this->get_logger(),
      "circle.");
    if (robot_state == 0) {
      this->robot_state = 1;
    }
    sleep(1);
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Not circle.");
  }
}

void JoyComponent::joyEnableDisable()
{
  if (this->button_->start) {
    if (this->robot_state < 2) {
      auto request = std::make_shared<mg400_msgs::srv::EnableRobot::Request>();
      using namespace std::literals::chrono_literals;
      while (!enable_robot_clnt_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(
            rclcpp::get_logger(
              "rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }
      using ServiceResponseFuture =
        rclcpp::Client<mg400_msgs::srv::EnableRobot>::SharedFuture;
      auto response_received_callback = [this](ServiceResponseFuture future) {
          auto result = future.get();
          RCLCPP_INFO(this->get_logger(), "Result: %d", result->res);
        };
      auto future_result = enable_robot_clnt_->async_send_request(
        request,
        response_received_callback);
      this->robot_state = 2;
      sleep(1);
    } else {
      auto request = std::make_shared<mg400_msgs::srv::DisableRobot::Request>();
      using namespace std::literals::chrono_literals;
      while (!disable_robot_clnt_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(
            rclcpp::get_logger(
              "rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }
      using ServiceResponseFuture =
        rclcpp::Client<mg400_msgs::srv::DisableRobot>::SharedFuture;
      auto response_received_callback = [this](ServiceResponseFuture future) {
          auto result = future.get();
          RCLCPP_INFO(this->get_logger(), "Result: %d", result->res);
        };
      auto future_result = disable_robot_clnt_->async_send_request(
        request,
        response_received_callback);
      this->robot_state = 1;
      sleep(1);
    }
  }
}

void JoyComponent::joyMoveJog()
{
  auto request = std::make_shared<mg400_msgs::srv::MoveJog::Request>();
  double lx = this->button_->lstick_x;
  double ly = this->button_->lstick_y;
  double rx = this->button_->rstick_x;
  double ry = this->button_->rstick_y;
  std::vector<double> input = {lx, fabs(lx), ly, fabs(ly), rx, fabs(rx), ry, fabs(ry)};
  std::vector<double>::iterator iter = std::max_element(input.begin(), input.end());
  if (*iter < 0.05) {
    if (this->robot_state != 3) {
      return;
    } else {
      request->axis_id = "";
      this->robot_state = 2;
    }
  } else {
    size_t index = std::distance(input.begin(), iter);
    std::vector<std::string> axis_vec = {"j1-", "j1+", "j2-", "j2+", "j4-", "j4+", "j3-", "j3+"};
    request->axis_id = axis_vec[index];
    this->robot_state = 3;
  }
  using namespace std::literals::chrono_literals;
  while (!move_jog_clnt_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
  using ServiceResponseFuture =
    rclcpp::Client<mg400_msgs::srv::MoveJog>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      auto result = future.get();
      RCLCPP_INFO(this->get_logger(), "Result: %d", result->res);
    };
  auto future_result = move_jog_clnt_->async_send_request(request, response_received_callback);
}

void JoyComponent::displayInfo() const noexcept
{
  auto tmp = std::system("clear");
  (void)tmp;  // for unused warnings;

  auto show_bool_state = [](
    const std::string & btn_name, const bool state) -> void {
      std::ostringstream ss;
      ss << std::left << std::setw(10) << btn_name + ":";
      ss << std::right << std::setw(10) << state;
      std::cout << ss.str() << std::endl;
    };
  auto show_float_state = [](
    const std::string & btn_name, const float state) -> void {
      std::ostringstream ss;
      ss << std::left << std::setw(10) << btn_name + ":";
      ss << std::right << std::setw(10) <<
        std::fixed << std::setprecision(3) << state;
      std::cout << ss.str() << std::endl;
    };
  auto show_bool_float_state = [](
    const std::string & btn_name,
    const bool b_state, const float f_state) -> void {
      std::ostringstream ss;
      ss << std::left << std::setw(10) << btn_name + ":";
      ss << std::right << std::setw(10) << b_state;
      ss << std::right << std::setw(10) <<
        std::fixed << std::setprecision(3) << f_state;
      std::cout << ss.str() << std::endl;
    };

  std::cout << std::endl;
  show_bool_state("cross", this->button_->cross);
  show_bool_state("triangle", this->button_->triangle);
  show_bool_state("circle", this->button_->circle);
  show_bool_state("square", this->button_->square);

  show_bool_state("L1", this->button_->L1);
  show_bool_state("R1", this->button_->R1);

  show_bool_float_state("L2", this->button_->L2, this->button_->L2_analog);
  show_bool_float_state("R2", this->button_->R2, this->button_->R2_analog);

  show_bool_state("select", this->button_->select);
  show_bool_state("start", this->button_->start);
  show_bool_state("PS", this->button_->PS);

  show_float_state("LStick_x", this->button_->lstick_x);
  show_float_state("LStick_y", this->button_->lstick_y);
  show_float_state("RStick_x", this->button_->rstick_x);
  show_float_state("RStick_y", this->button_->rstick_y);

  show_float_state("DPad_x", this->button_->d_pad_x);
  show_float_state("Dpad_y", this->button_->d_pad_y);
}

/**
 * @brief update menber 'button_' by useing received joy_msg
 *
 * @param ref_joy_msg reference to joy message from joyCallback
 */
void JoyComponent::updateButton(
  std::reference_wrapper<const sensor_msgs::msg::Joy::UniquePtr> ref_joy_msg)
{

  switch (this->joy_type) {
    case JOY_TYPE::DUALSHOCK3:
      {
        throw std::runtime_error("Not implemented yet.");
        break;
      }
    case JOY_TYPE::DUALSHOCK4:
      {
        throw std::runtime_error("Not implemented yet.");
        break;
      }
    case JOY_TYPE::DUALSENSE:
      {
        using BUTTONS = BUTTONS_DUALSENSE;
        using AXES = AXES_DUALSENSE;

        this->button_->cross =
          ref_joy_msg.get()->buttons.at(static_cast<size_t>(BUTTONS::CROSS));
        this->button_->circle =
          ref_joy_msg.get()->buttons.at(static_cast<size_t>(BUTTONS::CIRCLE));
        this->button_->triangle =
          ref_joy_msg.get()->buttons.at(static_cast<size_t>(BUTTONS::TRIANGLE));
        this->button_->square =
          ref_joy_msg.get()->buttons.at(static_cast<size_t>(BUTTONS::SQUARE));

        this->button_->L1 =
          ref_joy_msg.get()->buttons.at(static_cast<size_t>(BUTTONS::L1));
        this->button_->R1 =
          ref_joy_msg.get()->buttons.at(static_cast<size_t>(BUTTONS::R1));
        this->button_->L2 =
          ref_joy_msg.get()->buttons.at(static_cast<size_t>(BUTTONS::L2));
        this->button_->R2 =
          ref_joy_msg.get()->buttons.at(static_cast<size_t>(BUTTONS::R2));

        this->button_->select =
          ref_joy_msg.get()->buttons.at(static_cast<size_t>(BUTTONS::SELECT));
        this->button_->start =
          ref_joy_msg.get()->buttons.at(static_cast<size_t>(BUTTONS::START));
        this->button_->PS =
          ref_joy_msg.get()->buttons.at(static_cast<size_t>(BUTTONS::PS));

        this->button_->lstick_x =
          ref_joy_msg.get()->axes.at(static_cast<size_t>(AXES::LSTICK_X));
        this->button_->lstick_y =
          ref_joy_msg.get()->axes.at(static_cast<size_t>(AXES::LSTICK_Y));
        this->button_->rstick_x =
          ref_joy_msg.get()->axes.at(static_cast<size_t>(AXES::RSTICK_X));
        this->button_->rstick_y =
          ref_joy_msg.get()->axes.at(static_cast<size_t>(AXES::RSTICK_Y));

        this->button_->L2_analog =
          ref_joy_msg.get()->axes.at(static_cast<size_t>(AXES::L2));
        this->button_->R2_analog =
          ref_joy_msg.get()->axes.at(static_cast<size_t>(AXES::R2));

        this->button_->d_pad_x =
          ref_joy_msg.get()->axes.at(static_cast<size_t>(AXES::DPAD_X));
        this->button_->d_pad_y =
          ref_joy_msg.get()->axes.at(static_cast<size_t>(AXES::DPAD_Y));

        break;
      }
    default:
      throw std::runtime_error("Invalid JOY_TYPE given");
  }
}

}  // namespace mg400_joy
