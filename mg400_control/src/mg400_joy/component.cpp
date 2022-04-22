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

  RCLCPP_INFO(
    this->get_logger(),
    "Ready.");
}

void JoyComponent::joyCallback(const sensor_msgs::msg::Joy::UniquePtr joy_msg)
{
  this->updateButton(joy_msg);

  this->displayInfo();

  // TODO: Call jog service here
  // You can access the state of each button as follows
  // this->button_->circle;
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
