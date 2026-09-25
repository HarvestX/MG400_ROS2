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

#ifndef __MG400_PLUGIN_BASE_API_PLUGIN_BASE_HPP__
#define __MG400_PLUGIN_BASE_API_PLUGIN_BASE_HPP__

#include <cinttypes>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_services_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_timers_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include <mg400_msgs/msg/robot_mode.hpp>
#include <mg400_interface/mg400_interface.hpp>

namespace mg400_plugin_base
{

template<typename CommanderType>
class ApiPluginBase
{
public:
  typedef CommanderType CommanderT;
  using SharedPtr = std::shared_ptr<ApiPluginBase<CommanderT>>;

protected:
  typename CommanderT::SharedPtr commander_;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_if_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_if_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_if_;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_if_;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitable_if_;
  mg400_interface::MG400Interface::SharedPtr
    mg400_interface_;

public:
  ApiPluginBase() {}
  virtual ~ApiPluginBase() {}
  virtual void configure(
    const typename CommanderT::SharedPtr,
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr,
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr,
    const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr,
    const rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr,
    const mg400_interface::MG400Interface::SharedPtr) = 0;

protected:
  bool configure_base(
    const typename CommanderT::SharedPtr commander,
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_if,
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_if,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_if,
    const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_if,
    const rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitable_if,
    const mg400_interface::MG400Interface::SharedPtr mg400_if)
  {
    if (this->node_base_if_ && this->node_clock_if_ &&
      this->node_logging_if_ && this->node_services_if_ && this->node_waitable_if_)
    {
      RCLCPP_WARN(
        this->node_logging_if_->get_logger(),
        "Plugin already configured.");
      return false;
    }

    this->commander_ = commander;
    this->node_base_if_ = node_base_if;
    this->node_clock_if_ = node_clock_if;
    this->node_logging_if_ = node_logging_if;
    this->node_services_if_ = node_services_if;
    this->node_waitable_if_ = node_waitable_if;

    if (mg400_if) {
      this->mg400_interface_ = mg400_if;
    }

    return true;
  }

};

class DashboardApiPluginBase
  : public ApiPluginBase<mg400_interface::DashboardCommander>
{
};

class MotionApiPluginBase
  : public ApiPluginBase<mg400_interface::MotionCommander>
{
public:
  using SharedPtr = std::shared_ptr<MotionApiPluginBase>;

  /// Supply node interfaces needed by streaming plugins before configure().
  void setNodeResources(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters,
    const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics,
    const rclcpp::node_interfaces::NodeTimersInterface::SharedPtr timers)
  {
    node_parameters_if_ = parameters;
    node_topics_if_ = topics;
    node_timers_if_ = timers;
  }

  /// Called after the MG400 connection becomes active.
  virtual void activate() {}

  /// Called before the MG400 connection is closed.
  virtual void deactivate() {}

  /// Select state rules for starting a motion command or stopping an active jog.
  enum class CommandPolicy
  {
    STANDARD,
    JOG_STOP,
  };

  /// Return whether a coherent robot-state snapshot permits the requested command.
  static bool isMotionStateReady(
    const mg400_interface::RobotStateMachine::Snapshot & snapshot,
    const CommandPolicy policy = CommandPolicy::STANDARD) noexcept
  {
    using RobotMode = mg400_msgs::msg::RobotMode;
    using State = mg400_interface::RobotStateMachine::State;

    if (!snapshot.feedback_fresh) {
      return false;
    }
    if (snapshot.state == State::ENABLED) {
      return true;
    }
    return policy == CommandPolicy::JOG_STOP &&
           snapshot.state == State::PAUSED_OR_JOG &&
           snapshot.raw_robot_mode == RobotMode::JOG;
  }

protected:
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_if_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_if_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_if_;

  /// Validate connection and robot state, and log the rejection reason.
  bool isMotionCommandReady(
    const char * command_name,
    const CommandPolicy policy = CommandPolicy::STANDARD) const
  {
    if (!this->mg400_interface_ || !this->mg400_interface_->robot_state_machine) {
      RCLCPP_ERROR(
        this->node_logging_if_->get_logger(),
        "%s rejected: MG400 interface is not configured", command_name);
      return false;
    }

    const auto snapshot = this->mg400_interface_->robot_state_machine->getSnapshot();
    const bool interface_ok = this->mg400_interface_->ok();
    if (interface_ok && isMotionStateReady(snapshot, policy)) {
      return true;
    }

    RCLCPP_ERROR(
      this->node_logging_if_->get_logger(),
      "%s rejected: interface_ok=%s, state=%u, raw_robot_mode=%" PRIu64
      ", feedback_fresh=%s",
      command_name, interface_ok ? "true" : "false",
      static_cast<unsigned int>(snapshot.state),
      snapshot.raw_robot_mode, snapshot.feedback_fresh ? "true" : "false");
    return false;
  }
};
}  // namespace mg400_plugin_base
#endif
