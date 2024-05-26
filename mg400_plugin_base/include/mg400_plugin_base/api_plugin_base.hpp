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

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_services_interface.hpp>
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
};
}  // namespace mg400_plugin_base
#endif
