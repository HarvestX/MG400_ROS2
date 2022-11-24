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

#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
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
  rclcpp::Node::SharedPtr base_node_;
  mg400_interface::RealtimeFeedbackTcpInterface::SharedPtr
    realtime_tcp_interface_;

public:
  ApiPluginBase() {}
  virtual ~ApiPluginBase() {}
  virtual void configure(
    const typename CommanderT::SharedPtr,
    const rclcpp::Node::SharedPtr,
    const mg400_interface::RealtimeFeedbackTcpInterface::SharedPtr) = 0;

protected:
  bool configure_base(
    const typename CommanderT::SharedPtr commander,
    const rclcpp::Node::SharedPtr node,
    const mg400_interface::RealtimeFeedbackTcpInterface::SharedPtr
    rt_if = nullptr)
  {
    if (this->base_node_) {
      RCLCPP_WARN(
        this->base_node_->get_logger(),
        "Plugin already configured.");
      return false;
    }

    this->commander_ = commander;
    this->base_node_ = node;

    if (rt_if) {
      this->realtime_tcp_interface_ = rt_if;
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
