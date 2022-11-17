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
class DashboardApiBase
{
public:
  using SharedPtr = std::shared_ptr<DashboardApiBase>;

protected:
  mg400_interface::DashboardCommander::SharedPtr commander_;
  rclcpp::Node::SharedPtr base_node_;

public:
  DashboardApiBase() {}
  virtual ~DashboardApiBase() {}

  bool confiture_base(
    const mg400_interface::DashboardCommander::SharedPtr commander,
    const rclcpp::Node::SharedPtr node
  )
  {
    if (this->base_node_) {
      RCLCPP_WARN(
        this->base_node_->get_logger(),
        "Plugin already configured.");
      return false;
    }

    this->commander_ = commander;
    this->base_node_ = node;
    return true;
  }

  virtual void configure(
    const mg400_interface::DashboardCommander::SharedPtr,
    const rclcpp::Node::SharedPtr) = 0;
};
}  // namespace mg400_plugin_base
