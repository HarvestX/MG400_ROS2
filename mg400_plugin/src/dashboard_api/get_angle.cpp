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

#include "mg400_plugin/dashboard_api/get_angle.hpp"

namespace mg400_plugin
{

void GetAngle::configure(
  const mg400_interface::DashboardCommander::SharedPtr commander,
  const rclcpp::Node::SharedPtr node,
  const mg400_interface::MG400Interface::SharedPtr mg400_if)
{
  if (!this->configure_base(commander, node, mg400_if)) {
    return;
  }

  using namespace std::placeholders;  // NOLINT
  this->srv_ = node->create_service<ServiceT>(
    "get_angle",
    std::bind(&GetAngle::onServiceCall, this, _1, _2));
}

void GetAngle::onServiceCall(
  const ServiceT::Request::SharedPtr,
  ServiceT::Response::SharedPtr res)
{
  if (this->mg400_interface_->ok()) {
    try {
      auto joints = this->commander_->getAngle();
      res->joint1 = joints[0];
      res->joint2 = joints[1];
      res->joint3 = joints[2];
      res->joint4 = joints[3];
      res->joint5 = joints[4];
      res->joint6 = joints[5];
    } catch (const std::runtime_error & ex) {
      RCLCPP_ERROR(this->base_node_->get_logger(), ex.what());
    } catch (...) {
      RCLCPP_ERROR(this->base_node_->get_logger(), "Interface Error");
    }
  } else {
    RCLCPP_ERROR(this->base_node_->get_logger(), "MG400 is not connected");
  }
}
}  // namespace mg400_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  mg400_plugin::GetAngle,
  mg400_plugin_base::DashboardApiPluginBase)
