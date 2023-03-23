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

#include "mg400_plugin/dashboard_api/di.hpp"

namespace mg400_plugin
{

void DI::configure(
  const mg400_interface::DashboardCommander::SharedPtr commander,
  const rclcpp::Node::SharedPtr node,
  const mg400_interface::RealtimeFeedbackTcpInterface::SharedPtr rt_if)
{
  if (!this->configure_base(commander, node, rt_if)) {
    return;
  }

  using namespace std::placeholders;  // NOLINT
  this->srv_ = node->create_service<ServiceT>(
    "di",
    std::bind(&DI::onServiceCall, this, _1, _2));
}

void DI::onServiceCall(
  const ServiceT::Request::SharedPtr req,
  ServiceT::Response::SharedPtr res)
{
  try {
    res->result = this->commander_->DI(req->index);
  } catch (const std::runtime_error & ex) {
    RCLCPP_ERROR(this->base_node_->get_logger(), ex.what());
  } catch (...) {
    RCLCPP_ERROR(this->base_node_->get_logger(), "Interface Error");
  }
}
}  // namespace mg400_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  mg400_plugin::DI,
  mg400_plugin_base::DashboardApiPluginBase)
