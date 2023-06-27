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

#include "mg400_plugin/dashboard_api/payload.hpp"

namespace mg400_plugin
{

void Payload::configure(
  const mg400_interface::DashboardCommander::SharedPtr commander,
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_if,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_log_if,
  const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_srv_if,
  const mg400_interface::MG400Interface::SharedPtr mg400_if)
{
  if (!this->configure_base(commander, node_base_if, node_log_if, node_srv_if, mg400_if)) {
    return;
  }

  using namespace std::placeholders;  // NOLINT
  this->srv_ = rclcpp::create_service<ServiceT, CallbackT>(
    this->node_base_,
    this->node_services_,
    "payload",
    std::bind(&Payload::onServiceCall, this, _1, _2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(),
    this->node_base_->get_default_callback_group());
}

void Payload::onServiceCall(
  const ServiceT::Request::SharedPtr req,
  ServiceT::Response::SharedPtr res)
{
  res->result = false;
  if (this->mg400_interface_->ok()) {
    try {
      this->commander_->payload(req->weight, req->inertia);
      res->result = true;
    } catch (const std::runtime_error & ex) {
      RCLCPP_ERROR(this->node_logging_->get_logger(), ex.what());
    } catch (...) {
      RCLCPP_ERROR(this->node_logging_->get_logger(), "Interface Error");
    }
  } else {
    RCLCPP_ERROR(this->node_logging_->get_logger(), "MG400 is not connected");
  }
}
}  // namespace mg400_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  mg400_plugin::Payload,
  mg400_plugin_base::DashboardApiPluginBase)