// Copyright 2025 HarvestX Inc.
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

#include "mg400_plugin/dashboard_api/get_error_id.hpp"

namespace mg400_plugin
{

void GetErrorID::configure(
  const mg400_interface::DashboardCommander::SharedPtr commander,
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_if,
  const rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_if,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_if,
  const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_if,
  const rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_if,
  const mg400_interface::MG400Interface::SharedPtr mg400_if)
{
  if (!this->configure_base(
      commander, node_base_if, node_clock_if,
      node_logging_if, node_services_if, node_waitables_if, mg400_if))
  {
    return;
  }

  using namespace std::placeholders;  // NOLINT
  this->srv_ = rclcpp::create_service<ServiceT, CallbackT>(
    this->node_base_if_,
    this->node_services_if_,
    "get_error_id",
    std::bind(&GetErrorID::onServiceCall, this, _1, _2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(),
    this->node_base_if_->get_default_callback_group());
}

void GetErrorID::onServiceCall(
  const ServiceT::Request::SharedPtr /*req*/,
  ServiceT::Response::SharedPtr res)
{
  res->error_id = -1;
  if (this->mg400_interface_->ok()) {
    try {
      const auto error_ids = this->commander_->getErrorId();
      this->commander_->convertToErrorIdMsg(error_ids, res->error_ids);
      res->error_id = 0;
    } catch (const mg400_interface::DashboardCommandException & ex) {
      RCLCPP_ERROR(this->node_logging_if_->get_logger(), ex.what());
      res->error_id = ex.getDashboardResponse().error_id;
    } catch (const std::runtime_error & ex) {
      RCLCPP_ERROR(this->node_logging_if_->get_logger(), ex.what());
    } catch (...) {
      RCLCPP_ERROR(this->node_logging_if_->get_logger(), "Interface Error");
    }
  } else {
    RCLCPP_ERROR(this->node_logging_if_->get_logger(), "MG400 is not connected");
  }
}
}  // namespace mg400_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  mg400_plugin::GetErrorID,
  mg400_plugin_base::DashboardApiPluginBase)
