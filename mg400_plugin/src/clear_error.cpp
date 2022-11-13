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

#include "mg400_plugin/clear_error.hpp"


namespace mg400_plugin
{

void ClearError::configure(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services)
{

  this->srv_ = rclcpp::create_service<ServiceT>(
    node_base,
    node_services,
    "clear_error",
    std::bind(
      &ClearError::onServiceCall, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, nullptr);
}

void ClearError::onServiceCall(
  const ServiceT::Request::SharedPtr,
  ServiceT::Response::SharedPtr
)
{

}
}  // namespace mg400_plugin
