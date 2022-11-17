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

#include "mg400_plugin/dashboard_api/clear_error.hpp"


namespace mg400_plugin
{

void ClearError::configure(
  const rclcpp::Node::SharedPtr node
)
{
  using namespace std::placeholders;  // NOLINT
  this->srv_ = node->create_service<ServiceT>(
    "clear_error",
    std::bind(&ClearError::onServiceCall, this, _1, _2));
}

void ClearError::onServiceCall(
  const ServiceT::Request::SharedPtr req,
  ServiceT::Response::SharedPtr res)
{

}
}  // namespace mg400_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mg400_plugin::ClearError,
  mg400_plugin_base::DashboardApiBase)
