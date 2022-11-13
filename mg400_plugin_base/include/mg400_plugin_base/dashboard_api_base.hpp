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

#include <rclcpp/rclcpp.hpp>

namespace mg400_plugin_base
{
template<typename ServiceType>
class DashboardApiBase
{
protected:
  typedef ServiceType ServiceT;

public:
  DashboardApiBase() {}
  virtual ~DashboardApiBase() {}

protected:
  virtual void configure(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr,
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr) = 0;

  virtual void onServiceCall(
    const typename ServiceType::Request::SharedPtr,
    typename ServiceType::Response::SharedPtr) = 0;
};
}  // namespace mg400_plugin_base
