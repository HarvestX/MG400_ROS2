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

#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <mg400_plugin_base/dashboard_api_base.hpp>
#include <pluginlib/class_loader.hpp>

namespace mg400_node
{
class MG400Node : public rclcpp::Node
{
private:
  std::unique_ptr<
    pluginlib::ClassLoader<mg400_plugin_base::DashboardApiBase>> class_loader_;

  rclcpp::TimerBase::SharedPtr init_timer_;
  mg400_interface::MG400Interface::UniquePtr interface_;

  std::unordered_map<
    std::string,
    mg400_plugin_base::DashboardApiBase::SharedPtr>
  dashboard_api_plugin_map_;

public:
  MG400Node() = delete;
  explicit MG400Node(const rclcpp::NodeOptions &);

};
}  // namespace mg400_node


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mg400_node::MG400Node)
