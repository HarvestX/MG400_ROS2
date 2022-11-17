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

#include "mg400_node/mg400_node.hpp"


namespace mg400_node
{
MG400Node::MG400Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("mg400_node", options)
{

  const std::string ip_address =
    this->declare_parameter<std::string>("ip_address", "192.168.1.6");
  RCLCPP_INFO(
    this->get_logger(), "Connecting to %s ...", ip_address.c_str());

  this->interface_ =
    std::make_unique<mg400_interface::MG400Interface>(ip_address);

  if (!this->interface_->configure()) {
    rclcpp::shutdown();
    return;
  }

  if (!this->interface_->activate()) {
    rclcpp::shutdown();
    return;
  }

  this->class_loader_ =
    std::make_unique<
    pluginlib::ClassLoader<mg400_plugin_base::DashboardApiBase>>(
    "mg400_plugin_base", "mg400_plugin_base::DashboardApiBase");


  using namespace std::chrono_literals;  // NOLINT
  this->init_timer_ = this->create_wall_timer(
    0s,
    [&]() {
      std::vector<std::string> plugins = {
        "mg400_plugin::ClearError"
      };

      for (const auto & plugin : plugins) {
        this->dashboard_api_plugin_map_.insert(
          std::make_pair(
            plugin, this->class_loader_->createSharedInstance(plugin)));

        this->dashboard_api_plugin_map_.at(plugin)->configure(
          this->interface_->dashboard_commander,
          this->shared_from_this());
      }
    });

}

}  // namespace mg400_node
