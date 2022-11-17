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

  using namespace std::chrono_literals;  // NOLINT
  this->init_timer_ = this->create_wall_timer(
    0s,
    [&]() {
      pluginlib::ClassLoader<mg400_plugin_base::DashboardApiBase>
      class_loader(
        "mg400_plugin_base",
        "mg400_plugin_base::DashboardApiBase");

      auto shared_instance = class_loader.createSharedInstance(
        "mg400_plugin::ClearError");
      shared_instance->configure(this->shared_from_this());
    });

}

}  // namespace mg400_node
