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

  this->declare_parameter<std::vector<std::string>>(
    "dashboard_api_plugins", this->default_dashboard_api_plugins_);
  this->declare_parameter<std::vector<std::string>>(
    "motion_api_plugins", this->default_motion_api_plugins_);


  this->interface_ =
    std::make_unique<mg400_interface::MG400Interface>(ip_address);

  this->declare_parameter<std::string>("prefix", "");
  if (!this->interface_->configure(this->get_parameter("prefix").as_string())) {
    rclcpp::shutdown();
    return;
  }

  if (!this->interface_->activate()) {
    rclcpp::shutdown();
    return;
  }

  this->dashboard_api_loader_ =
    std::make_shared<mg400_plugin_base::DashboardApiLoader>();
  this->dashboard_api_loader_->loadPlugins(
    this->get_parameter("dashboard_api_plugins").as_string_array());

  this->motion_api_loader_ =
    std::make_shared<mg400_plugin_base::MotionApiLoader>();
  this->motion_api_loader_->loadPlugins(
    this->get_parameter("motion_api_plugins").as_string_array());


  using namespace std::chrono_literals;   // NOLINT
  this->init_timer_ = this->create_wall_timer(
    0s,
    [&]() {
      this->init_timer_->cancel();
      this->init();
      this->dashboard_api_loader_->showPluginInfo(
        this->get_node_logging_interface());
      this->motion_api_loader_->showPluginInfo(
        this->get_node_logging_interface());
    });
}

MG400Node::~MG400Node()
{
  if (this->interface_) {
    this->interface_->deactivate();
  }
}

void MG400Node::init()
{
  this->dashboard_api_loader_->configure(
    this->interface_->dashboard_commander,
    this->shared_from_this());

  this->motion_api_loader_->configure(
    this->interface_->motion_commander,
    this->shared_from_this(),
    this->interface_->realtime_tcp_interface);
}

}  // namespace mg400_node
