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

#ifndef __MG400_PLUGIN_BASE_API_LOADER_BASE_HPP__
#define __MG400_PLUGIN_BASE_API_LOADER_BASE_HPP__

#include <string>
#include <memory>
#include <unordered_map>
#include <vector>

#include <pluginlib/class_loader.hpp>

#include "mg400_plugin_base/api_plugin_base.hpp"


namespace mg400_plugin_base
{

template<typename PluginT>
class ApiLoaderBase
{
public:
  using SharedPtr = std::shared_ptr<ApiLoaderBase<PluginT>>;

protected:
  const std::string plugin_basename_;
  std::unique_ptr<pluginlib::ClassLoader<PluginT>> loader_;
  std::unordered_map<std::string, typename PluginT::SharedPtr> plugin_map_;

public:
  ApiLoaderBase() = delete;
  explicit ApiLoaderBase(const std::string && plugin_basename)
  : plugin_basename_(plugin_basename)
  {
    this->loader_ =
      std::make_unique<pluginlib::ClassLoader<PluginT>>(
      "mg400_plugin_base",
      "mg400_plugin_base::" + this->plugin_basename_);
  }

  void loadPlugins(const std::vector<std::string> & plugins)
  {
    for (const auto & plugin : plugins) {
      this->plugin_map_.insert(
        std::make_pair(
          plugin,
          this->loader_->createSharedInstance(plugin)));
    }
  }

  void configure(
    typename PluginT::CommanderT::SharedPtr commander,
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_if,
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_if,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_if,
    const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_if,
    const rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitable_if,
    mg400_interface::MG400Interface::SharedPtr mg400_if)
  {
    for (const auto & it : this->plugin_map_) {
      it.second->configure(
        commander, node_base_if, node_clock_if,
        node_logging_if, node_services_if, node_waitable_if, mg400_if);
    }
  }

  void showPluginInfo(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr &
    logging_interface) const
  {
    std::stringstream ss;
    ss << "Loading [" << this->plugin_basename_ << "] Plugins..." << std::endl;
    for (const auto & it : this->plugin_map_) {
      ss << "  [" << it.first << "]: " <<
        this->loader_->getClassDescription(it.first) << std::endl;
    }
    static const char ANSI_COLOR_BLUE[] = "\x1b[34m";
    static const char ANSI_COLOR_RESET[] = "\x1b[0m";
    RCLCPP_INFO(
      logging_interface->get_logger(),
      "%s%s%s", ANSI_COLOR_BLUE, ss.str().c_str(), ANSI_COLOR_RESET);
  }
};

class DashboardApiLoader
  : public ApiLoaderBase<DashboardApiPluginBase>
{
public:
  DashboardApiLoader()
  : ApiLoaderBase<DashboardApiPluginBase>(
      "DashboardApiPluginBase")
  {}
};
class MotionApiLoader
  : public ApiLoaderBase<MotionApiPluginBase>
{
public:
  MotionApiLoader()
  : ApiLoaderBase<MotionApiPluginBase>(
      "MotionApiPluginBase")
  {}
};
}  // namespace mg400_plugin_base
#endif
