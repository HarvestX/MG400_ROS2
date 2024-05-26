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

#ifndef __MG400_PLUGIN_DASHBOARD_API_GET_POSE_HPP__
#define __MG400_PLUGIN_DASHBOARD_API_GET_POSE_HPP__

#include <mg400_plugin_base/api_plugin_base.hpp>
#include <mg400_msgs/srv/get_pose.hpp>

namespace mg400_plugin
{
class GetPose final : public mg400_plugin_base::DashboardApiPluginBase
{
public:
  using ServiceT = mg400_msgs::srv::GetPose;
  using CallbackT = std::function<
    void (
      const typename ServiceT::Request::SharedPtr,
      typename ServiceT::Response::SharedPtr
    )>;

private:
  rclcpp::Service<ServiceT>::SharedPtr srv_;

public:
  void configure(
    const mg400_interface::DashboardCommander::SharedPtr,
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr,
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr,
    const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr,
    const rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr,
    const mg400_interface::MG400Interface::SharedPtr) override;

private:
  void onServiceCall(
    const ServiceT::Request::SharedPtr, ServiceT::Response::SharedPtr);
};
}  // mg400_plugin
#endif
