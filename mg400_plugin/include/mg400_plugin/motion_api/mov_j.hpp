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

#include <mg400_plugin_base/api_plugin_base.hpp>
#include <mg400_msgs/action/mov_j.hpp>

namespace mg400_plugin
{
class MovJ final : public mg400_plugin_base::MotionApiPluginBase
{
public:
  using ActionT = mg400_msgs::action::MovJ;

public:
  void configure(
    const mg400_interface::MotionCommander::SharedPtr,
    const rclcpp::Node::SharedPtr,
    const mg400_interface::RealtimeFeedbackTcpInterface::SharedPtr)
  override;

private:
};
}  // namespace mg400_plugin
