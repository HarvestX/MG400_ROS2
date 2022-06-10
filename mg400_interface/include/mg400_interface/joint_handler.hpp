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

#include <memory>
#include <string>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mg400_interface
{
const char BASE_LINK_NAME[] = "mg400_base_link";
const char J1_NAME[] = "mg400_j1";
const char J2_1_NAME[] = "mg400_j2_1";
const char J2_2_NAME[] = "mg400_j2_2";
const char J3_1_NAME[] = "mg400_j3_1";
const char J3_2_NAME[] = "mg400_j3_2";
const char J4_1_NAME[] = "mg400_j4_1";
const char J4_2_NAME[] = "mg400_j4_2";
const char J5_NAME[] = "mg400_j5";

std::unique_ptr<sensor_msgs::msg::JointState> getJointState(
  const double &, const double &, const double &, const double &,
  const std::string &);
}  // namespace mg400_interface
