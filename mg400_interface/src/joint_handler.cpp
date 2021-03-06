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

#include "mg400_interface/joint_handler.hpp"


namespace mg400_interface
{

std::unique_ptr<sensor_msgs::msg::JointState> getJointState(
  const double & j1, const double & j2, const double & j3, const double & j4,
  const std::string & prefix)
{
  auto msg = std::make_unique<sensor_msgs::msg::JointState>();
  msg->header.stamp = rclcpp::Clock().now();
  msg->header.frame_id = prefix + BASE_LINK_NAME;

  msg->name = {
    prefix + J1_NAME,
    prefix + J2_1_NAME,
    prefix + J2_2_NAME,
    prefix + J3_1_NAME,
    prefix + J3_2_NAME,
    prefix + J4_1_NAME,
    prefix + J4_2_NAME,
    prefix + J5_NAME
  };

  msg->position = {
    j1,       // j1
    j2,       // j2_1
    j2,       // j2_2
    j3 - j2,  // j3_1
    -j2,      // j3_2
    -j3,      // j4_1
    j3,       // j4_2
    j4        // j5
  };

  return msg;
}


}  // namespace mg400_interface
