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

#include <eigen3/Eigen/Core>

#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mg400_common/mg400_ik_util.hpp>
#include "mg400_interface/command_utils.hpp"

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


class JointHandler
{
private:
  using JointState = sensor_msgs::msg::JointState;
  using Pose = geometry_msgs::msg::Pose;

public:
  static JointState::UniquePtr getJointState(const std::array<double, 4> &, const std::string &);

  static JointState::UniquePtr getJointState(
    const double &, const double &, const double &, const double &, const std::string &);


  static bool getEndPose(const std::array<double, 4> &, Pose &);

  static bool getEndPose(const JointState::ConstSharedPtr, Pose &);

private:
  static Eigen::MatrixXd rotY(const Eigen::MatrixXd &, const double &);
  static Eigen::MatrixXd rotZ(const Eigen::MatrixXd &, const double &);
};
}  // namespace mg400_interface
