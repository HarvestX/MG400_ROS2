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

sensor_msgs::msg::JointState::UniquePtr
JointHandler::getJointState(
  const std::array<double, 4> & joint_states,
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
    joint_states[0],                    // j1
    joint_states[1],                    // j2_1
    joint_states[1],                    // j2_2
    joint_states[2] - joint_states[1],  // j3_1
    -joint_states[1],                   // j3_2
    -joint_states[2],                   // j4_1
    joint_states[2],                    // j4_2
    joint_states[3]                     // j5
  };

  return msg;
}

sensor_msgs::msg::JointState::UniquePtr
JointHandler::getJointState(
  const double & j1, const double & j2, const double & j3, const double & j4,
  const std::string & prefix)
{
  return JointHandler::getJointState({j1, j2, j3, j4}, prefix);
}

bool JointHandler::getEndPose(
  const std::array<double, 4> & joints,
  mg400_msgs::msg::EndPose & pose,
  const bool && is_ref)
{
  Eigen::MatrixXd pos(3, 1);
  Eigen::MatrixXd p(3, 1);

  Eigen::MatrixXd LINK1(3, 1);
  Eigen::MatrixXd LINK2(3, 1);
  Eigen::MatrixXd LINK3(3, 1);
  Eigen::MatrixXd LINK4(3, 1);

  LINK1 << 0.043, 0.0, 0.0;
  LINK2 << 0.0, 0.0, 0.175;
  LINK3 << 0.175, 0.0, 0.0;
  LINK4 << 0.066, 0.0, -0.057;

  pos = LINK1 +
    rotY(LINK2, joints.at(1)) +
    rotY(LINK3, joints.at(2)) +
    LINK4;
  p = rotZ(pos, joints.at(0));

  pose.x = static_cast<double>(p(0, 0));
  pose.y = static_cast<double>(p(1, 0));
  pose.z = static_cast<double>(p(2, 0));
  if (is_ref) {
    pose.r =
      joints.at(0) + joints.at(3);
  } else {
    pose.r = joints.at(3);
  }

  return true;
}

bool JointHandler::getEndPose(
  const sensor_msgs::msg::JointState::ConstSharedPtr joint_state,
  mg400_msgs::msg::EndPose & pose, const bool && is_ref)
{
  if (!joint_state) {
    return false;
  }

  if (joint_state->position.size() != 8) {
    return false;
  }

  return JointHandler::getEndPose(
    {joint_state->position.at(0),
      joint_state->position.at(1),
      joint_state->position.at(6),
      joint_state->position.at(7)},
    pose, std::forward<const bool>(is_ref));
}


Eigen::MatrixXd JointHandler::rotY(
  const Eigen::MatrixXd & vec, const double & angle)
{
  Eigen::Matrix3d rot_mat;
  const double co = cos(angle);
  const double si = sin(angle);

  rot_mat <<
    co, 0.0, si,
    0.0, 1.0, 0.0,
    -si, 0.0, co;

  return rot_mat * vec;
}

Eigen::MatrixXd JointHandler::rotZ(
  const Eigen::MatrixXd & vec, const double & angle)
{
  Eigen::Matrix3d rot_mat;
  const double co = cos(angle);
  const double si = sin(angle);

  rot_mat <<
    co, -si, 0.0,
    si, co, 0.0,
    0.0, 0.0, 1.0;

  return rot_mat * vec;
}
}  // namespace mg400_interface
