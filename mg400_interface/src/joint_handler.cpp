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

#include <mg400_common/kinematics.hpp>

namespace mg400_interface
{
JointHandler::JointState::UniquePtr
JointHandler::getJointState(const std::array<double, 4> & joint_states, const std::string & prefix)
{
  auto msg = std::make_unique<JointState>();
  msg->header.stamp = rclcpp::Clock().now();
  msg->header.frame_id = prefix + BASE_LINK_NAME;

  msg->name = {
    prefix + J1_NAME,
    prefix + J2_1_NAME,
    prefix + J3_1_NAME,
    prefix + J5_NAME
  };

  msg->position = {
    joint_states[0],                    // j1
    joint_states[1],                    // j2_1
    joint_states[2] - joint_states[1],  // j3_1
    joint_states[3]                     // j5
  };

  return msg;
}

JointHandler::JointState::UniquePtr JointHandler::getJointState(
  const double & j1, const double & j2, const double & j3, const double & j4,
  const std::string & prefix)
{
  return JointHandler::getJointState({j1, j2, j3, j4}, prefix);
}

bool JointHandler::getEndPose(const std::array<double, 4> & joints, Pose & pose)
{
  const Eigen::Vector3d pos =
    mg400_common::kinematics::LINK1 +
    rotY(mg400_common::kinematics::LINK2, joints.at(1)) +
    rotY(mg400_common::kinematics::LINK3, joints.at(2)) +
    mg400_common::kinematics::LINK4;
  const Eigen::Vector3d p = rotZ(pos, joints.at(0));

  pose.position.x = p.x();
  pose.position.y = p.y();
  pose.position.z = p.z();

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, joints.at(0) + joints.at(3));

  pose.orientation.w = quat.getW();
  pose.orientation.x = quat.getX();
  pose.orientation.y = quat.getY();
  pose.orientation.z = quat.getZ();

  return true;
}

bool JointHandler::getEndPose(const JointState::ConstSharedPtr joint_state, Pose & pose)
{
  if (!joint_state) {
    return false;
  }

  if (joint_state->position.size() != 8) {
    return false;
  }

  return JointHandler::getEndPose(
    {joint_state->position.at(0), joint_state->position.at(1),
      joint_state->position.at(6), joint_state->position.at(7)}, pose);
}


Eigen::Vector3d JointHandler::rotY(const Eigen::Vector3d & vec, const double & angle)
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

Eigen::Vector3d JointHandler::rotZ(const Eigen::Vector3d & vec, const double & angle)
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
