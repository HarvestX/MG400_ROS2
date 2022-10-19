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

geometry_msgs::msg::Pose getEndPoint(
  const sensor_msgs::msg::JointState & joint_state)
{
  geometry_msgs::msg::Pose msg;
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
    rotY(LINK2, joint_state.position[1]) +
    rotY(LINK3, joint_state.position[6]) +
    LINK4;
  p = rotZ(pos, joint_state.position[0]);

  msg.position.x = static_cast<double>(p(0, 0));
  msg.position.y = static_cast<double>(p(1, 0));
  msg.position.z = static_cast<double>(p(2, 0));

  return msg;
}

Eigen::MatrixXd rotY(
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

Eigen::MatrixXd rotZ(
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
