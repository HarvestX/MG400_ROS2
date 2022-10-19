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

  LINK1(0, 0) = 43;
  LINK1(1, 0) = 0;
  LINK1(2, 0) = 0;
  LINK2(0, 0) = 0;
  LINK2(1, 0) = 0;
  LINK2(2, 0) = 175;
  LINK3(0, 0) = 175;
  LINK3(1, 0) = 0;
  LINK3(2, 0) = 0;
  LINK4(0, 0) = 66;
  LINK4(1, 0) = 0;
  LINK4(2, 0) = -57;

  pos = LINK1 + rot_y(LINK2, joint_state.position[1]) + \
    rot_y(LINK3, joint_state.position[6]) + LINK4;
  p = rot_z(pos, joint_state.position[0]);

  msg.position.x = static_cast<double>(p(0, 0));
  msg.position.y = static_cast<double>(p(1, 0));
  msg.position.z = static_cast<double>(p(2, 0));

  return msg;
}

Eigen::MatrixXd rot_y(
  const Eigen::MatrixXd & vec, const double & angle)
{
  Eigen::Matrix3d rot_mtrx;

  rot_mtrx(0, 0) = cos(angle);
  rot_mtrx(0, 1) = 0;
  rot_mtrx(0, 2) = sin(angle);
  rot_mtrx(1, 0) = 0;
  rot_mtrx(1, 1) = 1;
  rot_mtrx(1, 2) = 0;
  rot_mtrx(2, 0) = -sin(angle);
  rot_mtrx(2, 1) = 0;
  rot_mtrx(2, 2) = cos(angle);

  return rot_mtrx * vec;
}

Eigen::MatrixXd rot_z(
  const Eigen::MatrixXd & vec, const double & angle)
{
  Eigen::Matrix3d rot_mtrx;

  rot_mtrx(0, 0) = cos(angle);
  rot_mtrx(0, 1) = -sin(angle);
  rot_mtrx(0, 2) = 0;
  rot_mtrx(1, 0) = sin(angle);
  rot_mtrx(1, 1) = cos(angle);
  rot_mtrx(1, 2) = 0;
  rot_mtrx(2, 0) = 0;
  rot_mtrx(2, 1) = 0;
  rot_mtrx(2, 2) = 1;

  return rot_mtrx * vec;
}
}  // namespace mg400_interface
