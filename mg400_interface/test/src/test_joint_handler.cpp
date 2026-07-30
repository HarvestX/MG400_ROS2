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


#include <gtest/gtest.h>
#include <mg400_common/kinematics.hpp>
#include <mg400_interface/mg400_interface.hpp>


class TestJointHandler : public ::testing::Test
{
protected:
  void SetUp() override {}

  void TearDown() override {}
};


TEST_F(TestJointHandler, getEndPoint)
{
  auto js = std::make_shared<sensor_msgs::msg::JointState>();
  js->position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  geometry_msgs::msg::Pose actual;
  const auto ret = mg400_interface::JointHandler::getEndPose(js, actual);
  ASSERT_TRUE(ret);
  const Eigen::Vector3d expected_position =
    mg400_common::kinematics::LINK1 +
    mg400_common::kinematics::LINK2 +
    mg400_common::kinematics::LINK3 +
    mg400_common::kinematics::LINK4;
  EXPECT_DOUBLE_EQ(expected_position.x(), actual.position.x);
  EXPECT_DOUBLE_EQ(expected_position.y(), actual.position.y);
  EXPECT_DOUBLE_EQ(expected_position.z(), actual.position.z);
  EXPECT_DOUBLE_EQ(1.0, actual.orientation.w);
  EXPECT_DOUBLE_EQ(0.0, actual.orientation.x);
  EXPECT_DOUBLE_EQ(0.0, actual.orientation.y);
  EXPECT_DOUBLE_EQ(0.0, actual.orientation.z);
}
