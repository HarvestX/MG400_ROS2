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
#include <mg400_common/mg400_ik_util.hpp>
#include <mg400_interface/mg400_interface.hpp>


class TestJointHandler : public ::testing::Test
{
protected:
  virtual void SetUp() {}

  virtual void TearDown() {}
};


TEST_F(TestJointHandler, getEndPoint)
{
  auto js = std::make_shared<sensor_msgs::msg::JointState>();
  js->position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  geometry_msgs::msg::Pose actual;
  const auto ret = mg400_interface::JointHandler::getEndPose(js, actual);
  ASSERT_TRUE(ret);
  double eq_x = mg400_common::LINK1_X + mg400_common::LINK2_X + mg400_common::LINK3_X +
    mg400_common::LINK4_X;
  double eq_y = mg400_common::LINK1_Y + mg400_common::LINK2_Y + mg400_common::LINK3_Y +
    mg400_common::LINK4_Y;
  double eq_z = mg400_common::LINK1_Z + mg400_common::LINK2_Z + mg400_common::LINK3_Z +
    mg400_common::LINK4_Z;
  EXPECT_DOUBLE_EQ(eq_x, actual.position.x);
  EXPECT_DOUBLE_EQ(eq_y, actual.position.y);
  EXPECT_DOUBLE_EQ(eq_z, actual.position.z);
  EXPECT_DOUBLE_EQ(1.0, actual.orientation.w);
  EXPECT_DOUBLE_EQ(0.0, actual.orientation.x);
  EXPECT_DOUBLE_EQ(0.0, actual.orientation.y);
  EXPECT_DOUBLE_EQ(0.0, actual.orientation.z);
}
