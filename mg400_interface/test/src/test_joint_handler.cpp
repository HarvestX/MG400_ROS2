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
#include <mg400_interface/mg400_interface.hpp>


class TestJointHandler : public ::testing::Test
{
protected:
  virtual void SetUp() {}

  virtual void TearDown() {}
};


TEST_F(TestJointHandler, getEndPoint)
{
  sensor_msgs::msg::JointState js;
  js.position = {0.0, 0.0, 0.0, 0.0};

  const auto actual = mg400_interface::getEndPose(js);
  EXPECT_DOUBLE_EQ(0.284, actual.x);
  EXPECT_DOUBLE_EQ(0.0, actual.y);
  EXPECT_DOUBLE_EQ(0.118, actual.z);
}
