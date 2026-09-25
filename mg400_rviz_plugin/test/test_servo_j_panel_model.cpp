// Copyright 2026 HarvestX Inc.
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

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "mg400_rviz_plugin/servo_j_panel_model.hpp"

namespace mg400_rviz_plugin
{

TEST(ServoJPanelModel, ExtractsControllerAnglesByNameAndPreservesAbsoluteJ3)
{
  sensor_msgs::msg::JointState feedback;
  feedback.name = {
    "arm_mg400_j3_1", "arm_mg400_j5", "arm_mg400_j4_2",
    "arm_mg400_j2_1", "arm_mg400_j1"};
  feedback.position = {0.3, 0.4, 0.8, 0.5, 0.2};
  JointArray joints{};

  ASSERT_TRUE(extractServoJoints(feedback, joints));
  EXPECT_DOUBLE_EQ(joints[0], 0.2);
  EXPECT_DOUBLE_EQ(joints[1], 0.5);
  EXPECT_DOUBLE_EQ(joints[2], 0.8);
  EXPECT_DOUBLE_EQ(joints[3], 0.4);
}

TEST(ServoJPanelModel, RejectsMissingDuplicateOrNonfiniteFeedback)
{
  sensor_msgs::msg::JointState feedback;
  feedback.name = {"mg400_j1", "mg400_j2_1", "mg400_j4_2"};
  feedback.position = {0.0, 0.0, 0.0};
  JointArray joints{};
  EXPECT_FALSE(extractServoJoints(feedback, joints));

  feedback.name.push_back("mg400_j5");
  feedback.position.push_back(std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(extractServoJoints(feedback, joints));

  feedback.position.back() = 0.0;
  feedback.name.push_back("other_mg400_j5");
  feedback.position.push_back(0.0);
  EXPECT_FALSE(extractServoJoints(feedback, joints));
}

TEST(ServoJPanelModel, StartsAtFeedbackAndLimitsMotion)
{
  ServoJRateLimiter limiter;
  const JointArray feedback{0.1, 0.2, 0.3, 0.4};
  limiter.reset(feedback);
  EXPECT_EQ(limiter.current(), feedback);

  const JointArray target{1.1, 0.2, 0.3, 0.4};
  const auto first = limiter.step(target, 0.03, 0.2, 0.5);
  EXPECT_GT(first[0], feedback[0]);
  EXPECT_LE(first[0] - feedback[0], 0.5 * 0.03 * 0.03 + 1e-12);
  EXPECT_EQ(first[1], feedback[1]);
  EXPECT_EQ(first[2], feedback[2]);
  EXPECT_EQ(first[3], feedback[3]);

  auto previous = first;
  for (int step = 0; step < 1000; ++step) {
    const auto next = limiter.step(target, 0.03, 0.2, 0.5);
    EXPECT_LE(std::abs(next[0] - previous[0]), 0.2 * 0.03 + 1e-12);
    EXPECT_LE(next[0], target[0]);
    previous = next;
  }
  EXPECT_NEAR(limiter.current()[0], target[0], 1e-9);
}

}  // namespace mg400_rviz_plugin
