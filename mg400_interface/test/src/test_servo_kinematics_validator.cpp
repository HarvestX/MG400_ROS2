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

#include <array>
#include <cmath>
#include <limits>
#include <utility>

#include <gtest/gtest.h>
#include <mg400_common/kinematics.hpp>

#include "mg400_interface/servo_kinematics_validator.hpp"

namespace
{
using Code = mg400_interface::ServoSafetyViolationCode;
using Validator = mg400_interface::ServoKinematicsValidator;

TEST(ServoKinematicsValidator, ServoJUsesCanonicalIndividualLimitsAndBoundaries)
{
  Validator validator;
  const double margin = mg400_common::kinematics::MARGIN;
  const std::array<std::pair<std::size_t, std::pair<double, double>>, 4> limits{{
    {0, {mg400_common::kinematics::J1_MIN, mg400_common::kinematics::J1_MAX}},
    {1, {mg400_common::kinematics::J2_MIN, mg400_common::kinematics::J2_MAX}},
    {2, {mg400_common::kinematics::J3_MIN, mg400_common::kinematics::J3_MAX}},
    {3, {mg400_common::kinematics::J4_MIN, mg400_common::kinematics::J4_MAX}}}};

  for (const auto & entry : limits) {
    auto joints = std::array<double, 4>{{0.0, 0.0, 30.0 * M_PI / 180.0, 0.0}};
    joints[entry.first] = entry.second.first + margin;
    EXPECT_TRUE(validator.validateServoJ(joints).success) << entry.first;
    joints[entry.first] -= 1.0e-6;
    const auto below = validator.validateServoJ(joints);
    EXPECT_FALSE(below.success) << entry.first;
    EXPECT_EQ(Code::SERVO_J_JOINT_LIMIT, below.code) << entry.first;

    joints = {{0.0, 0.0, 30.0 * M_PI / 180.0, 0.0}};
    // Keep the canonical J3-J2 coupling valid while exercising the J3 maximum.
    if (entry.first == 2) {
      joints[1] = 1.0;
    }
    joints[entry.first] = entry.second.second - margin;
    EXPECT_TRUE(validator.validateServoJ(joints).success) << entry.first;
    joints[entry.first] += 1.0e-6;
    const auto above = validator.validateServoJ(joints);
    EXPECT_FALSE(above.success) << entry.first;
    EXPECT_EQ(Code::SERVO_J_JOINT_LIMIT, above.code) << entry.first;
  }
}

TEST(ServoKinematicsValidator, ServoJClassifiesCanonicalCoupledConstraint)
{
  Validator validator;
  const double margin = mg400_common::kinematics::MARGIN;
  std::array<double, 4> joints{{0.0, 0.0, 0.0, 0.0}};

  joints[2] = mg400_common::kinematics::J3_1_MAX - margin;
  EXPECT_TRUE(validator.validateServoJ(joints).success);
  joints[2] += 1.0e-6;
  const auto outside = validator.validateServoJ(joints);
  EXPECT_FALSE(outside.success);
  EXPECT_EQ(Code::SERVO_J_COUPLED_LIMIT, outside.code);
}

TEST(ServoKinematicsValidator, ServoJUsesCanonicalConditionalJ2Constraint)
{
  Validator validator;
  const double boundary = mg400_common::kinematics::J2_MIN_NO_COLLISION +
    mg400_common::kinematics::MARGIN;
  std::array<double, 4> joints{{0.0, boundary, -0.1, 0.0}};
  EXPECT_TRUE(validator.validateServoJ(joints).success);

  joints[1] -= 1.0e-6;
  const auto outside = validator.validateServoJ(joints);
  EXPECT_FALSE(outside.success);
  EXPECT_EQ(Code::SERVO_J_COUPLED_LIMIT, outside.code);
}

TEST(ServoKinematicsValidator, NonFiniteInputsStopBeforeKinematics)
{
  Validator validator;
  const auto servo_j = validator.validateServoJ(
    {{0.0, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}});
  EXPECT_FALSE(servo_j.success);
}

}  // namespace
