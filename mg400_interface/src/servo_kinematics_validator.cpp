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

#include "mg400_interface/servo_kinematics_validator.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

#include <mg400_common/kinematics.hpp>

namespace mg400_interface
{
namespace
{

using Kinematics = ServoKinematicsValidator;
using Result = ServoKinematicsValidator::Result;

bool finite(const std::array<double, 4> & values)
{
  return std::all_of(
    values.begin(), values.end(), [](const double value) {return std::isfinite(value);});
}

std::string valuesToString(const std::array<double, 4> & values)
{
  std::ostringstream stream;
  stream << std::setprecision(12) << '[' << values[0] << ", " << values[1] << ", " <<
    values[2] << ", " << values[3] << ']';
  return stream.str();
}

Eigen::Vector4d toEigen(const std::array<double, 4> & values)
{
  Eigen::Vector4d result;
  result << values[0], values[1], values[2], values[3];
  return result;
}

mg400_common::kinematics::ConstraintResult checkIndividualJointConstraints(
  const Eigen::Vector4d & joints)
{
  // Use the repository's canonical checker unchanged. This option set only
  // separates the unconditional joint ranges from MG400's conditional J2 and
  // J3-J2 coupled constraints so ServoJ can report the required root cause.
  mg400_common::kinematics::ConstraintOptions options;
  options.j2_min_no_collision = options.j2_min;
  options.j3_1_min = -std::numeric_limits<double>::max();
  options.j3_1_max = std::numeric_limits<double>::max();
  return mg400_common::kinematics::check_constraints(joints, options);
}

}  // namespace

constexpr double ServoKinematicsValidator::FLOATING_POINT_TOLERANCE;

ServoKinematicsValidator::Result ServoKinematicsValidator::validateServoJ(
  const std::array<double, 4> & joint_angles_rad) const
{
  if (!finite(joint_angles_rad)) {
    return Result{
      false, ServoSafetyViolationCode::SERVO_J_JOINT_LIMIT,
      "ServoJ target contains a non-finite value",
      "Non-finite ServoJ values were rejected before MG400 kinematics"};
  }

  const auto joints = toEigen(joint_angles_rad);
  const auto individual = checkIndividualJointConstraints(joints);
  if (!individual.allValid()) {
    std::ostringstream detail;
    detail << "ServoJ target(rad)=" << valuesToString(joint_angles_rad) <<
      "; canonical individual constraint results: J1=" << individual.j1_valid <<
      ", J2=" << individual.j2_valid << ", J3=" << individual.j3_valid <<
      ", J4=" << individual.j4_valid << "; frame=MG400 joint coordinates";
    return Result{
      false, ServoSafetyViolationCode::SERVO_J_JOINT_LIMIT,
      "ServoJ target violates an MG400 joint limit", detail.str()};
  }

  const auto full = mg400_common::kinematics::check_constraints(joints);
  if (!full.allValid()) {
    std::ostringstream detail;
    detail << "ServoJ target(rad)=" << valuesToString(joint_angles_rad) <<
      "; canonical coupled constraint results: conditional_J2=" << full.j2_valid <<
      ", J3_minus_J2=" << full.j3_1_valid << "; frame=MG400 joint coordinates";
    return Result{
      false, ServoSafetyViolationCode::SERVO_J_COUPLED_LIMIT,
      "ServoJ target violates an MG400 coupled-joint constraint",
      detail.str()};
  }

  return Result{true, ServoSafetyViolationCode::NONE, {}, {}};
}

}  // namespace mg400_interface
