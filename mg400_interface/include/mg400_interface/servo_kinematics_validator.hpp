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

#ifndef MG400_INTERFACE__SERVO_KINEMATICS_VALIDATOR_HPP_
#define MG400_INTERFACE__SERVO_KINEMATICS_VALIDATOR_HPP_

#include <array>
#include <string>

#include "mg400_interface/servo_safety_violation.hpp"

namespace mg400_interface
{

/// ROS-independent MG400 ServoJ kinematics safety validation.
class ServoKinematicsValidator
{
public:
  struct Result
  {
    bool success{false};
    ServoSafetyViolationCode code{ServoSafetyViolationCode::NONE};
    std::string message;
    std::string detail;
  };

  /// Central tolerance for safety comparison floating-point boundaries.
  static constexpr double FLOATING_POINT_TOLERANCE = 1.0e-12;

  Result validateServoJ(const std::array<double, 4> & joint_angles_rad) const;
};

}  // namespace mg400_interface

#endif  // MG400_INTERFACE__SERVO_KINEMATICS_VALIDATOR_HPP_
