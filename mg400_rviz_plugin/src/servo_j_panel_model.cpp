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

#include "mg400_rviz_plugin/servo_j_panel_model.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace mg400_rviz_plugin
{
namespace
{

bool endsWith(const std::string & text, const std::string & suffix)
{
  return text.size() >= suffix.size() &&
         text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

double bounded(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

}  // namespace

bool extractServoJoints(
  const sensor_msgs::msg::JointState & feedback, JointArray & joints)
{
  if (feedback.name.size() != feedback.position.size()) {
    return false;
  }

  const std::array<std::string, 4> names = {
    "mg400_j1", "mg400_j2_1", "mg400_j4_2", "mg400_j5"};
  JointArray extracted{};
  std::array<bool, 4> found{};
  for (size_t sample = 0; sample < feedback.name.size(); ++sample) {
    for (size_t joint = 0; joint < names.size(); ++joint) {
      if (!endsWith(feedback.name[sample], names[joint])) {
        continue;
      }
      if (found[joint] || !std::isfinite(feedback.position[sample])) {
        return false;
      }
      found[joint] = true;
      extracted[joint] = feedback.position[sample];
    }
  }
  if (!std::all_of(found.begin(), found.end(), [](bool value) {return value;})) {
    return false;
  }
  joints = extracted;
  return true;
}

void ServoJRateLimiter::reset(const JointArray & joints)
{
  position_ = joints;
  velocity_.fill(0.0);
}

JointArray ServoJRateLimiter::step(
  const JointArray & target, double dt, double max_speed, double max_acceleration)
{
  if (!std::isfinite(dt) || dt <= 0.0 ||
    !std::isfinite(max_speed) || max_speed <= 0.0 ||
    !std::isfinite(max_acceleration) || max_acceleration <= 0.0)
  {
    return position_;
  }

  for (size_t joint = 0; joint < position_.size(); ++joint) {
    if (!std::isfinite(target[joint])) {
      continue;
    }
    const double distance = target[joint] - position_[joint];
    const double stopping_speed = std::sqrt(2.0 * max_acceleration * std::abs(distance));
    const double desired_velocity = std::copysign(std::min(max_speed, stopping_speed), distance);
    const double velocity_change = bounded(
      desired_velocity - velocity_[joint], -max_acceleration * dt, max_acceleration * dt);
    velocity_[joint] += velocity_change;
    const double delta = velocity_[joint] * dt;
    if (std::abs(delta) >= std::abs(distance) && delta * distance >= 0.0) {
      position_[joint] = target[joint];
      velocity_[joint] = 0.0;
    } else {
      position_[joint] += delta;
    }
  }
  return position_;
}

JointArray ServoJRateLimiter::current() const
{
  return position_;
}

}  // namespace mg400_rviz_plugin
