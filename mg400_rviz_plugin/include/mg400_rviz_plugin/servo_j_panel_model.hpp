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

#ifndef MG400_RVIZ_PLUGIN__SERVO_J_PANEL_MODEL_HPP_
#define MG400_RVIZ_PLUGIN__SERVO_J_PANEL_MODEL_HPP_

#include <array>

#include <sensor_msgs/msg/joint_state.hpp>

namespace mg400_rviz_plugin
{

using JointArray = std::array<double, 4>;

/** Extract controller-space J1-J4 angles from expanded RViz joint feedback.
 *
 * The driver publishes eight visualization joints. In particular, the ServoJ
 * J3 target is the absolute angle represented by mg400_j4_2, not mg400_j3_1.
 * Names are matched by suffix so a configured frame prefix is supported.
 *
 * @param feedback The driver's expanded joint-state message.
 * @param joints Output angles in radians, in controller J1-J4 order.
 * @return True only when all required joints occur exactly once and are finite.
 */
bool extractServoJoints(
  const sensor_msgs::msg::JointState & feedback, JointArray & joints);

/** Generate bounded intermediate ServoJ targets for manual slider control.
 *
 * State is reset to measured feedback at session start, so the first output
 * cannot jump to a stale or quantized slider value.
 */
class ServoJRateLimiter
{
public:
  /** Anchor the trajectory at the latest measured controller angles. */
  void reset(const JointArray & joints);

  /** Advance toward a target with per-joint speed and acceleration limits.
   *
   * @param target Requested controller angles in radians.
   * @param dt Elapsed seconds, capped by the caller after long timer gaps.
   * @param max_speed Maximum absolute joint speed in radians per second.
   * @param max_acceleration Maximum joint acceleration in radians per second squared.
   * @return The next bounded target in radians.
   */
  JointArray step(
    const JointArray & target, double dt, double max_speed, double max_acceleration);

  /** Return the most recently generated target. */
  JointArray current() const;

private:
  JointArray position_{};
  JointArray velocity_{};
};

}  // namespace mg400_rviz_plugin

#endif  // MG400_RVIZ_PLUGIN__SERVO_J_PANEL_MODEL_HPP_
