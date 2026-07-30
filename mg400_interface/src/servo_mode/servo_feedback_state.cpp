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

#include "mg400_interface/servo_mode/servo_feedback_state.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mg400_interface
{

bool ServoFeedbackState::Snapshot::isFresh(
  const ConnectionEpoch expected_epoch,
  const Clock::duration max_age,
  const Clock::time_point now) const noexcept
{
  return this->has_feedback && this->connection_epoch == expected_epoch &&
         max_age >= Clock::duration::zero() && now >= this->received_at &&
         now - this->received_at <= max_age;
}

ServoFeedbackState::ServoFeedbackState(const ConnectionEpoch connection_epoch)
: connection_epoch_(connection_epoch), snapshot_{}
{
  if (this->connection_epoch_ == 0) {
    throw std::invalid_argument("Servo feedback connection epoch must be non-zero");
  }
  this->snapshot_.connection_epoch = this->connection_epoch_;
}

ServoFeedbackState::ConnectionEpoch ServoFeedbackState::connectionEpoch() const
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->connection_epoch_;
}

void ServoFeedbackState::beginConnectionEpoch(const ConnectionEpoch connection_epoch)
{
  if (connection_epoch == 0) {
    throw std::invalid_argument("Servo feedback connection epoch must be non-zero");
  }
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->connection_epoch_ = connection_epoch;
  this->snapshot_ = Snapshot{};
  this->snapshot_.connection_epoch = connection_epoch;
}

void ServoFeedbackState::update(
  const std::array<double, 4> & joint_angles_rad,
  const Clock::time_point received_at)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  const auto finite = [](const double value) {return std::isfinite(value);};
  if (!std::all_of(joint_angles_rad.begin(), joint_angles_rad.end(), finite)) {
    this->snapshot_.has_feedback = false;
    return;
  }
  this->snapshot_.joint_angles_rad = joint_angles_rad;
  this->snapshot_.received_at = received_at;
  this->snapshot_.has_feedback = true;
  this->snapshot_.connection_epoch = this->connection_epoch_;
}

void ServoFeedbackState::invalidate()
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->snapshot_.has_feedback = false;
}

ServoFeedbackState::Snapshot ServoFeedbackState::getSnapshot() const
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->snapshot_;
}

}  // namespace mg400_interface
