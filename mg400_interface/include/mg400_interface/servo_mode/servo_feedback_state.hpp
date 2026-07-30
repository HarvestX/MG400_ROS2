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

#ifndef MG400_INTERFACE__SERVO_FEEDBACK_STATE_HPP_
#define MG400_INTERFACE__SERVO_FEEDBACK_STATE_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>

namespace mg400_interface
{

/// Thread-safe, ROS-independent SI-unit view of one connection epoch's feedback.
class ServoFeedbackState
{
public:
  using Clock = std::chrono::steady_clock;
  using ConnectionEpoch = std::uint64_t;
  using SharedPtr = std::shared_ptr<ServoFeedbackState>;

  struct Snapshot
  {
    std::array<double, 4> joint_angles_rad{{0.0, 0.0, 0.0, 0.0}};
    Clock::time_point received_at{};
    bool has_feedback{false};
    ConnectionEpoch connection_epoch{0};

    /// A timeout boundary is usable: age <= max_age.
    bool isFresh(
      ConnectionEpoch expected_epoch,
      Clock::duration max_age,
      Clock::time_point now) const noexcept;
  };

  explicit ServoFeedbackState(ConnectionEpoch connection_epoch);

  ServoFeedbackState(const ServoFeedbackState &) = delete;
  ServoFeedbackState & operator=(const ServoFeedbackState &) = delete;

  ConnectionEpoch connectionEpoch() const;

  /// Start a reconnect epoch and discard all values from the preceding epoch.
  void beginConnectionEpoch(ConnectionEpoch connection_epoch);

  /// Atomically replace joint values from one feedback frame.
  void update(
    const std::array<double, 4> & joint_angles_rad,
    Clock::time_point received_at = Clock::now());

  /// Make the current epoch unusable until a later valid feedback frame arrives.
  void invalidate();

  Snapshot getSnapshot() const;

private:
  ConnectionEpoch connection_epoch_;
  mutable std::mutex mutex_;
  Snapshot snapshot_;
};

}  // namespace mg400_interface

#endif  // MG400_INTERFACE__SERVO_FEEDBACK_STATE_HPP_
