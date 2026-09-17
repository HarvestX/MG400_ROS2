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

#ifndef MG400_INTERFACE__ROBOT_STATE_MACHINE_HPP_
#define MG400_INTERFACE__ROBOT_STATE_MACHINE_HPP_

#include <cstdint>
#include <memory>
#include <mutex>

namespace mg400_interface
{

class RealtimeFeedbackTcpInterface;

/// Read-only view of the robot state derived from realtime feedback.
class RobotStateMachine
{
public:
  using SharedPtr = std::shared_ptr<RobotStateMachine>;
  using ConstSharedPtr = std::shared_ptr<const RobotStateMachine>;

  enum class State : uint8_t
  {
    UNKNOWN = 0,
    NOT_READY = 1,
    MANUAL = 2,
    DISABLED = 3,
    ENABLED = 4,
    RUNNING = 5,
    PAUSED_OR_JOG = 6,
    ERROR = 7,
  };

  struct Snapshot
  {
    State state{State::UNKNOWN};
    uint64_t raw_robot_mode{0};
    bool feedback_fresh{false};
  };

  /// Construct the state machine in UNKNOWN state.
  RobotStateMachine() noexcept;

  /// Return the latest state derived from realtime feedback.
  State getState() const;

  /// Return whether the latest state matches the expected state.
  bool isState(State expected) const;

  /// Return an internally consistent snapshot for ROS publication.
  Snapshot getSnapshot() const;

  /// Convert a raw Dobot RobotMode value into a driver state.
  static State fromRobotMode(uint64_t robot_mode) noexcept;

private:
  friend class RealtimeFeedbackTcpInterface;

  void update(uint64_t robot_mode);
  void reset();

  mutable std::mutex mutex_;
  Snapshot snapshot_;
};

}  // namespace mg400_interface

#endif  // MG400_INTERFACE__ROBOT_STATE_MACHINE_HPP_
