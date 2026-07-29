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

#ifndef MG400_INTERFACE__SERVO_SAFETY_VIOLATION_HPP_
#define MG400_INTERFACE__SERVO_SAFETY_VIOLATION_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <mg400_msgs/msg/servo_error.hpp>

namespace mg400_interface
{

/// Driver-local Servo safety violations backed by the ServoError wire values.
enum class ServoSafetyViolationCode : std::uint8_t
{
  NONE = mg400_msgs::msg::ServoError::SAFETY_NONE,
  SERVO_J_JOINT_LIMIT = mg400_msgs::msg::ServoError::SAFETY_SERVO_J_JOINT_LIMIT,
  SERVO_J_COUPLED_LIMIT = mg400_msgs::msg::ServoError::SAFETY_SERVO_J_COUPLED_LIMIT,
  SERVO_J_COMMAND_DISCONTINUITY =
    mg400_msgs::msg::ServoError::SAFETY_SERVO_J_COMMAND_DISCONTINUITY,
  REALTIME_FEEDBACK_UNAVAILABLE =
    mg400_msgs::msg::ServoError::SAFETY_REALTIME_FEEDBACK_UNAVAILABLE,
};

/// Return the stable enum-name representation, or UNKNOWN for an unknown value.
const char * toString(ServoSafetyViolationCode code) noexcept;

struct ServoSafetyViolation
{
  ServoSafetyViolationCode code{ServoSafetyViolationCode::NONE};
  std::string message;
};

/// Thread-safe first-fault latch shared across Servo connection epochs.
class ServoSafetyViolationState
{
private:
  class CallbackSlot;

public:
  using SharedPtr = std::shared_ptr<ServoSafetyViolationState>;
  using Snapshot = ServoSafetyViolation;
  using ChangeCallback = std::function<void ()>;

  /// RAII registration whose destruction waits for an in-flight callback.
  class CallbackHandle
  {
public:
    ~CallbackHandle();

    CallbackHandle(const CallbackHandle &) = delete;
    CallbackHandle & operator=(const CallbackHandle &) = delete;

private:
    friend class ServoSafetyViolationState;
    explicit CallbackHandle(std::shared_ptr<CallbackSlot> slot);

    std::shared_ptr<CallbackSlot> slot_;
  };

  ServoSafetyViolationState();
  ~ServoSafetyViolationState();

  ServoSafetyViolationState(const ServoSafetyViolationState &) = delete;
  ServoSafetyViolationState & operator=(const ServoSafetyViolationState &) = delete;

  /// Latch the first non-NONE violation. Returns true only when state changes.
  bool reportViolation(ServoSafetyViolationCode code, const std::string & message);

  /// Clear a latched violation for a successful explicit Servo rearm.
  bool clearForRearm();

  Snapshot getSnapshot() const;

  /// Register the single owner notification callback.
  std::unique_ptr<CallbackHandle> setChangeCallback(ChangeCallback callback);

private:
  mutable std::mutex mutex_;
  Snapshot state_;
  std::shared_ptr<CallbackSlot> callback_slot_;
};

}  // namespace mg400_interface

#endif  // MG400_INTERFACE__SERVO_SAFETY_VIOLATION_HPP_
