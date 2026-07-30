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

#ifndef MG400_INTERFACE__SERVO_OPERATIONAL_ERROR_HPP_
#define MG400_INTERFACE__SERVO_OPERATIONAL_ERROR_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <mg400_msgs/msg/servo_error.hpp>

namespace mg400_interface
{

/// ROS-driver-local Servo operational errors backed by the ServoError wire values.
enum class ServoOperationalErrorCode : std::uint8_t
{
  NONE = mg400_msgs::msg::ServoError::OPERATIONAL_NONE,
  WATCHDOG_TIMEOUT = mg400_msgs::msg::ServoError::OPERATIONAL_WATCHDOG_TIMEOUT,
  MOTION_TCP_SEND_FAILED = mg400_msgs::msg::ServoError::OPERATIONAL_MOTION_TCP_SEND_FAILED,
  SERVO_LEASE_LOST = mg400_msgs::msg::ServoError::OPERATIONAL_SERVO_LEASE_LOST,
  REALTIME_CONNECTION_LOST =
    mg400_msgs::msg::ServoError::OPERATIONAL_REALTIME_CONNECTION_LOST,
  STOP_RESET_FAILED = mg400_msgs::msg::ServoError::OPERATIONAL_STOP_RESET_FAILED,
  STOP_CONFIRMATION_TIMEOUT =
    mg400_msgs::msg::ServoError::OPERATIONAL_STOP_CONFIRMATION_TIMEOUT,
  INTERNAL_ERROR = mg400_msgs::msg::ServoError::OPERATIONAL_INTERNAL_ERROR,
};

/// Return the stable enum-name representation, or UNKNOWN for an unknown value.
const char * toString(ServoOperationalErrorCode code) noexcept;

struct ServoOperationalError
{
  ServoOperationalErrorCode code{ServoOperationalErrorCode::NONE};
  std::string message;
};

/// Thread-safe first-fault latch shared across Servo connection epochs.
class ServoOperationalErrorState
{
private:
  class CallbackSlot;

public:
  using SharedPtr = std::shared_ptr<ServoOperationalErrorState>;
  using Snapshot = ServoOperationalError;
  using ChangeCallback = std::function<void ()>;

  /// RAII registration whose destruction waits for an in-flight callback.
  class CallbackHandle
  {
public:
    ~CallbackHandle();

    CallbackHandle(const CallbackHandle &) = delete;
    CallbackHandle & operator=(const CallbackHandle &) = delete;

private:
    friend class ServoOperationalErrorState;
    explicit CallbackHandle(std::shared_ptr<CallbackSlot> slot);

    std::shared_ptr<CallbackSlot> slot_;
  };

  ServoOperationalErrorState();
  ~ServoOperationalErrorState();

  ServoOperationalErrorState(const ServoOperationalErrorState &) = delete;
  ServoOperationalErrorState & operator=(const ServoOperationalErrorState &) = delete;

  /// Latch the first non-NONE error. Returns true only when state changes.
  bool reportError(ServoOperationalErrorCode code, const std::string & message);

  /// Clear a latched error for a fully successful explicit Servo rearm.
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

#endif  // MG400_INTERFACE__SERVO_OPERATIONAL_ERROR_HPP_
