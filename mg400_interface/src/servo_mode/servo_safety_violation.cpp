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

#include "mg400_interface/servo_mode/servo_safety_violation.hpp"

#include <condition_variable>
#include <stdexcept>
#include <utility>

namespace mg400_interface
{

const char * toString(const ServoSafetyViolationCode code) noexcept
{
  switch (code) {
    case ServoSafetyViolationCode::NONE:
      return "NONE";
    case ServoSafetyViolationCode::SERVO_J_JOINT_LIMIT:
      return "SERVO_J_JOINT_LIMIT";
    case ServoSafetyViolationCode::SERVO_J_COUPLED_LIMIT:
      return "SERVO_J_COUPLED_LIMIT";
    case ServoSafetyViolationCode::SERVO_J_COMMAND_DISCONTINUITY:
      return "SERVO_J_COMMAND_DISCONTINUITY";
    case ServoSafetyViolationCode::REALTIME_FEEDBACK_UNAVAILABLE:
      return "REALTIME_FEEDBACK_UNAVAILABLE";
    default:
      return "UNKNOWN";
  }
}

class ServoSafetyViolationState::CallbackSlot
{
public:
  explicit CallbackSlot(ChangeCallback callback)
  : callback_(std::move(callback)), enabled_(true), callbacks_in_progress_(0) {}

  void invoke() noexcept
  {
    ChangeCallback callback;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      if (!this->enabled_) {
        return;
      }
      ++this->callbacks_in_progress_;
      callback = this->callback_;
    }

    try {
      callback();
    } catch (...) {
      // Notification failures must not undo or hide the already-latched state.
    }

    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      --this->callbacks_in_progress_;
      if (this->callbacks_in_progress_ == 0) {
        this->cv_.notify_all();
      }
    }
  }

  void disableAndWait() noexcept
  {
    std::unique_lock<std::mutex> lock(this->mutex_);
    this->enabled_ = false;
    this->callback_ = ChangeCallback();
    this->cv_.wait(lock, [this]() {return this->callbacks_in_progress_ == 0;});
  }

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  ChangeCallback callback_;
  bool enabled_;
  std::size_t callbacks_in_progress_;
};

ServoSafetyViolationState::CallbackHandle::CallbackHandle(
  std::shared_ptr<CallbackSlot> slot)
: slot_(std::move(slot))
{
}

ServoSafetyViolationState::CallbackHandle::~CallbackHandle()
{
  if (this->slot_) {
    this->slot_->disableAndWait();
  }
}

ServoSafetyViolationState::ServoSafetyViolationState()
: state_{}
{
}

ServoSafetyViolationState::~ServoSafetyViolationState()
{
  std::shared_ptr<CallbackSlot> callback_slot;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    callback_slot = std::move(this->callback_slot_);
  }
  if (callback_slot) {
    callback_slot->disableAndWait();
  }
}

bool ServoSafetyViolationState::reportViolation(
  const ServoSafetyViolationCode code, const std::string & message)
{
  if (code == ServoSafetyViolationCode::NONE) {
    return false;
  }

  std::shared_ptr<CallbackSlot> callback_slot;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (this->state_.code != ServoSafetyViolationCode::NONE) {
      return false;
    }
    this->state_.code = code;
    this->state_.message = message;
    callback_slot = this->callback_slot_;
  }

  if (callback_slot) {
    callback_slot->invoke();
  }
  return true;
}

bool ServoSafetyViolationState::clearForRearm()
{
  std::shared_ptr<CallbackSlot> callback_slot;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (this->state_.code == ServoSafetyViolationCode::NONE) {
      return false;
    }
    this->state_.code = ServoSafetyViolationCode::NONE;
    this->state_.message.clear();
    callback_slot = this->callback_slot_;
  }

  if (callback_slot) {
    callback_slot->invoke();
  }
  return true;
}

ServoSafetyViolationState::Snapshot ServoSafetyViolationState::getSnapshot() const
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->state_;
}

std::unique_ptr<ServoSafetyViolationState::CallbackHandle>
ServoSafetyViolationState::setChangeCallback(ChangeCallback callback)
{
  if (!callback) {
    throw std::invalid_argument("Servo safety violation callback must not be empty");
  }

  auto callback_slot = std::make_shared<CallbackSlot>(std::move(callback));
  std::shared_ptr<CallbackSlot> previous_slot;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    previous_slot = std::move(this->callback_slot_);
    this->callback_slot_ = callback_slot;
  }
  if (previous_slot) {
    previous_slot->disableAndWait();
  }
  return std::unique_ptr<CallbackHandle>(new CallbackHandle(std::move(callback_slot)));
}

}  // namespace mg400_interface
