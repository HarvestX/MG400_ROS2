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

#include "mg400_interface/servo_mode/servo_operational_error.hpp"

#include <condition_variable>
#include <stdexcept>
#include <utility>

namespace mg400_interface
{

const char * toString(const ServoOperationalErrorCode code) noexcept
{
  switch (code) {
    case ServoOperationalErrorCode::NONE:
      return "NONE";
    case ServoOperationalErrorCode::WATCHDOG_TIMEOUT:
      return "WATCHDOG_TIMEOUT";
    case ServoOperationalErrorCode::MOTION_TCP_SEND_FAILED:
      return "MOTION_TCP_SEND_FAILED";
    case ServoOperationalErrorCode::SERVO_LEASE_LOST:
      return "SERVO_LEASE_LOST";
    case ServoOperationalErrorCode::REALTIME_CONNECTION_LOST:
      return "REALTIME_CONNECTION_LOST";
    case ServoOperationalErrorCode::STOP_RESET_FAILED:
      return "STOP_RESET_FAILED";
    case ServoOperationalErrorCode::STOP_CONFIRMATION_TIMEOUT:
      return "STOP_CONFIRMATION_TIMEOUT";
    case ServoOperationalErrorCode::INTERNAL_ERROR:
      return "INTERNAL_ERROR";
    default:
      return "UNKNOWN";
  }
}

class ServoOperationalErrorState::CallbackSlot
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

ServoOperationalErrorState::CallbackHandle::CallbackHandle(
  std::shared_ptr<CallbackSlot> slot)
: slot_(std::move(slot))
{
}

ServoOperationalErrorState::CallbackHandle::~CallbackHandle()
{
  if (this->slot_) {
    this->slot_->disableAndWait();
  }
}

ServoOperationalErrorState::ServoOperationalErrorState()
: state_{}
{
}

ServoOperationalErrorState::~ServoOperationalErrorState()
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

bool ServoOperationalErrorState::reportError(
  const ServoOperationalErrorCode code, const std::string & message)
{
  if (code == ServoOperationalErrorCode::NONE) {
    return false;
  }

  std::shared_ptr<CallbackSlot> callback_slot;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (this->state_.code != ServoOperationalErrorCode::NONE) {
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

bool ServoOperationalErrorState::clearForRearm()
{
  std::shared_ptr<CallbackSlot> callback_slot;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (this->state_.code == ServoOperationalErrorCode::NONE) {
      return false;
    }
    this->state_.code = ServoOperationalErrorCode::NONE;
    this->state_.message.clear();
    callback_slot = this->callback_slot_;
  }

  if (callback_slot) {
    callback_slot->invoke();
  }
  return true;
}

ServoOperationalErrorState::Snapshot ServoOperationalErrorState::getSnapshot() const
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->state_;
}

std::unique_ptr<ServoOperationalErrorState::CallbackHandle>
ServoOperationalErrorState::setChangeCallback(ChangeCallback callback)
{
  if (!callback) {
    throw std::invalid_argument("Servo operational error callback must not be empty");
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
