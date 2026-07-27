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

#ifndef MG400_PLUGIN_BASE__REGULAR_MOTION_LEASE_HPP_
#define MG400_PLUGIN_BASE__REGULAR_MOTION_LEASE_HPP_

#include <cstddef>
#include <exception>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

#include <mg400_interface/control_state_manager.hpp>

namespace mg400_plugin_base
{

/// Move-only RAII ownership of one ControlStateManager regular-motion lease.
class RegularMotionLease
{
public:
  using Manager = mg400_interface::ControlStateManager;
  using LeaseId = Manager::LeaseId;
  using ReleaseFailureReporter = std::function<void (LeaseId, const std::string &)>;

  RegularMotionLease() noexcept = default;
  RegularMotionLease(const RegularMotionLease &) = delete;
  RegularMotionLease & operator=(const RegularMotionLease &) = delete;

  RegularMotionLease(RegularMotionLease && other) noexcept
  {
    this->moveFrom(std::move(other));
  }

  RegularMotionLease & operator=(RegularMotionLease && other) noexcept
  {
    if (this != &other) {
      this->release();
      this->moveFrom(std::move(other));
    }
    return *this;
  }

  ~RegularMotionLease()
  {
    this->release();
  }

  static std::optional<RegularMotionLease> tryAcquire(
    const Manager::SharedPtr & manager,
    Manager::Result & acquisition_result,
    ReleaseFailureReporter release_failure_reporter = {})
  {
    if (!manager) {
      acquisition_result = Manager::Result{
        false, Manager::State::UNAVAILABLE, Manager::NO_LEASE,
        "ControlStateManager is not available"};
      return std::nullopt;
    }

    acquisition_result = manager->tryAcquireRegularMotion();
    if (!acquisition_result.success) {
      return std::nullopt;
    }

    return RegularMotionLease(
      manager, acquisition_result.lease_id, std::move(release_failure_reporter));
  }

  bool release() noexcept
  {
    if (!this->manager_ || this->lease_id_ == Manager::NO_LEASE) {
      return true;
    }

    // Clear the local ownership first. A failed/stale release must never be
    // retried later against a different ControlStateManager owner.
    const auto manager = std::move(this->manager_);
    const auto lease_id = std::exchange(this->lease_id_, Manager::NO_LEASE);
    auto reporter = std::move(this->release_failure_reporter_);

    try {
      const auto result = manager->releaseRegularMotion(lease_id);
      if (!result.success) {
        reportFailure(reporter, lease_id, result.message);
        return false;
      }
      return true;
    } catch (const std::exception & error) {
      reportFailure(reporter, lease_id, error.what());
    } catch (...) {
      reportFailure(
        reporter, lease_id, "unknown exception while releasing regular-motion lease");
    }
    return false;
  }

  /// Stop the local RAII object without releasing manager ownership.
  ///
  /// This is only for fail-closed teardown after a stop could not be
  /// confirmed. Normal completion must use release() or the destructor.
  void retainManagerOwnership() noexcept
  {
    this->manager_.reset();
    this->lease_id_ = Manager::NO_LEASE;
    this->release_failure_reporter_ = {};
  }

  bool isCurrent() const noexcept
  {
    if (!this->manager_ || this->lease_id_ == Manager::NO_LEASE) {
      return false;
    }
    const auto snapshot = this->manager_->getSnapshot();
    return snapshot.motion_owner == Manager::MotionOwner::REGULAR_MOTION &&
           snapshot.lease_id == this->lease_id_;
  }

  explicit operator bool() const noexcept
  {
    return this->manager_ && this->lease_id_ != Manager::NO_LEASE;
  }

  LeaseId id() const noexcept
  {
    return this->lease_id_;
  }

private:
  Manager::SharedPtr manager_;
  LeaseId lease_id_{Manager::NO_LEASE};
  ReleaseFailureReporter release_failure_reporter_;

  RegularMotionLease(
    Manager::SharedPtr manager, const LeaseId lease_id,
    ReleaseFailureReporter release_failure_reporter) noexcept
  : manager_(std::move(manager)),
    lease_id_(lease_id),
    release_failure_reporter_(std::move(release_failure_reporter))
  {}

  void moveFrom(RegularMotionLease && other) noexcept
  {
    this->manager_ = std::move(other.manager_);
    this->lease_id_ = std::exchange(other.lease_id_, Manager::NO_LEASE);
    this->release_failure_reporter_ = std::move(other.release_failure_reporter_);
  }

  static void reportFailure(
    const ReleaseFailureReporter & reporter, const LeaseId lease_id,
    const std::string & message) noexcept
  {
    if (!reporter) {
      return;
    }
    try {
      reporter(lease_id, message);
    } catch (...) {
      // A diagnostic callback must never turn a noexcept lease destructor
      // into process termination.
    }
  }
};

/// Thread-safe handoff from an Action goal callback to its execution thread.
///
/// Context is deliberately generic so this component has no dependency on a
/// ROS Action type. Erasing an entry releases its lease through RAII.
template<typename Key, typename Context, typename Hash = std::hash<Key>>
class RegularMotionGoalReservations
{
public:
  struct Entry
  {
    RegularMotionLease lease;
    Context context;
  };

  bool reserve(const Key & key, Entry entry)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    // try_emplace does not move entry when key already exists. The rejected
    // entry (and its lease) is therefore destroyed only after lock goes out
    // of scope at function exit.
    return this->entries_.try_emplace(key, std::move(entry)).second;
  }

  std::optional<Entry> take(const Key & key)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    const auto found = this->entries_.find(key);
    if (found == this->entries_.end()) {
      return std::nullopt;
    }

    Entry entry = std::move(found->second);
    this->entries_.erase(found);
    return entry;
  }

  bool erase(const Key & key)
  {
    // take() moves ownership out while locked; destruction and lease release
    // happen here after take() has released the reservation mutex.
    return this->take(key).has_value();
  }

  std::size_t size() const
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->entries_.size();
  }

private:
  mutable std::mutex mutex_;
  std::unordered_map<Key, Entry, Hash> entries_;
};

template<typename ByteContainer>
std::string makeGoalReservationKey(const ByteContainer & bytes)
{
  return std::string(
    reinterpret_cast<const char *>(bytes.data()),
    bytes.size() * sizeof(typename ByteContainer::value_type));
}

}  // namespace mg400_plugin_base

#endif  // MG400_PLUGIN_BASE__REGULAR_MOTION_LEASE_HPP_
