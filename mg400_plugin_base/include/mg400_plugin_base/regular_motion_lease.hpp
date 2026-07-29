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

#include <exception>
#include <functional>
#include <optional>
#include <string>
#include <utility>

#include <mg400_interface/control_state_manager.hpp>

namespace mg400_plugin_base
{

/// Move-only RAII ownership of one REGULAR_MOTION control-state lease.
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

    acquisition_result = manager->tryAcquire(Manager::State::REGULAR_MOTION);
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

    // Clear local ownership before calling the manager. A failed or stale
    // release must never be retried against a later owner.
    const auto manager = std::move(this->manager_);
    const auto lease_id = std::exchange(this->lease_id_, Manager::NO_LEASE);
    auto reporter = std::move(this->release_failure_reporter_);

    try {
      const auto result = manager->release(Manager::State::REGULAR_MOTION, lease_id);
      if (!result.success) {
        reportFailure(reporter, lease_id, result.message);
        return false;
      }
      return true;
    } catch (const std::exception & error) {
      reportFailure(reporter, lease_id, error.what());
    } catch (...) {
      reportFailure(reporter, lease_id, "unknown exception while releasing lease");
    }
    return false;
  }

  /// Drop the local token while deliberately retaining fail-closed ownership.
  void retainManagerOwnership() noexcept
  {
    this->manager_.reset();
    this->lease_id_ = Manager::NO_LEASE;
    this->release_failure_reporter_ = {};
  }

  bool isCurrent() const noexcept
  {
    return this->manager_ &&
           this->manager_->owns(Manager::State::REGULAR_MOTION, this->lease_id_);
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
      // Diagnostic reporting must not escape a noexcept release path.
    }
  }
};

}  // namespace mg400_plugin_base

#endif  // MG400_PLUGIN_BASE__REGULAR_MOTION_LEASE_HPP_
