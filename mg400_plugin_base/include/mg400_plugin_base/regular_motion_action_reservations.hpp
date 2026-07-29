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

#ifndef MG400_PLUGIN_BASE__REGULAR_MOTION_ACTION_RESERVATIONS_HPP_
#define MG400_PLUGIN_BASE__REGULAR_MOTION_ACTION_RESERVATIONS_HPP_

#include <cstddef>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

#include <mg400_plugin_base/regular_motion_lease.hpp>

namespace mg400_plugin_base
{

/// Transfers an accepted Action's lease to its execution callback.
template<typename Key, typename Hash = std::hash<Key>>
class RegularMotionActionReservations
{
public:
  using Lease = RegularMotionLease;

  bool reserve(const Key & key, Lease lease)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->leases_.try_emplace(key, std::move(lease)).second;
  }

  std::optional<Lease> take(const Key & key)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    const auto found = this->leases_.find(key);
    if (found == this->leases_.end()) {
      return std::nullopt;
    }

    Lease lease = std::move(found->second);
    this->leases_.erase(found);
    return lease;
  }

  std::size_t size() const
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->leases_.size();
  }

private:
  mutable std::mutex mutex_;
  std::unordered_map<Key, Lease, Hash> leases_;
};

template<typename ByteContainer>
std::string makeActionGoalKey(const ByteContainer & bytes)
{
  return std::string(
    reinterpret_cast<const char *>(bytes.data()),
    bytes.size() * sizeof(typename ByteContainer::value_type));
}

}  // namespace mg400_plugin_base

#endif  // MG400_PLUGIN_BASE__REGULAR_MOTION_ACTION_RESERVATIONS_HPP_
