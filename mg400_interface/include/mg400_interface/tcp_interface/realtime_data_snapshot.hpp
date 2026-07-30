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

#ifndef MG400_INTERFACE__REALTIME_DATA_SNAPSHOT_HPP_
#define MG400_INTERFACE__REALTIME_DATA_SNAPSHOT_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

#include "mg400_interface/command_utils.hpp"
#include "mg400_interface/tcp_interface/realtime_data.hpp"

namespace mg400_interface
{

/// One atomically captured port-30004 packet and its local receive metadata.
struct RealtimeDataSnapshot
{
  using Clock = std::chrono::steady_clock;
  using ConnectionEpoch = std::uint64_t;

  RealTimeData data{};
  Clock::time_point received_at{};
  bool has_data{false};
  ConnectionEpoch connection_epoch{0};

  /// A timeout boundary is usable: age <= max_age.
  bool isFresh(
    const ConnectionEpoch expected_epoch,
    const Clock::duration max_age,
    const Clock::time_point now) const noexcept
  {
    return this->has_data && this->connection_epoch == expected_epoch &&
           max_age >= Clock::duration::zero() && now >= this->received_at &&
           now - this->received_at <= max_age;
  }

  std::array<double, 4> jointAnglesRad() const noexcept
  {
    std::array<double, 4> joints{};
    for (std::size_t index = 0; index < joints.size(); ++index) {
      joints[index] = this->data.q_actual[index] * TO_RADIAN;
    }
    return joints;
  }
};

}  // namespace mg400_interface

#endif  // MG400_INTERFACE__REALTIME_DATA_SNAPSHOT_HPP_
