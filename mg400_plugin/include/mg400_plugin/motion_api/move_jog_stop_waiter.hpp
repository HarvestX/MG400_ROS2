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

#ifndef MG400_PLUGIN__MOTION_API__MOVE_JOG_STOP_WAITER_HPP_
#define MG400_PLUGIN__MOTION_API__MOVE_JOG_STOP_WAITER_HPP_

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <optional>

#include <mg400_interface/tcp_interface/motion_tcp_interface.hpp>

namespace mg400_plugin
{

enum class MoveJogRobotStatus
{
  WAITING,
  ENABLED,
  DISCONNECTED,
};

enum class MoveJogStopResult
{
  CONFIRMED,
  TIMEOUT,
  CONTROLLER_ERROR,
  RESPONSE_ERROR,
  DISCONNECTED,
};

struct MoveJogStopConfirmation
{
  MoveJogStopResult result{MoveJogStopResult::TIMEOUT};
  int32_t controller_error_id{-1};
};

/// Wait for the MoveJog() response and then RobotMode=ENABLE.
///
/// All time, response, robot-state, and waiting behavior is injected so this
/// safety decision can be tested deterministically without a robot or sleeps.
template<typename Now, typename WaitResponse, typename ReadRobotStatus, typename Wait>
MoveJogStopConfirmation waitForMoveJogStop(
  const std::chrono::steady_clock::time_point & deadline,
  Now now, WaitResponse wait_response, ReadRobotStatus read_robot_status, Wait wait)
{
  while (now() < deadline) {
    const auto response = wait_response(deadline - now());
    if (!response) {
      return {MoveJogStopResult::TIMEOUT, -1};
    }
    if (response->command != "MoveJog()") {
      continue;
    }
    if (response->result == mg400_interface::MotionResponseResult::CONTROLLER_ERROR) {
      return {MoveJogStopResult::CONTROLLER_ERROR, response->error_code};
    }
    if (response->result != mg400_interface::MotionResponseResult::SUCCESS) {
      return {MoveJogStopResult::RESPONSE_ERROR, -1};
    }
    break;
  }

  while (now() < deadline) {
    switch (read_robot_status()) {
      case MoveJogRobotStatus::ENABLED:
        return {MoveJogStopResult::CONFIRMED, 0};
      case MoveJogRobotStatus::DISCONNECTED:
        return {MoveJogStopResult::DISCONNECTED, -1};
      case MoveJogRobotStatus::WAITING:
        break;
    }

    using namespace std::chrono_literals;  // NOLINT
    const auto remaining = deadline - now();
    wait(std::min<std::chrono::steady_clock::duration>(remaining, 10ms));
  }
  return {MoveJogStopResult::TIMEOUT, -1};
}

}  // namespace mg400_plugin

#endif  // MG400_PLUGIN__MOTION_API__MOVE_JOG_STOP_WAITER_HPP_
