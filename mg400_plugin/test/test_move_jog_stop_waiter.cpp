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

#include <chrono>
#include <deque>
#include <optional>

#include <gtest/gtest.h>

#include "mg400_plugin/motion_api/move_jog_stop_waiter.hpp"

namespace
{

using mg400_interface::MotionResponse;
using mg400_interface::MotionResponseResult;
using mg400_plugin::MoveJogRobotStatus;
using mg400_plugin::MoveJogStopResult;

class FakeSteadyClock
{
public:
  std::chrono::steady_clock::time_point now{};

  void advance(const std::chrono::steady_clock::duration & duration)
  {
    this->now += duration;
  }
};

TEST(TestMoveJogStopWaiter, ConfirmsOnlyAfterStopResponseAndEnable)
{
  using namespace std::chrono_literals;  // NOLINT
  FakeSteadyClock clock;
  std::deque<MotionResponse> responses{
    MotionResponse{1, "MoveJog(j1+)", "", 0, {}, {}, MotionResponseResult::SUCCESS},
    MotionResponse{2, "MoveJog()", "", 0, {}, {}, MotionResponseResult::SUCCESS}};
  std::deque<MoveJogRobotStatus> statuses{
    MoveJogRobotStatus::WAITING, MoveJogRobotStatus::ENABLED};

  const auto result = mg400_plugin::waitForMoveJogStop(
    clock.now + 2s,
    [&clock]() {return clock.now;},
    [&responses](const auto &) -> std::optional<MotionResponse> {
      if (responses.empty()) {
        return std::nullopt;
      }
      auto response = responses.front();
      responses.pop_front();
      return response;
    },
    [&statuses]() {
      const auto status = statuses.front();
      statuses.pop_front();
      return status;
    },
    [&clock](const auto & duration) {clock.advance(duration);});

  EXPECT_EQ(MoveJogStopResult::CONFIRMED, result.result);
  EXPECT_EQ(0, result.controller_error_id);
  EXPECT_TRUE(responses.empty());
  EXPECT_TRUE(statuses.empty());
}

TEST(TestMoveJogStopWaiter, TimesOutWithoutReleasingOnMissingResponse)
{
  using namespace std::chrono_literals;  // NOLINT
  FakeSteadyClock clock;
  bool robot_status_was_read = false;

  const auto result = mg400_plugin::waitForMoveJogStop(
    clock.now + 2s,
    [&clock]() {return clock.now;},
    [&clock](const auto & remaining) -> std::optional<MotionResponse> {
      clock.advance(remaining);
      return std::nullopt;
    },
    [&robot_status_was_read]() {
      robot_status_was_read = true;
      return MoveJogRobotStatus::ENABLED;
    },
    [](const auto &) {});

  EXPECT_EQ(MoveJogStopResult::TIMEOUT, result.result);
  EXPECT_EQ(-1, result.controller_error_id);
  EXPECT_FALSE(robot_status_was_read);
}

TEST(TestMoveJogStopWaiter, PreservesControllerErrorCode)
{
  using namespace std::chrono_literals;  // NOLINT
  FakeSteadyClock clock;
  MotionResponse response;
  response.command = "MoveJog()";
  response.result = MotionResponseResult::CONTROLLER_ERROR;
  response.error_code = -3;

  const auto result = mg400_plugin::waitForMoveJogStop(
    clock.now + 2s,
    [&clock]() {return clock.now;},
    [response](const auto &) -> std::optional<MotionResponse> {return response;},
    []() {return MoveJogRobotStatus::ENABLED;},
    [](const auto &) {});

  EXPECT_EQ(MoveJogStopResult::CONTROLLER_ERROR, result.result);
  EXPECT_EQ(-3, result.controller_error_id);
}

TEST(TestMoveJogStopWaiter, DisconnectCannotConfirmStop)
{
  using namespace std::chrono_literals;  // NOLINT
  FakeSteadyClock clock;
  MotionResponse response;
  response.command = "MoveJog()";
  response.result = MotionResponseResult::SUCCESS;

  const auto result = mg400_plugin::waitForMoveJogStop(
    clock.now + 2s,
    [&clock]() {return clock.now;},
    [response](const auto &) -> std::optional<MotionResponse> {return response;},
    []() {return MoveJogRobotStatus::DISCONNECTED;},
    [](const auto &) {});

  EXPECT_EQ(MoveJogStopResult::DISCONNECTED, result.result);
  EXPECT_EQ(-1, result.controller_error_id);
}

}  // namespace
