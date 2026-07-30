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

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "mg400_interface/servo_mode/servo_safety_violation.hpp"

namespace
{

using Code = mg400_interface::ServoSafetyViolationCode;
using State = mg400_interface::ServoSafetyViolationState;

TEST(ServoSafetyViolationState, InitialStateIsNoneWithEmptyMessage)
{
  State state;
  const auto snapshot = state.getSnapshot();
  EXPECT_EQ(Code::NONE, snapshot.code);
  EXPECT_TRUE(snapshot.message.empty());
}

TEST(ServoSafetyViolationState, FirstViolationIsLatchedUntilRearm)
{
  State state;
  EXPECT_TRUE(
    state.reportViolation(
      Code::SERVO_J_COMMAND_DISCONTINUITY,
      "Servo target is too far from the current position"));
  EXPECT_FALSE(
    state.reportViolation(
      Code::REALTIME_FEEDBACK_UNAVAILABLE,
      "Realtime feedback is stale"));

  auto snapshot = state.getSnapshot();
  EXPECT_EQ(Code::SERVO_J_COMMAND_DISCONTINUITY, snapshot.code);
  EXPECT_EQ("Servo target is too far from the current position", snapshot.message);

  EXPECT_TRUE(state.clearForRearm());
  snapshot = state.getSnapshot();
  EXPECT_EQ(Code::NONE, snapshot.code);
  EXPECT_TRUE(snapshot.message.empty());
  EXPECT_FALSE(state.clearForRearm());
}

TEST(ServoSafetyViolationState, RejectsNoneReports)
{
  State state;
  EXPECT_FALSE(state.reportViolation(Code::NONE, "must be rejected"));
  const auto snapshot = state.getSnapshot();
  EXPECT_EQ(Code::NONE, snapshot.code);
  EXPECT_TRUE(snapshot.message.empty());
}

TEST(ServoSafetyViolationCode, ConvertsEveryStableValueAndUnknown)
{
  EXPECT_STREQ("NONE", mg400_interface::toString(Code::NONE));
  EXPECT_STREQ(
    "SERVO_J_JOINT_LIMIT", mg400_interface::toString(Code::SERVO_J_JOINT_LIMIT));
  EXPECT_STREQ(
    "SERVO_J_COUPLED_LIMIT", mg400_interface::toString(Code::SERVO_J_COUPLED_LIMIT));
  EXPECT_STREQ(
    "SERVO_J_COMMAND_DISCONTINUITY",
    mg400_interface::toString(Code::SERVO_J_COMMAND_DISCONTINUITY));
  EXPECT_STREQ(
    "REALTIME_FEEDBACK_UNAVAILABLE",
    mg400_interface::toString(Code::REALTIME_FEEDBACK_UNAVAILABLE));
  EXPECT_STREQ("UNKNOWN", mg400_interface::toString(static_cast<Code>(255)));
}

TEST(ServoSafetyViolationState, InvokesChangeCallbackWithoutHoldingStateMutex)
{
  State state;
  std::atomic<unsigned int> callback_count{0};
  auto callback = state.setChangeCallback(
    [&state, &callback_count]() {
      static_cast<void>(state.getSnapshot());
      ++callback_count;
    });

  EXPECT_TRUE(state.reportViolation(Code::SERVO_J_JOINT_LIMIT, "joint limit"));
  EXPECT_TRUE(state.clearForRearm());
  EXPECT_EQ(2U, callback_count.load());
  static_cast<void>(callback);
}

TEST(ServoSafetyViolationState, ConcurrentReportSnapshotAndClearStayConsistent)
{
  State state;
  std::atomic<bool> start{false};
  std::atomic<bool> inconsistent{false};
  std::vector<std::thread> threads;
  threads.reserve(4);

  for (int writer = 0; writer < 4; ++writer) {
    threads.emplace_back(
      [&state, &start, writer]() {
        while (!start.load()) {}
        const auto code = writer % 2 == 0 ?
        Code::SERVO_J_JOINT_LIMIT : Code::SERVO_J_COUPLED_LIMIT;
        const std::string message = writer % 2 == 0 ? "joint" : "coupled";
        for (int iteration = 0; iteration < 5000; ++iteration) {
          static_cast<void>(state.reportViolation(code, message));
        }
      });
  }
  threads.emplace_back(
    [&state, &start]() {
      while (!start.load()) {}
      for (int iteration = 0; iteration < 5000; ++iteration) {
        static_cast<void>(state.clearForRearm());
      }
    });
  for (int reader = 0; reader < 3; ++reader) {
    threads.emplace_back(
      [&state, &start, &inconsistent]() {
        while (!start.load()) {}
        for (int iteration = 0; iteration < 10000; ++iteration) {
          const auto snapshot = state.getSnapshot();
          const bool valid_none =
          snapshot.code == Code::NONE && snapshot.message.empty();
          const bool valid_joint =
          snapshot.code == Code::SERVO_J_JOINT_LIMIT && snapshot.message == "joint";
          const bool valid_coupled =
          snapshot.code == Code::SERVO_J_COUPLED_LIMIT && snapshot.message == "coupled";
          if (!valid_none && !valid_joint && !valid_coupled) {
            inconsistent.store(true);
          }
        }
      });
  }

  start.store(true);
  for (auto & thread : threads) {
    thread.join();
  }
  EXPECT_FALSE(inconsistent.load());

  state.clearForRearm();
  const auto snapshot = state.getSnapshot();
  EXPECT_EQ(Code::NONE, snapshot.code);
  EXPECT_TRUE(snapshot.message.empty());
}

}  // namespace
