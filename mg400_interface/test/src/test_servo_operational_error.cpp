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

#include <gtest/gtest.h>

#include "mg400_interface/servo_operational_error.hpp"

namespace
{

using Code = mg400_interface::ServoOperationalErrorCode;
using State = mg400_interface::ServoOperationalErrorState;

TEST(ServoOperationalErrorState, InitialStateIsNoneWithEmptyMessage)
{
  State state;
  const auto snapshot = state.getSnapshot();
  EXPECT_EQ(Code::NONE, snapshot.code);
  EXPECT_TRUE(snapshot.message.empty());
}

TEST(ServoOperationalErrorState, FirstErrorIsLatchedUntilRearm)
{
  State state;
  EXPECT_TRUE(state.reportError(Code::WATCHDOG_TIMEOUT, "target watchdog expired"));
  EXPECT_FALSE(state.reportError(Code::STOP_RESET_FAILED, "ResetRobot failed"));
  EXPECT_FALSE(state.reportError(Code::NONE, "NONE must be rejected"));

  auto snapshot = state.getSnapshot();
  EXPECT_EQ(Code::WATCHDOG_TIMEOUT, snapshot.code);
  EXPECT_EQ("target watchdog expired", snapshot.message);

  EXPECT_TRUE(state.clearForRearm());
  snapshot = state.getSnapshot();
  EXPECT_EQ(Code::NONE, snapshot.code);
  EXPECT_TRUE(snapshot.message.empty());
  EXPECT_FALSE(state.clearForRearm());
}

TEST(ServoOperationalErrorState, CallbackRunsOnlyForChangesAndWithoutStateMutex)
{
  State state;
  std::atomic<unsigned int> callback_count{0};
  auto callback = state.setChangeCallback(
    [&state, &callback_count]() {
      static_cast<void>(state.getSnapshot());
      ++callback_count;
    });

  EXPECT_TRUE(state.reportError(Code::MOTION_TCP_SEND_FAILED, "send failed"));
  EXPECT_FALSE(state.reportError(Code::MOTION_RESPONSE_TIMEOUT, "response timed out"));
  EXPECT_TRUE(state.clearForRearm());
  EXPECT_EQ(2U, callback_count.load());
  static_cast<void>(callback);
}

TEST(ServoOperationalErrorCode, ConvertsEveryStableValueAndUnknown)
{
  EXPECT_STREQ("NONE", mg400_interface::toString(Code::NONE));
  EXPECT_STREQ("WATCHDOG_TIMEOUT", mg400_interface::toString(Code::WATCHDOG_TIMEOUT));
  EXPECT_STREQ(
    "MOTION_TCP_SEND_FAILED", mg400_interface::toString(Code::MOTION_TCP_SEND_FAILED));
  EXPECT_STREQ(
    "MOTION_RESPONSE_CONTROLLER_ERROR",
    mg400_interface::toString(Code::MOTION_RESPONSE_CONTROLLER_ERROR));
  EXPECT_STREQ(
    "MOTION_RESPONSE_TIMEOUT", mg400_interface::toString(Code::MOTION_RESPONSE_TIMEOUT));
  EXPECT_STREQ(
    "MOTION_RESPONSE_DISCONNECTED",
    mg400_interface::toString(Code::MOTION_RESPONSE_DISCONNECTED));
  EXPECT_STREQ(
    "MOTION_RESPONSE_PARSE_ERROR",
    mg400_interface::toString(Code::MOTION_RESPONSE_PARSE_ERROR));
  EXPECT_STREQ(
    "MOTION_RESPONSE_QUEUE_OVERFLOW",
    mg400_interface::toString(Code::MOTION_RESPONSE_QUEUE_OVERFLOW));
  EXPECT_STREQ("SERVO_LEASE_LOST", mg400_interface::toString(Code::SERVO_LEASE_LOST));
  EXPECT_STREQ(
    "REALTIME_CONNECTION_LOST", mg400_interface::toString(Code::REALTIME_CONNECTION_LOST));
  EXPECT_STREQ("STOP_RESET_FAILED", mg400_interface::toString(Code::STOP_RESET_FAILED));
  EXPECT_STREQ(
    "STOP_CONFIRMATION_TIMEOUT",
    mg400_interface::toString(Code::STOP_CONFIRMATION_TIMEOUT));
  EXPECT_STREQ("INTERNAL_ERROR", mg400_interface::toString(Code::INTERNAL_ERROR));
  EXPECT_STREQ("UNKNOWN", mg400_interface::toString(static_cast<Code>(255)));
}

}  // namespace
