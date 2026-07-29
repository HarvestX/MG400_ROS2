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
#include <chrono>
#include <limits>
#include <thread>

#include <gtest/gtest.h>

#include "mg400_interface/servo_feedback_state.hpp"

namespace
{
using namespace std::chrono_literals;  // NOLINT
using FeedbackState = mg400_interface::ServoFeedbackState;

TEST(ServoFeedbackState, StartsUnavailableAndUsesInclusiveTimeoutBoundary)
{
  FeedbackState state(7);
  const auto initial = state.getSnapshot();
  EXPECT_FALSE(initial.has_feedback);
  EXPECT_EQ(7U, initial.connection_epoch);

  const auto received_at = FeedbackState::Clock::now();
  state.update({{1.0, 2.0, 3.0, 4.0}}, received_at);
  const auto snapshot = state.getSnapshot();

  EXPECT_TRUE(snapshot.isFresh(7, 100ms, received_at + 99ms));
  EXPECT_TRUE(snapshot.isFresh(7, 100ms, received_at + 100ms));
  EXPECT_FALSE(snapshot.isFresh(7, 100ms, received_at + 100ms + 1ns));
  EXPECT_FALSE(snapshot.isFresh(8, 100ms, received_at));
  EXPECT_EQ((std::array<double, 4>{{1.0, 2.0, 3.0, 4.0}}), snapshot.joint_angles_rad);
}

TEST(ServoFeedbackState, NewConnectionEpochInvalidatesTheOldFrame)
{
  FeedbackState state(11);
  state.update({{1.0, 1.0, 1.0, 1.0}});
  ASSERT_TRUE(state.getSnapshot().has_feedback);

  state.beginConnectionEpoch(12);
  auto snapshot = state.getSnapshot();
  EXPECT_FALSE(snapshot.has_feedback);
  EXPECT_EQ(12U, snapshot.connection_epoch);
  EXPECT_FALSE(snapshot.isFresh(11, 1s, FeedbackState::Clock::now()));

  state.update({{2.0, 2.0, 2.0, 2.0}});
  snapshot = state.getSnapshot();
  EXPECT_TRUE(snapshot.has_feedback);
  EXPECT_EQ(12U, snapshot.connection_epoch);
  state.invalidate();
  EXPECT_FALSE(state.getSnapshot().has_feedback);
}

TEST(ServoFeedbackState, NonFiniteFrameInvalidatesTheSnapshot)
{
  FeedbackState state(13);
  state.update({{0.0, 0.0, 0.2, 0.0}});
  ASSERT_TRUE(state.getSnapshot().has_feedback);

  state.update({{0.0, std::numeric_limits<double>::quiet_NaN(), 0.2, 0.0}});
  EXPECT_FALSE(state.getSnapshot().has_feedback);
}

TEST(ServoFeedbackState, ConcurrentFramesRemainInternallyConsistent)
{
  FeedbackState state(21);
  std::atomic<bool> done{false};
  std::atomic<bool> inconsistent{false};

  std::thread writer([&state, &done]() {
      for (int frame = 1; frame <= 10000; ++frame) {
        const double value = static_cast<double>(frame);
        state.update({{value, value, value, value}});
      }
      done.store(true);
    });
  std::thread reader([&state, &done, &inconsistent]() {
      while (!done.load()) {
        const auto snapshot = state.getSnapshot();
        if (!snapshot.has_feedback) {
          continue;
        }
        const double value = snapshot.joint_angles_rad[0];
        for (const double joint : snapshot.joint_angles_rad) {
          inconsistent.store(inconsistent.load() || joint != value);
        }
      }
    });

  writer.join();
  reader.join();
  EXPECT_FALSE(inconsistent.load());
}

}  // namespace
