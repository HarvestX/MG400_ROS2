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
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <mg400_msgs/msg/robot_mode.hpp>

#include "mg400_interface/control_state_manager.hpp"

namespace
{

using Manager = mg400_interface::ControlStateManager;
using RobotMode = mg400_msgs::msg::RobotMode;

TEST(TestControlStateManager, StartsUnavailable)
{
  Manager manager;
  const auto snapshot = manager.getSnapshot();
  EXPECT_EQ(Manager::State::UNAVAILABLE, snapshot.control_state);
  EXPECT_EQ(Manager::NO_LEASE, snapshot.lease_id);
  EXPECT_FALSE(snapshot.connected);
}

TEST(TestControlStateManager, BecomesIdleOnlyWhenConnectedAndEnabled)
{
  Manager manager;
  EXPECT_EQ(
    Manager::State::UNAVAILABLE,
    manager.updateRobotStatus(false, RobotMode::ENABLE).control_state);
  EXPECT_EQ(
    Manager::State::UNAVAILABLE,
    manager.updateRobotStatus(true, RobotMode::DISABLED).control_state);
  EXPECT_EQ(
    Manager::State::IDLE,
    manager.updateRobotStatus(true, RobotMode::ENABLE).control_state);
  EXPECT_EQ(
    Manager::State::UNAVAILABLE,
    manager.updateRobotStatus(true, RobotMode::ERROR).control_state);
}

TEST(TestControlStateManager, AcquireUsesOneAtomicPathForEveryMotionType)
{
  for (const auto state : {Manager::State::REGULAR_MOTION, Manager::State::SERVO_J}) {
    Manager manager;
    manager.updateRobotStatus(true, RobotMode::ENABLE);
    const auto acquired = manager.tryAcquire(state);
    ASSERT_TRUE(acquired.success) << acquired.message;
    EXPECT_EQ(state, acquired.current_state);
    EXPECT_NE(Manager::NO_LEASE, acquired.lease_id);
    EXPECT_TRUE(manager.owns(state, acquired.lease_id));
  }
}

TEST(TestControlStateManager, RejectsNonOwningAcquireTargets)
{
  Manager manager;
  manager.updateRobotStatus(true, RobotMode::ENABLE);
  EXPECT_FALSE(manager.tryAcquire(Manager::State::IDLE).success);
  EXPECT_FALSE(manager.tryAcquire(Manager::State::UNAVAILABLE).success);
  EXPECT_EQ(Manager::State::IDLE, manager.getState());
}

TEST(TestControlStateManager, ServoLeaseSurvivesEnableAndRunning)
{
  Manager manager;
  manager.updateRobotStatus(true, RobotMode::ENABLE);
  const auto started = manager.tryAcquire(Manager::State::SERVO_J);
  ASSERT_TRUE(started.success) << started.message;
  EXPECT_TRUE(manager.acceptsServoJTarget(started.lease_id));

  EXPECT_EQ(
    Manager::State::SERVO_J,
    manager.updateRobotStatus(true, RobotMode::RUNNING).control_state);
  EXPECT_TRUE(manager.acceptsServoJTarget(started.lease_id));
  EXPECT_EQ(
    Manager::State::SERVO_J,
    manager.updateRobotStatus(true, RobotMode::ENABLE).control_state);
}

TEST(TestControlStateManager, UnsafeRobotModeRevokesOwnership)
{
  Manager manager;
  manager.updateRobotStatus(true, RobotMode::ENABLE);
  const auto started = manager.tryAcquire(Manager::State::SERVO_J);
  ASSERT_TRUE(started.success) << started.message;

  const auto snapshot = manager.updateRobotStatus(true, RobotMode::ERROR);
  EXPECT_EQ(Manager::State::UNAVAILABLE, snapshot.control_state);
  EXPECT_EQ(Manager::NO_LEASE, snapshot.lease_id);
  EXPECT_FALSE(manager.acceptsServoJTarget(started.lease_id));
}

TEST(TestControlStateManager, ServoReleaseRequiresStopAndMatchingLease)
{
  Manager manager;
  manager.updateRobotStatus(true, RobotMode::ENABLE);
  const auto started = manager.tryAcquire(Manager::State::SERVO_J);
  ASSERT_TRUE(started.success) << started.message;

  EXPECT_FALSE(manager.release(Manager::State::SERVO_J, started.lease_id).success);
  EXPECT_FALSE(manager.beginServoStop(started.lease_id + 1).success);
  ASSERT_TRUE(manager.beginServoStop(started.lease_id).success);
  EXPECT_FALSE(manager.acceptsServoJTarget(started.lease_id));
  EXPECT_TRUE(manager.release(Manager::State::SERVO_J, started.lease_id).success);
  EXPECT_EQ(Manager::State::IDLE, manager.getState());
}

TEST(TestControlStateManager, RegularMotionAndServoAreMutuallyExclusive)
{
  Manager manager;
  manager.updateRobotStatus(true, RobotMode::ENABLE);

  const auto regular = manager.tryAcquire(Manager::State::REGULAR_MOTION);
  ASSERT_TRUE(regular.success) << regular.message;
  EXPECT_EQ(Manager::State::REGULAR_MOTION, manager.getState());
  EXPECT_FALSE(manager.tryAcquire(Manager::State::SERVO_J).success);

  ASSERT_TRUE(
    manager.release(Manager::State::REGULAR_MOTION, regular.lease_id).success);
  const auto servo = manager.tryAcquire(Manager::State::SERVO_J);
  ASSERT_TRUE(servo.success) << servo.message;
  EXPECT_FALSE(manager.tryAcquire(Manager::State::REGULAR_MOTION).success);
}

TEST(TestControlStateManager, AcquiresAndReleasesRegularMotionByLease)
{
  Manager manager;
  manager.updateRobotStatus(true, RobotMode::ENABLE);

  const auto acquired = manager.tryAcquire(Manager::State::REGULAR_MOTION);
  ASSERT_TRUE(acquired.success) << acquired.message;
  EXPECT_EQ(Manager::State::REGULAR_MOTION, manager.getState());
  EXPECT_FALSE(manager.tryAcquire(Manager::State::REGULAR_MOTION).success);

  ASSERT_TRUE(
    manager.release(Manager::State::REGULAR_MOTION, acquired.lease_id).success);
  EXPECT_EQ(Manager::State::IDLE, manager.getState());
  EXPECT_FALSE(
    manager.release(Manager::State::REGULAR_MOTION, acquired.lease_id).success);
}

TEST(TestControlStateManager, RegularMotionStateSurvivesEmbeddedModes)
{
  Manager manager;
  manager.updateRobotStatus(true, RobotMode::ENABLE);
  const auto regular = manager.tryAcquire(Manager::State::REGULAR_MOTION);
  ASSERT_TRUE(regular.success) << regular.message;

  for (const auto mode : {RobotMode::RUNNING, RobotMode::PAUSE, RobotMode::JOG,
      RobotMode::ENABLE})
  {
    const auto snapshot = manager.updateRobotStatus(true, mode);
    EXPECT_EQ(Manager::State::REGULAR_MOTION, snapshot.control_state);
    EXPECT_EQ(regular.lease_id, snapshot.lease_id);
  }

  EXPECT_TRUE(
    manager.release(Manager::State::REGULAR_MOTION, regular.lease_id).success);
  EXPECT_EQ(Manager::State::IDLE, manager.getState());
}

TEST(TestControlStateManager, OnlyOneConcurrentCallerAcquiresOwnership)
{
  Manager manager;
  manager.updateRobotStatus(true, RobotMode::ENABLE);

  constexpr std::size_t thread_count = 16;
  std::atomic<std::size_t> success_count{0};
  std::vector<std::thread> threads;
  for (std::size_t index = 0; index < thread_count; ++index) {
    threads.emplace_back(
      [&manager, &success_count, index]() {
        const auto state = index % 2 == 0 ?
        Manager::State::REGULAR_MOTION : Manager::State::SERVO_J;
        if (manager.tryAcquire(state).success) {
          ++success_count;
        }
      });
  }
  for (auto & thread : threads) {
    thread.join();
  }
  EXPECT_EQ(1U, success_count.load());
}

TEST(TestControlStateManager, StaleLeaseCannotReleaseNewOwner)
{
  Manager manager;
  manager.updateRobotStatus(true, RobotMode::ENABLE);
  const auto first = manager.tryAcquire(Manager::State::REGULAR_MOTION);
  ASSERT_TRUE(first.success) << first.message;

  manager.updateRobotStatus(false, RobotMode::INVALID);
  manager.updateRobotStatus(true, RobotMode::ENABLE);
  const auto second = manager.tryAcquire(Manager::State::REGULAR_MOTION);
  ASSERT_TRUE(second.success) << second.message;
  ASSERT_NE(first.lease_id, second.lease_id);

  EXPECT_FALSE(
    manager.release(Manager::State::REGULAR_MOTION, first.lease_id).success);
  EXPECT_TRUE(manager.owns(Manager::State::REGULAR_MOTION, second.lease_id));
}

}  // namespace
