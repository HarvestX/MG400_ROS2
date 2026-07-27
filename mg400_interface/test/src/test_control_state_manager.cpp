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
  EXPECT_EQ(Manager::MotionOwner::NONE, snapshot.motion_owner);
  EXPECT_EQ(Manager::NO_LEASE, snapshot.lease_id);
  EXPECT_FALSE(snapshot.connected);
  EXPECT_FALSE(snapshot.accepting_servo_targets);
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

TEST(TestControlStateManager, ServoLeaseSurvivesEnableAndRunning)
{
  Manager manager;
  manager.updateRobotStatus(true, RobotMode::ENABLE);

  const auto started = manager.requestControlState(Manager::State::SERVO_J);
  ASSERT_TRUE(started.success) << started.message;
  ASSERT_NE(Manager::NO_LEASE, started.lease_id);
  EXPECT_TRUE(manager.acceptsServoTarget(Manager::State::SERVO_J, started.lease_id));
  EXPECT_FALSE(manager.acceptsServoTarget(Manager::State::SERVO_P, started.lease_id));

  auto snapshot = manager.updateRobotStatus(true, RobotMode::RUNNING);
  EXPECT_EQ(Manager::State::SERVO_J, snapshot.control_state);
  EXPECT_EQ(Manager::MotionOwner::SERVO_J, snapshot.motion_owner);
  EXPECT_EQ(started.lease_id, snapshot.lease_id);
  EXPECT_TRUE(manager.acceptsServoTarget(Manager::State::SERVO_J, started.lease_id));

  snapshot = manager.updateRobotStatus(true, RobotMode::ENABLE);
  EXPECT_EQ(Manager::State::SERVO_J, snapshot.control_state);
  EXPECT_EQ(started.lease_id, snapshot.lease_id);
}

TEST(TestControlStateManager, UnsafeRobotModeRevokesServoOwnership)
{
  Manager manager;
  manager.updateRobotStatus(true, RobotMode::ENABLE);
  const auto started = manager.requestControlState(Manager::State::SERVO_P);
  ASSERT_TRUE(started.success) << started.message;

  const auto snapshot = manager.updateRobotStatus(true, RobotMode::PAUSE);
  EXPECT_EQ(Manager::State::UNAVAILABLE, snapshot.control_state);
  EXPECT_EQ(Manager::MotionOwner::NONE, snapshot.motion_owner);
  EXPECT_EQ(Manager::NO_LEASE, snapshot.lease_id);
  EXPECT_FALSE(manager.acceptsServoTarget(Manager::State::SERVO_P, started.lease_id));
}

TEST(TestControlStateManager, ServoStopRequiresTheActiveLease)
{
  Manager manager;
  manager.updateRobotStatus(true, RobotMode::ENABLE);
  const auto started = manager.requestControlState(Manager::State::SERVO_J);
  ASSERT_TRUE(started.success) << started.message;

  const auto wrong_lease = manager.requestControlState(
    Manager::State::IDLE, started.lease_id + 1);
  EXPECT_FALSE(wrong_lease.success);
  EXPECT_EQ(Manager::State::SERVO_J, wrong_lease.current_state);

  const auto stop_not_begun = manager.requestControlState(
    Manager::State::IDLE, started.lease_id);
  EXPECT_FALSE(stop_not_begun.success);
  EXPECT_TRUE(manager.acceptsServoTarget(Manager::State::SERVO_J, started.lease_id));

  const auto begin_stop = manager.beginServoStop(started.lease_id);
  ASSERT_TRUE(begin_stop.success) << begin_stop.message;
  EXPECT_FALSE(manager.acceptsServoTarget(Manager::State::SERVO_J, started.lease_id));
  EXPECT_EQ(Manager::MotionOwner::SERVO_J, manager.getSnapshot().motion_owner);

  const auto stopped = manager.requestControlState(
    Manager::State::IDLE, started.lease_id);
  EXPECT_TRUE(stopped.success) << stopped.message;
  EXPECT_EQ(Manager::State::IDLE, stopped.current_state);
  EXPECT_EQ(Manager::NO_LEASE, stopped.lease_id);
}

TEST(TestControlStateManager, RejectsDirectServoModeSwitch)
{
  Manager manager;
  manager.updateRobotStatus(true, RobotMode::ENABLE);
  const auto started = manager.requestControlState(Manager::State::SERVO_J);
  ASSERT_TRUE(started.success) << started.message;

  const auto switched = manager.requestControlState(Manager::State::SERVO_P);
  EXPECT_FALSE(switched.success);
  EXPECT_EQ(Manager::State::SERVO_J, switched.current_state);
  EXPECT_EQ(started.lease_id, switched.lease_id);
}

TEST(TestControlStateManager, WatchdogReleasesServoOwnership)
{
  Manager manager;
  manager.updateRobotStatus(true, RobotMode::ENABLE);
  const auto started = manager.requestControlState(Manager::State::SERVO_P);
  ASSERT_TRUE(started.success) << started.message;

  ASSERT_TRUE(manager.beginServoStop(started.lease_id).success);
  const auto timed_out = manager.handleServoWatchdogTimeout(started.lease_id);
  EXPECT_TRUE(timed_out.success) << timed_out.message;
  EXPECT_EQ(Manager::State::IDLE, timed_out.current_state);
  EXPECT_EQ(Manager::NO_LEASE, timed_out.lease_id);
}

TEST(TestControlStateManager, RegularMotionAndServoAreMutuallyExclusive)
{
  Manager manager;
  manager.updateRobotStatus(true, RobotMode::ENABLE);

  const auto regular = manager.tryAcquireRegularMotion();
  ASSERT_TRUE(regular.success) << regular.message;
  ASSERT_NE(Manager::NO_LEASE, regular.lease_id);

  const auto servo_while_regular = manager.requestControlState(Manager::State::SERVO_J);
  EXPECT_FALSE(servo_while_regular.success);

  EXPECT_TRUE(manager.releaseRegularMotion(regular.lease_id).success);
  const auto servo = manager.requestControlState(Manager::State::SERVO_J);
  ASSERT_TRUE(servo.success) << servo.message;

  const auto regular_while_servo = manager.tryAcquireRegularMotion();
  EXPECT_FALSE(regular_while_servo.success);
}

TEST(TestControlStateManager, RegularMotionLeaseSurvivesItsEmbeddedModes)
{
  Manager manager;
  manager.updateRobotStatus(true, RobotMode::ENABLE);
  const auto regular = manager.tryAcquireRegularMotion();
  ASSERT_TRUE(regular.success) << regular.message;

  for (const auto mode : {RobotMode::RUNNING, RobotMode::PAUSE, RobotMode::JOG,
      RobotMode::ENABLE})
  {
    const auto snapshot = manager.updateRobotStatus(true, mode);
    EXPECT_EQ(Manager::State::IDLE, snapshot.control_state);
    EXPECT_EQ(Manager::MotionOwner::REGULAR_MOTION, snapshot.motion_owner);
    EXPECT_EQ(regular.lease_id, snapshot.lease_id);
  }

  const auto released = manager.releaseRegularMotion(regular.lease_id);
  EXPECT_TRUE(released.success) << released.message;
  EXPECT_EQ(Manager::State::IDLE, released.current_state);
}

TEST(TestControlStateManager, OnlyOneConcurrentCallerAcquiresOwnership)
{
  Manager manager;
  manager.updateRobotStatus(true, RobotMode::ENABLE);

  constexpr std::size_t thread_count = 16;
  std::atomic<std::size_t> success_count{0};
  std::vector<std::thread> threads;
  threads.reserve(thread_count);

  for (std::size_t index = 0; index < thread_count; ++index) {
    threads.emplace_back(
      [&manager, &success_count]() {
        if (manager.tryAcquireRegularMotion().success) {
          ++success_count;
        }
      });
  }
  for (auto & thread : threads) {
    thread.join();
  }

  EXPECT_EQ(1U, success_count.load());
}

TEST(TestControlStateManager, BothServoOwnersRejectRegularMotion)
{
  for (const auto servo_state : {Manager::State::SERVO_J, Manager::State::SERVO_P}) {
    Manager manager;
    manager.updateRobotStatus(true, RobotMode::ENABLE);
    const auto servo = manager.requestControlState(servo_state);
    ASSERT_TRUE(servo.success) << servo.message;
    EXPECT_FALSE(manager.tryAcquireRegularMotion().success);
  }
}

TEST(TestControlStateManager, ForeignAndStaleLeaseCannotReleaseCurrentOwnership)
{
  Manager manager;
  manager.updateRobotStatus(true, RobotMode::ENABLE);
  const auto first = manager.tryAcquireRegularMotion();
  ASSERT_TRUE(first.success) << first.message;

  EXPECT_FALSE(manager.releaseRegularMotion(first.lease_id + 1).success);
  EXPECT_EQ(first.lease_id, manager.getSnapshot().lease_id);

  manager.updateRobotStatus(false, RobotMode::INVALID);
  manager.updateRobotStatus(true, RobotMode::ENABLE);
  const auto second = manager.tryAcquireRegularMotion();
  ASSERT_TRUE(second.success) << second.message;
  ASSERT_NE(first.lease_id, second.lease_id);

  EXPECT_FALSE(manager.releaseRegularMotion(first.lease_id).success);
  EXPECT_EQ(second.lease_id, manager.getSnapshot().lease_id);
}

}  // namespace
