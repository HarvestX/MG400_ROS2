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
#include <future>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <mg400_msgs/msg/robot_mode.hpp>

#include "mg400_plugin_base/regular_motion_action_reservations.hpp"
#include "mg400_plugin_base/regular_motion_lease.hpp"

namespace
{

using Manager = mg400_interface::ControlStateManager;
using Lease = mg400_plugin_base::RegularMotionLease;
using RobotMode = mg400_msgs::msg::RobotMode;

Manager::SharedPtr makeEnabledManager()
{
  auto manager = std::make_shared<Manager>();
  manager->updateRobotStatus(true, RobotMode::ENABLE);
  return manager;
}

std::optional<Lease> acquire(const Manager::SharedPtr & manager)
{
  Manager::Result result{};
  auto lease = Lease::tryAcquire(manager, result);
  EXPECT_TRUE(result.success) << result.message;
  return lease;
}

TEST(TestRegularMotionLease, ReleasesOnEveryScopeExit)
{
  const auto manager = makeEnabledManager();

  enum class ExitPath {SUCCESS, EARLY_RETURN, EXCEPTION, DISCONNECT};
  for (const auto exit_path : {
      ExitPath::SUCCESS, ExitPath::EARLY_RETURN,
      ExitPath::EXCEPTION, ExitPath::DISCONNECT})
  {
    try {
      auto execute = [&]() {
          auto lease = acquire(manager);
          ASSERT_TRUE(lease);
          if (exit_path == ExitPath::EARLY_RETURN) {
            return;
          }
          if (exit_path == ExitPath::EXCEPTION) {
            throw std::runtime_error("injected failure");
          }
          if (exit_path == ExitPath::DISCONNECT) {
            manager->updateRobotStatus(false, RobotMode::INVALID);
          }
        };
      execute();
    } catch (const std::runtime_error &) {
    }

    EXPECT_EQ(Manager::NO_LEASE, manager->getSnapshot().lease_id);
    manager->updateRobotStatus(true, RobotMode::ENABLE);
  }
}

TEST(TestRegularMotionLease, StaleLeaseCannotReleaseNewOwner)
{
  const auto manager = makeEnabledManager();
  std::atomic<int> release_failure_count{0};
  Manager::Result result{};
  auto stale = Lease::tryAcquire(
    manager, result,
    [&release_failure_count](const auto, const auto &) {++release_failure_count;});
  ASSERT_TRUE(stale);

  manager->updateRobotStatus(false, RobotMode::INVALID);
  manager->updateRobotStatus(true, RobotMode::ENABLE);
  auto current = acquire(manager);
  ASSERT_TRUE(current);
  ASSERT_NE(stale->id(), current->id());

  stale.reset();
  EXPECT_EQ(1, release_failure_count.load());
  EXPECT_TRUE(current->isCurrent());
  EXPECT_EQ(current->id(), manager->getSnapshot().lease_id);
}

TEST(TestRegularMotionLease, ActionReservationTransfersLease)
{
  using Reservations = mg400_plugin_base::RegularMotionActionReservations<std::string>;

  const auto manager = makeEnabledManager();
  auto lease = acquire(manager);
  ASSERT_TRUE(lease);
  const auto lease_id = lease->id();

  Reservations reservations;
  ASSERT_TRUE(reservations.reserve("goal-1", std::move(*lease)));
  EXPECT_EQ(1U, reservations.size());
  EXPECT_FALSE(manager->tryAcquire(Manager::State::REGULAR_MOTION).success);

  auto execution = reservations.take("goal-1");
  ASSERT_TRUE(execution);
  EXPECT_EQ(lease_id, execution->id());
  EXPECT_TRUE(execution->isCurrent());
  EXPECT_EQ(0U, reservations.size());

  execution.reset();
  EXPECT_EQ(Manager::State::IDLE, manager->getState());
}

TEST(TestRegularMotionLease, DuplicateActionGoalDoesNotReplaceReservation)
{
  using Reservations = mg400_plugin_base::RegularMotionActionReservations<std::string>;

  Reservations reservations;
  ASSERT_TRUE(reservations.reserve("goal", Lease{}));
  EXPECT_FALSE(reservations.reserve("goal", Lease{}));

  auto reserved = reservations.take("goal");
  ASSERT_TRUE(reserved);
}

TEST(TestRegularMotionLease, UnconfirmedJogStopRetainsOwnershipFailClosed)
{
  const auto manager = makeEnabledManager();
  auto lease = acquire(manager);
  ASSERT_TRUE(lease);
  manager->updateRobotStatus(true, RobotMode::JOG);

  lease->retainManagerOwnership();
  lease.reset();

  EXPECT_EQ(Manager::State::REGULAR_MOTION, manager->getState());
  EXPECT_FALSE(manager->tryAcquire(Manager::State::SERVO_J).success);
  EXPECT_FALSE(manager->tryAcquire(Manager::State::REGULAR_MOTION).success);
}

TEST(TestRegularMotionLease, ConcurrentAdmissionAndStatusUpdatesDoNotDeadlock)
{
  const auto manager = makeEnabledManager();
  std::promise<void> start_promise;
  auto start = start_promise.get_future().share();
  std::atomic<bool> stop{false};

  auto admission = std::async(
    std::launch::async,
    [&]() {
      start.wait();
      while (!stop.load()) {
        Manager::Result result{};
        auto lease = Lease::tryAcquire(manager, result);
      }
    });
  auto status = std::async(
    std::launch::async,
    [&]() {
      start.wait();
      for (int index = 0; index < 1000; ++index) {
        manager->updateRobotStatus(false, RobotMode::INVALID);
        manager->updateRobotStatus(true, RobotMode::ENABLE);
      }
      stop.store(true);
    });

  start_promise.set_value();
  EXPECT_EQ(std::future_status::ready, status.wait_for(std::chrono::seconds(2)));
  stop.store(true);
  EXPECT_EQ(std::future_status::ready, admission.wait_for(std::chrono::seconds(2)));
}

}  // namespace
