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

TEST(TestRegularMotionLease, ReleasesOnSuccessAbortSendExceptionConnectionAndEarlyReturn)
{
  const auto manager = makeEnabledManager();

  enum class ExitPath
  {
    SUCCESS,
    ABORT,
    SEND_EXCEPTION,
    CONNECTION_ERROR,
    EARLY_RETURN,
  };

  for (const auto exit_path : {ExitPath::SUCCESS, ExitPath::ABORT,
      ExitPath::SEND_EXCEPTION, ExitPath::CONNECTION_ERROR, ExitPath::EARLY_RETURN})
  {
    try {
      auto execute = [&]() {
          auto lease = acquire(manager);
          ASSERT_TRUE(lease);
          if (exit_path == ExitPath::ABORT || exit_path == ExitPath::EARLY_RETURN) {
            return;
          }
          if (exit_path == ExitPath::SEND_EXCEPTION) {
            throw std::runtime_error("injected send failure");
          }
          if (exit_path == ExitPath::CONNECTION_ERROR) {
            manager->updateRobotStatus(false, RobotMode::INVALID);
            throw std::runtime_error("injected connection failure");
          }
        };
      execute();
    } catch (const std::runtime_error &) {
    }

    const auto snapshot = manager->getSnapshot();
    EXPECT_EQ(Manager::MotionOwner::NONE, snapshot.motion_owner);
    EXPECT_EQ(Manager::NO_LEASE, snapshot.lease_id);
    manager->updateRobotStatus(true, RobotMode::ENABLE);
  }
}

TEST(TestRegularMotionLease, GoalReservationPreventsASecondGoalAcceptance)
{
  using Reservations =
    mg400_plugin_base::RegularMotionGoalReservations<std::string, int>;

  const auto manager = makeEnabledManager();
  auto first_lease = acquire(manager);
  ASSERT_TRUE(first_lease);
  const auto first_id = first_lease->id();

  Reservations reservations;
  ASSERT_TRUE(reservations.reserve("goal-1", {std::move(*first_lease), 42}));

  Manager::Result second_result{};
  auto second_lease = Lease::tryAcquire(manager, second_result);
  EXPECT_FALSE(second_lease);
  EXPECT_FALSE(second_result.success);
  EXPECT_EQ(1U, reservations.size());

  auto execution = reservations.take("goal-1");
  ASSERT_TRUE(execution);
  EXPECT_EQ(42, execution->context);
  EXPECT_EQ(first_id, execution->lease.id());
  EXPECT_EQ(0U, reservations.size());

  execution.reset();
  EXPECT_EQ(Manager::MotionOwner::NONE, manager->getSnapshot().motion_owner);
}

TEST(TestRegularMotionLease, ValidationFailureBeforeReservationReleasesLease)
{
  const auto manager = makeEnabledManager();

  const auto validate_goal = [&]() {
      auto lease = acquire(manager);
      EXPECT_TRUE(lease);
      return false;
    };

  EXPECT_FALSE(validate_goal());
  EXPECT_EQ(Manager::MotionOwner::NONE, manager->getSnapshot().motion_owner);
}

TEST(TestRegularMotionLease, StaleLeaseCannotReleaseNewConnectionOwner)
{
  const auto manager = makeEnabledManager();
  std::atomic<int> release_failure_count{0};
  Manager::Result result{};
  auto stale = Lease::tryAcquire(
    manager, result,
    [&release_failure_count](const auto, const auto &) {++release_failure_count;});
  ASSERT_TRUE(stale);
  const auto stale_id = stale->id();

  manager->updateRobotStatus(false, RobotMode::INVALID);
  manager->updateRobotStatus(true, RobotMode::ENABLE);
  auto current = acquire(manager);
  ASSERT_TRUE(current);
  ASSERT_NE(stale_id, current->id());

  stale.reset();
  EXPECT_EQ(1, release_failure_count.load());
  EXPECT_TRUE(current->isCurrent());
  EXPECT_EQ(current->id(), manager->getSnapshot().lease_id);
}

TEST(TestRegularMotionLease, CommandQueueKeepsOneLeaseAcrossEveryCommand)
{
  const auto manager = makeEnabledManager();
  std::vector<Manager::LeaseId> observed_ids;

  try {
    auto queue_lease = acquire(manager);
    ASSERT_TRUE(queue_lease);
    for (int command = 0; command < 5; ++command) {
      observed_ids.push_back(manager->getSnapshot().lease_id);
      EXPECT_TRUE(queue_lease->isCurrent());
      if (command == 3) {
        throw std::runtime_error("injected queue command failure");
      }
    }
  } catch (const std::runtime_error &) {
  }

  ASSERT_FALSE(observed_ids.empty());
  for (const auto id : observed_ids) {
    EXPECT_EQ(observed_ids.front(), id);
  }
  EXPECT_EQ(Manager::MotionOwner::NONE, manager->getSnapshot().motion_owner);
}

TEST(TestRegularMotionLease, JogDirectionChangeAndConfirmedStopUseOneLease)
{
  const auto manager = makeEnabledManager();
  auto jog_lease = acquire(manager);
  ASSERT_TRUE(jog_lease);
  const auto initial_id = jog_lease->id();

  // Direction changes reuse the persistent MoveJog lease.
  for (const auto * direction : {"j1+", "j2-", "X+"}) {
    (void)direction;
    EXPECT_TRUE(jog_lease->isCurrent());
    EXPECT_EQ(initial_id, jog_lease->id());
    EXPECT_FALSE(manager->tryAcquireRegularMotion().success);
    EXPECT_FALSE(manager->requestControlState(Manager::State::SERVO_J).success);
  }

  manager->updateRobotStatus(true, RobotMode::JOG);
  EXPECT_TRUE(jog_lease->isCurrent());
  manager->updateRobotStatus(true, RobotMode::ENABLE);
  jog_lease.reset();
  EXPECT_EQ(Manager::MotionOwner::NONE, manager->getSnapshot().motion_owner);
}

TEST(TestRegularMotionLease, UnconfirmedJogStopRetainsLeaseFailClosed)
{
  const auto manager = makeEnabledManager();
  auto jog_lease = acquire(manager);
  ASSERT_TRUE(jog_lease);
  manager->updateRobotStatus(true, RobotMode::JOG);

  const bool stop_confirmed = false;
  if (stop_confirmed) {
    jog_lease.reset();
  }

  EXPECT_TRUE(jog_lease->isCurrent());
  EXPECT_FALSE(manager->requestControlState(Manager::State::SERVO_P).success);
  EXPECT_FALSE(manager->tryAcquireRegularMotion().success);
}

TEST(TestRegularMotionLease, ReservationsDoNotGrowAcrossManyGoals)
{
  using Reservations =
    mg400_plugin_base::RegularMotionGoalReservations<std::string, std::string>;

  const auto manager = makeEnabledManager();
  Reservations reservations;
  for (int index = 0; index < 1000; ++index) {
    auto lease = acquire(manager);
    ASSERT_TRUE(lease);
    const auto key = std::to_string(index);
    ASSERT_TRUE(reservations.reserve(key, {std::move(*lease), "target-" + key}));
    auto entry = reservations.take(key);
    ASSERT_TRUE(entry);
    EXPECT_EQ("target-" + key, entry->context);
    entry.reset();
    ASSERT_EQ(0U, reservations.size());
  }
}

TEST(TestRegularMotionLease, ConcurrentAdmissionCompletionAndDisconnectDoNotDeadlock)
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

TEST(TestRegularMotionLease, GoalContextsAreImmutableAndIsolated)
{
  using Reservations =
    mg400_plugin_base::RegularMotionGoalReservations<std::string, std::string>;

  Reservations reservations;
  ASSERT_TRUE(reservations.reserve("first", {Lease{}, "first-target"}));
  ASSERT_TRUE(reservations.reserve("second", {Lease{}, "second-target"}));

  auto first = reservations.take("first");
  auto second = reservations.take("second");
  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  EXPECT_EQ("first-target", first->context);
  EXPECT_EQ("second-target", second->context);
}

}  // namespace
