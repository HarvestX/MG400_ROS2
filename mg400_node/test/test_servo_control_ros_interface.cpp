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
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <mg400_msgs/msg/robot_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "mg400_interface/commander/motion_commander.hpp"
#include "mg400_interface/servo_stop_strategy.hpp"
#include "mg400_node/servo_control_ros_interface.hpp"

namespace
{

using namespace std::chrono_literals;  // NOLINT
using Manager = mg400_interface::ControlStateManager;
using Session = mg400_interface::ServoControlSession;
using StopStrategy = mg400_interface::ServoStopStrategy;
using RosInterface = mg400_node::ServoControlRosInterface;
using ChangeControlState = mg400_msgs::srv::ChangeControlState;
using RobotMode = mg400_msgs::msg::RobotMode;

class RosEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

const auto ros_environment = ::testing::AddGlobalTestEnvironment(new RosEnvironment);

class FakeMotionTcpInterface : public mg400_interface::MotionTcpInterfaceBase
{
public:
  void sendCommand(const std::string & command) override
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->commands_.push_back(command);
    }
    this->cv_.notify_all();
  }

  bool waitForCommandCount(
    const std::size_t count, const std::chrono::milliseconds timeout = 500ms)
  {
    std::unique_lock<std::mutex> lock(this->mutex_);
    return this->cv_.wait_for(
      lock, timeout, [this, count]() {return this->commands_.size() >= count;});
  }

  std::vector<std::string> commands() const
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->commands_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::vector<std::string> commands_;
};

class SequenceStopStrategy : public StopStrategy
{
public:
  Result stop(const Clock::time_point &) override
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    ++this->call_count_;
    if (this->results_.empty()) {
      return Result{Status::SUCCESS, "stop confirmed"};
    }
    const auto result = this->results_.front();
    this->results_.pop_front();
    return result;
  }

  void push(const Status status, const std::string & message)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->results_.push_back(Result{status, message});
  }

  std::size_t callCount() const
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->call_count_;
  }

private:
  mutable std::mutex mutex_;
  std::deque<Result> results_;
  std::size_t call_count_{0};
};

template<typename Predicate>
bool waitUntil(Predicate predicate, const std::chrono::milliseconds timeout = 1s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(2ms);
  }
  return predicate();
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> makeNode()
{
  static std::atomic<unsigned int> sequence{0};
  return std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "servo_ros_interface_test_" + std::to_string(++sequence));
}

RosInterface::Options rosOptions()
{
  RosInterface::Options options;
  options.diagnostics_period = 20ms;
  options.target_frame = "robot_mg400_origin_link";
  options.hardware_id = "test_mg400";
  return options;
}

Session::Options sessionOptions()
{
  Session::Options options;
  options.send_period = 10ms;
  options.target_watchdog_timeout = 5s;
  options.response_poll_period = 2ms;
  options.stop_confirmation_timeout = 50ms;
  return options;
}

struct Fixture
{
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node{makeNode()};
  Manager::SharedPtr manager{std::make_shared<Manager>()};
  FakeMotionTcpInterface tcp;
  mg400_interface::MotionCommander::SharedPtr commander{
    std::make_shared<mg400_interface::MotionCommander>(&tcp)};
  std::shared_ptr<SequenceStopStrategy> stop_strategy{
    std::make_shared<SequenceStopStrategy>()};
  std::shared_ptr<Session> session;
  std::unique_ptr<RosInterface> ros_interface;

  explicit Fixture(
    RosInterface::TransformPoseFunction transform =
    [] (const auto & input, const auto &, auto & output, auto & reason) {
      output = input;
      reason.clear();
      return true;
    })
  {
    manager->updateRobotStatus(true, RobotMode::ENABLE);
    session = std::make_shared<Session>(
      manager, commander, stop_strategy, sessionOptions());
    ros_interface = std::make_unique<RosInterface>(
      *node, manager, rosOptions(), std::move(transform));
    ros_interface->installSession(session);
  }

  ~Fixture()
  {
    if (ros_interface && ros_interface->hasSession()) {
      ros_interface->clearSession("test teardown");
    }
  }
};

std::shared_ptr<ChangeControlState::Response> requestState(
  RosInterface & ros_interface,
  const Manager::State state,
  const Manager::LeaseId lease_id)
{
  auto request = std::make_shared<ChangeControlState::Request>();
  request->target_state.control_state = static_cast<std::uint8_t>(state);
  request->lease_id = lease_id;
  auto response = std::make_shared<ChangeControlState::Response>();
  ros_interface.handleChangeControlState(request, response);
  return response;
}

geometry_msgs::msg::Quaternion yawQuaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw / 2.0);
  quaternion.w = std::cos(yaw / 2.0);
  return quaternion;
}

std::string diagnosticValue(
  const diagnostic_msgs::msg::DiagnosticArray & array,
  const std::string & key)
{
  if (array.status.empty()) {
    return "";
  }
  for (const auto & item : array.status.front().values) {
    if (item.key == key) {
      return item.value;
    }
  }
  return "";
}

TEST(ServoControlRosInterface, EnforcesLeaseServiceContractAndAllowsStopRetry)
{
  Fixture fixture;

  const auto invalid_start = requestState(
    *fixture.ros_interface, Manager::State::SERVO_J, 42);
  EXPECT_FALSE(invalid_start->success);
  EXPECT_EQ(Manager::NO_LEASE, invalid_start->lease_id);
  EXPECT_EQ(
    static_cast<std::uint8_t>(Manager::State::IDLE),
    invalid_start->current_state.control_state);

  const auto started = requestState(
    *fixture.ros_interface, Manager::State::SERVO_J, Manager::NO_LEASE);
  ASSERT_TRUE(started->success) << started->message;
  ASSERT_NE(Manager::NO_LEASE, started->lease_id);
  const auto lease = started->lease_id;

  const auto direct_switch = requestState(
    *fixture.ros_interface, Manager::State::SERVO_P, Manager::NO_LEASE);
  EXPECT_FALSE(direct_switch->success);
  EXPECT_EQ(Manager::NO_LEASE, direct_switch->lease_id);

  const auto foreign_stop = requestState(
    *fixture.ros_interface, Manager::State::IDLE, lease + 1);
  EXPECT_FALSE(foreign_stop->success);
  EXPECT_EQ(Manager::NO_LEASE, foreign_stop->lease_id);
  EXPECT_EQ(lease, fixture.manager->getSnapshot().lease_id);

  const auto zero_stop = requestState(
    *fixture.ros_interface, Manager::State::IDLE, Manager::NO_LEASE);
  EXPECT_FALSE(zero_stop->success);

  fixture.stop_strategy->push(
    StopStrategy::Status::CONFIRMATION_TIMEOUT, "ENABLE confirmation timed out");
  fixture.stop_strategy->push(StopStrategy::Status::SUCCESS, "retry confirmed");
  const auto failed_stop = requestState(
    *fixture.ros_interface, Manager::State::IDLE, lease);
  EXPECT_FALSE(failed_stop->success);
  EXPECT_EQ(Manager::NO_LEASE, failed_stop->lease_id);
  EXPECT_EQ(lease, fixture.manager->getSnapshot().lease_id);
  EXPECT_FALSE(fixture.manager->getSnapshot().accepting_servo_targets);

  const auto retried_stop = requestState(
    *fixture.ros_interface, Manager::State::IDLE, lease);
  EXPECT_TRUE(retried_stop->success) << retried_stop->message;
  EXPECT_EQ(Manager::NO_LEASE, retried_stop->lease_id);
  EXPECT_EQ(
    static_cast<std::uint8_t>(Manager::State::IDLE),
    retried_stop->current_state.control_state);
  EXPECT_EQ(Manager::NO_LEASE, fixture.manager->getSnapshot().lease_id);
  EXPECT_EQ(2U, fixture.stop_strategy->callCount());

  const auto stale_stop = requestState(
    *fixture.ros_interface, Manager::State::IDLE, lease);
  EXPECT_FALSE(stale_stop->success);
  EXPECT_EQ(Manager::NO_LEASE, stale_stop->lease_id);
}

TEST(ServoControlRosInterface, ServoJRejectsWrongLeaseKindAndNonFiniteTarget)
{
  Fixture fixture;
  const auto started = requestState(
    *fixture.ros_interface, Manager::State::SERVO_J, Manager::NO_LEASE);
  ASSERT_TRUE(started->success);

  auto wrong_lease = std::make_shared<mg400_msgs::msg::ServoJ>();
  wrong_lease->lease_id = started->lease_id + 1;
  wrong_lease->joint_angles = {{0.1, 0.2, 0.3, 0.4}};
  fixture.ros_interface->handleServoJTarget(wrong_lease);

  auto non_finite = std::make_shared<mg400_msgs::msg::ServoJ>(*wrong_lease);
  non_finite->lease_id = started->lease_id;
  non_finite->joint_angles[2] = std::numeric_limits<double>::infinity();
  fixture.ros_interface->handleServoJTarget(non_finite);

  auto wrong_kind = std::make_shared<mg400_msgs::msg::ServoP>();
  wrong_kind->lease_id = started->lease_id;
  wrong_kind->pose.header.frame_id = "world";
  wrong_kind->pose.pose.orientation.w = 1.0;
  fixture.ros_interface->handleServoPTarget(wrong_kind);

  auto accepted = std::make_shared<mg400_msgs::msg::ServoJ>();
  accepted->lease_id = started->lease_id;
  accepted->joint_angles = {{0.1, 0.2, 0.3, 0.4}};
  fixture.ros_interface->handleServoJTarget(accepted);

  const auto snapshot = fixture.session->getSnapshot();
  EXPECT_EQ(1U, snapshot.accepted_target_count);
  EXPECT_EQ(3U, snapshot.rejected_target_count);
  EXPECT_TRUE(
    requestState(
      *fixture.ros_interface, Manager::State::IDLE, started->lease_id)->success);
}

TEST(ServoControlRosInterface, ServoPTransformsToOriginFrameAndExtractsYaw)
{
  std::string requested_target_frame;
  Fixture fixture(
    [&requested_target_frame](
      const geometry_msgs::msg::PoseStamped & input,
      const std::string & target_frame,
      geometry_msgs::msg::PoseStamped & output,
      std::string & reason)
    {
      requested_target_frame = target_frame;
      output = input;
      output.header.frame_id = target_frame;
      output.pose.position.x = 0.1;
      output.pose.position.y = -0.2;
      output.pose.position.z = 0.3;
      output.pose.orientation = yawQuaternion(M_PI_2);
      reason.clear();
      return true;
    });
  const auto started = requestState(
    *fixture.ros_interface, Manager::State::SERVO_P, Manager::NO_LEASE);
  ASSERT_TRUE(started->success);

  auto target = std::make_shared<mg400_msgs::msg::ServoP>();
  target->lease_id = started->lease_id;
  target->pose.header.frame_id = "camera";
  target->pose.pose.orientation.w = 1.0;
  fixture.ros_interface->handleServoPTarget(target);

  EXPECT_EQ("robot_mg400_origin_link", requested_target_frame);
  ASSERT_TRUE(fixture.tcp.waitForCommandCount(1));
  ASSERT_FALSE(fixture.tcp.commands().empty());
  EXPECT_EQ("ServoP(100.000,-200.000,300.000,90.000)", fixture.tcp.commands().front());
  EXPECT_TRUE(
    requestState(
      *fixture.ros_interface, Manager::State::IDLE, started->lease_id)->success);
}

TEST(ServoControlRosInterface, InvalidQuaternionAndTfFailureNeverReachSession)
{
  Fixture fixture(
    [](const auto &, const auto &, auto &, auto & reason) {
      reason = "test transform unavailable";
      return false;
    });
  const auto started = requestState(
    *fixture.ros_interface, Manager::State::SERVO_P, Manager::NO_LEASE);
  ASSERT_TRUE(started->success);

  auto invalid_quaternion = std::make_shared<mg400_msgs::msg::ServoP>();
  invalid_quaternion->lease_id = started->lease_id;
  invalid_quaternion->pose.header.frame_id = "camera";
  fixture.ros_interface->handleServoPTarget(invalid_quaternion);

  auto tf_failure = std::make_shared<mg400_msgs::msg::ServoP>();
  tf_failure->lease_id = started->lease_id;
  tf_failure->pose.header.frame_id = "camera";
  tf_failure->pose.pose.orientation.w = 1.0;
  fixture.ros_interface->handleServoPTarget(tf_failure);

  EXPECT_EQ(0U, fixture.session->getSnapshot().accepted_target_count);
  EXPECT_EQ(0U, fixture.session->getSnapshot().rejected_target_count);
  EXPECT_EQ(2U, fixture.ros_interface->getRosRejectedTargetCount());
  EXPECT_EQ("test transform unavailable", fixture.ros_interface->getLastRosRejection());
  EXPECT_TRUE(
    requestState(
      *fixture.ros_interface, Manager::State::IDLE, started->lease_id)->success);
}

TEST(ServoControlRosInterface, QuaternionYawConversionRejectsInvalidValues)
{
  double yaw = 0.0;
  std::string reason;
  EXPECT_TRUE(RosInterface::quaternionToYaw(yawQuaternion(-0.75), yaw, reason));
  EXPECT_NEAR(-0.75, yaw, 1.0e-12);

  geometry_msgs::msg::Quaternion zero;
  zero.x = 0.0;
  zero.y = 0.0;
  zero.z = 0.0;
  zero.w = 0.0;
  EXPECT_FALSE(RosInterface::quaternionToYaw(zero, yaw, reason));

  geometry_msgs::msg::Quaternion non_finite;
  non_finite.w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(RosInterface::quaternionToYaw(non_finite, yaw, reason));
}

TEST(ServoControlRosInterface, LateSubscriberReceivesTransientLocalControlState)
{
  auto node = makeNode();
  auto manager = std::make_shared<Manager>();
  manager->updateRobotStatus(true, RobotMode::ENABLE);
  RosInterface ros_interface(
    *node, manager, rosOptions(),
    [](const auto & input, const auto &, auto & output, auto &) {
      output = input;
      return true;
    });
  ros_interface.publishControlState(true);

  auto subscriber_node = std::make_shared<rclcpp::Node>("late_control_state_subscriber");
  std::atomic<int> received{-1};
  auto subscription = subscriber_node->create_subscription<mg400_msgs::msg::ControlState>(
    "control_state",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    [&received](const mg400_msgs::msg::ControlState::SharedPtr message) {
      received.store(message->control_state);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(subscriber_node);
  const auto delivered = waitUntil(
    [&executor, &received]() {
      executor.spin_some();
      return received.load() >= 0;
    }, 2s);
  executor.remove_node(subscriber_node);
  executor.remove_node(node->get_node_base_interface());

  ASSERT_TRUE(delivered);
  EXPECT_EQ(static_cast<int>(Manager::State::IDLE), received.load());
  static_cast<void>(subscription);
}

TEST(ServoControlRosInterface, WatchdogStateIsPublishedInDiagnostics)
{
  auto node = makeNode();
  auto manager = std::make_shared<Manager>();
  manager->updateRobotStatus(true, RobotMode::ENABLE);
  FakeMotionTcpInterface tcp;
  auto commander = std::make_shared<mg400_interface::MotionCommander>(&tcp);
  auto stop_strategy = std::make_shared<SequenceStopStrategy>();
  auto options = sessionOptions();
  options.target_watchdog_timeout = 30ms;
  auto session = std::make_shared<Session>(
    manager, commander, stop_strategy, options);
  RosInterface ros_interface(
    *node, manager, rosOptions(),
    [](const auto & input, const auto &, auto & output, auto &) {
      output = input;
      return true;
    });
  ros_interface.installSession(session);

  auto subscriber_node = std::make_shared<rclcpp::Node>("servo_diagnostics_subscriber");
  std::mutex message_mutex;
  diagnostic_msgs::msg::DiagnosticArray latest;
  auto subscription =
    subscriber_node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "servo_diagnostics", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    [&message_mutex, &latest](
      const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message)
    {
      std::lock_guard<std::mutex> lock(message_mutex);
      latest = *message;
    });

  const auto started = requestState(
    ros_interface, Manager::State::SERVO_J, Manager::NO_LEASE);
  ASSERT_TRUE(started->success);
  ASSERT_TRUE(
    waitUntil(
      [&session]() {
        return session->getSnapshot().state == Session::State::IDLE;
      }));
  ros_interface.publishControlState();
  ros_interface.publishDiagnostics();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(subscriber_node);
  const auto delivered = waitUntil(
    [&executor, &message_mutex, &latest]() {
      executor.spin_some();
      std::lock_guard<std::mutex> lock(message_mutex);
      return diagnosticValue(latest, "watchdog_triggered") == "true";
    }, 2s);
  executor.remove_node(subscriber_node);
  executor.remove_node(node->get_node_base_interface());

  ASSERT_TRUE(delivered);
  {
    std::lock_guard<std::mutex> lock(message_mutex);
    EXPECT_EQ("WATCHDOG", diagnosticValue(latest, "stop_cause"));
    EXPECT_EQ("IDLE", diagnosticValue(latest, "control_state"));
  }
  ros_interface.clearSession("test complete");
  static_cast<void>(subscription);
}

TEST(ServoControlRosInterface, LifecycleStopFailureKeepsSessionAndLease)
{
  Fixture fixture;
  auto subscriber_node = std::make_shared<rclcpp::Node>("fault_diagnostics_subscriber");
  std::mutex message_mutex;
  diagnostic_msgs::msg::DiagnosticArray latest;
  auto subscription =
    subscriber_node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "servo_diagnostics", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    [&message_mutex, &latest](
      const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message)
    {
      std::lock_guard<std::mutex> lock(message_mutex);
      latest = *message;
    });
  fixture.stop_strategy->push(
    StopStrategy::Status::CONFIRMATION_TIMEOUT, "stop confirmation failed");
  const auto started = requestState(
    *fixture.ros_interface, Manager::State::SERVO_J, Manager::NO_LEASE);
  ASSERT_TRUE(started->success);

  const auto stopped = fixture.ros_interface->stopForLifecycle("test deactivate");
  EXPECT_FALSE(stopped.success);
  EXPECT_TRUE(fixture.ros_interface->hasSession());
  EXPECT_EQ(started->lease_id, fixture.manager->getSnapshot().lease_id);
  EXPECT_FALSE(fixture.manager->getSnapshot().accepting_servo_targets);
  fixture.ros_interface->publishDiagnostics();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(fixture.node->get_node_base_interface());
  executor.add_node(subscriber_node);
  const auto delivered = waitUntil(
    [&executor, &message_mutex, &latest]() {
      executor.spin_some();
      std::lock_guard<std::mutex> lock(message_mutex);
      return diagnosticValue(latest, "session_state") == "FAULTED";
    }, 2s);
  executor.remove_node(subscriber_node);
  executor.remove_node(fixture.node->get_node_base_interface());
  ASSERT_TRUE(delivered);
  {
    std::lock_guard<std::mutex> lock(message_mutex);
    ASSERT_FALSE(latest.status.empty());
    EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::ERROR, latest.status.front().level);
    EXPECT_EQ("SERVO_J", diagnosticValue(latest, "control_state"));
    EXPECT_EQ("stop confirmation failed", diagnosticValue(latest, "session_reason"));
  }

  EXPECT_TRUE(
    requestState(
      *fixture.ros_interface, Manager::State::IDLE, started->lease_id)->success);
  static_cast<void>(subscription);
}

TEST(ServoControlRosInterface, LifecycleStopCompletesBeforeSessionIsCleared)
{
  Fixture fixture;
  const auto started = requestState(
    *fixture.ros_interface, Manager::State::SERVO_P, Manager::NO_LEASE);
  ASSERT_TRUE(started->success);

  const auto stopped = fixture.ros_interface->stopForLifecycle("test deactivate");
  EXPECT_TRUE(stopped.success) << stopped.message;
  EXPECT_TRUE(fixture.ros_interface->hasSession());
  EXPECT_EQ(Session::State::IDLE, fixture.session->getSnapshot().state);
  EXPECT_EQ(Manager::NO_LEASE, fixture.manager->getSnapshot().lease_id);
  EXPECT_EQ(1U, fixture.stop_strategy->callCount());

  fixture.ros_interface->clearSession("test interface teardown");
  EXPECT_FALSE(fixture.ros_interface->hasSession());
}

TEST(ServoControlRosInterface, RetiredConnectionEpochCannotAffectNewLease)
{
  Fixture fixture;
  const auto old = requestState(
    *fixture.ros_interface, Manager::State::SERVO_J, Manager::NO_LEASE);
  ASSERT_TRUE(old->success);

  fixture.manager->updateRobotStatus(false, RobotMode::INVALID);
  fixture.manager->updateRobotStatus(true, RobotMode::ENABLE);
  EXPECT_TRUE(fixture.ros_interface->retireSessionIfLeaseLost("test reconnect"));
  EXPECT_FALSE(fixture.ros_interface->hasSession());
  EXPECT_EQ(0U, fixture.stop_strategy->callCount());

  auto new_session = std::make_shared<Session>(
    fixture.manager, fixture.commander, fixture.stop_strategy, sessionOptions());
  fixture.session = new_session;
  fixture.ros_interface->installSession(new_session);
  const auto current = requestState(
    *fixture.ros_interface, Manager::State::SERVO_P, Manager::NO_LEASE);
  ASSERT_TRUE(current->success);
  EXPECT_NE(old->lease_id, current->lease_id);
  EXPECT_EQ(current->lease_id, fixture.manager->getSnapshot().lease_id);
  EXPECT_TRUE(
    requestState(
      *fixture.ros_interface, Manager::State::IDLE, current->lease_id)->success);
}

}  // namespace
