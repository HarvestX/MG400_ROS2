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

#include <array>
#include <atomic>
#include <chrono>
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

#include <mg400_msgs/msg/robot_mode.hpp>
#include <mg400_msgs/msg/servo_error.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "mg400_interface/commander/motion_commander.hpp"
#include "mg400_interface/servo_stop_strategy.hpp"
#include "mg400_node/mg400_node.hpp"
#include "mg400_node/servo_control_ros_interface.hpp"

namespace
{

using namespace std::chrono_literals;  // NOLINT
using Manager = mg400_interface::ControlStateManager;
using Session = mg400_interface::ServoControlSession;
using FeedbackState = mg400_interface::ServoFeedbackState;
using OperationalCode = mg400_interface::ServoOperationalErrorCode;
using OperationalState = mg400_interface::ServoOperationalErrorState;
using SafetyCode = mg400_interface::ServoSafetyViolationCode;
using SafetyState = mg400_interface::ServoSafetyViolationState;
using StopStrategy = mg400_interface::ServoStopStrategy;
using RosInterface = mg400_node::ServoControlRosInterface;
using EnableServoJ = mg400_msgs::srv::EnableServoJ;
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
  static constexpr FeedbackState::ConnectionEpoch CONNECTION_EPOCH = 41;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node{makeNode()};
  Manager::SharedPtr manager{std::make_shared<Manager>()};
  FakeMotionTcpInterface tcp;
  mg400_interface::MotionCommander::SharedPtr commander{
    std::make_shared<mg400_interface::MotionCommander>(&tcp)};
  std::shared_ptr<SequenceStopStrategy> stop_strategy{
    std::make_shared<SequenceStopStrategy>()};
  FeedbackState::SharedPtr feedback_state{
    std::make_shared<FeedbackState>(CONNECTION_EPOCH)};
  SafetyState::SharedPtr safety_state{std::make_shared<SafetyState>()};
  OperationalState::SharedPtr operational_state{std::make_shared<OperationalState>()};
  std::shared_ptr<Session> session;
  std::unique_ptr<RosInterface> ros_interface;

  Fixture()
  {
    manager->updateRobotStatus(true, RobotMode::ENABLE);
    ros_interface = std::make_unique<RosInterface>(
      *node, manager, safety_state, operational_state);
    session = std::make_shared<Session>(
      manager, commander, stop_strategy,
      ros_interface->getSafetyViolationStateShared(),
      ros_interface->getOperationalErrorStateShared(), feedback_state, sessionOptions());
    ros_interface->installSession(session);
  }

  void updateFeedback(const std::array<double, 4> & joints)
  {
    feedback_state->update(joints);
  }

  ~Fixture()
  {
    if (ros_interface && ros_interface->hasSession()) {
      ros_interface->clearSession();
    }
  }
};

std::shared_ptr<EnableServoJ::Response> requestServoJ(
  RosInterface & ros_interface,
  const bool enable)
{
  auto request = std::make_shared<EnableServoJ::Request>();
  request->enable = enable;
  auto response = std::make_shared<EnableServoJ::Response>();
  ros_interface.handleEnableServoJ(request, response);
  return response;
}

TEST(ServoControlRosInterface, MakesServoErrorMessageFromSnapshots)
{
  using ServoError = mg400_msgs::msg::ServoError;
  const auto error = RosInterface::makeServoErrorMessage(
    {SafetyCode::SERVO_J_JOINT_LIMIT, "safety"},
    {OperationalCode::WATCHDOG_TIMEOUT, "operational"});
  EXPECT_EQ(ServoError::SAFETY_SERVO_J_JOINT_LIMIT, error.safety_violation_code);
  EXPECT_EQ("safety", error.safety_violation_message);
  EXPECT_EQ(ServoError::OPERATIONAL_WATCHDOG_TIMEOUT, error.operational_error_code);
  EXPECT_EQ("operational", error.operational_error_message);

  const auto clear = RosInterface::makeServoErrorMessage(
    {SafetyCode::NONE, "stale safety"},
    {OperationalCode::NONE, "stale operational"});
  EXPECT_EQ(ServoError::SAFETY_NONE, clear.safety_violation_code);
  EXPECT_TRUE(clear.safety_violation_message.empty());
  EXPECT_EQ(ServoError::OPERATIONAL_NONE, clear.operational_error_code);
  EXPECT_TRUE(clear.operational_error_message.empty());
}

TEST(ServoControlRosInterface, IssuesSingleLeaseAndAllowsUnauthenticatedStopRetry)
{
  Fixture fixture;

  const auto started = requestServoJ(*fixture.ros_interface, true);
  ASSERT_TRUE(started->success) << started->message;
  EXPECT_TRUE(started->enabled);
  ASSERT_NE(Manager::NO_LEASE, started->lease_id);
  const auto lease = started->lease_id;

  const auto duplicate_start = requestServoJ(*fixture.ros_interface, true);
  EXPECT_FALSE(duplicate_start->success);
  EXPECT_TRUE(duplicate_start->enabled);
  EXPECT_EQ(Manager::NO_LEASE, duplicate_start->lease_id);
  EXPECT_EQ(lease, fixture.manager->getSnapshot().lease_id);

  fixture.stop_strategy->push(
    StopStrategy::Status::CONFIRMATION_TIMEOUT, "ENABLE confirmation timed out");
  fixture.stop_strategy->push(StopStrategy::Status::SUCCESS, "retry confirmed");
  const auto failed_stop = requestServoJ(*fixture.ros_interface, false);
  EXPECT_FALSE(failed_stop->success);
  EXPECT_TRUE(failed_stop->enabled);
  EXPECT_EQ(Manager::NO_LEASE, failed_stop->lease_id);
  EXPECT_EQ(lease, fixture.manager->getSnapshot().lease_id);
  EXPECT_FALSE(fixture.manager->getSnapshot().accepting_servo_targets);

  const auto retried_stop = requestServoJ(*fixture.ros_interface, false);
  EXPECT_TRUE(retried_stop->success) << retried_stop->message;
  EXPECT_EQ(Manager::NO_LEASE, retried_stop->lease_id);
  EXPECT_FALSE(retried_stop->enabled);
  EXPECT_EQ(Manager::NO_LEASE, fixture.manager->getSnapshot().lease_id);
  EXPECT_EQ(2U, fixture.stop_strategy->callCount());

  const auto stale_stop = requestServoJ(*fixture.ros_interface, false);
  EXPECT_FALSE(stale_stop->success);
  EXPECT_EQ(Manager::NO_LEASE, stale_stop->lease_id);
}

TEST(ServoControlRosInterface, ServoJRejectsWrongLeaseAndNonFiniteTarget)
{
  Fixture fixture;
  const auto started = requestServoJ(*fixture.ros_interface, true);
  ASSERT_TRUE(started->success);

  auto wrong_lease = std::make_shared<mg400_msgs::msg::ServoJ>();
  wrong_lease->lease_id = started->lease_id + 1;
  wrong_lease->joint_angles = {{0.1, 0.2, 0.3, 0.4}};
  fixture.ros_interface->handleServoJTarget(wrong_lease);

  auto non_finite = std::make_shared<mg400_msgs::msg::ServoJ>(*wrong_lease);
  non_finite->lease_id = started->lease_id;
  non_finite->joint_angles[2] = std::numeric_limits<double>::infinity();
  fixture.ros_interface->handleServoJTarget(non_finite);

  auto accepted = std::make_shared<mg400_msgs::msg::ServoJ>();
  accepted->lease_id = started->lease_id;
  accepted->joint_angles = {{0.1, 0.2, 0.3, 0.4}};
  fixture.updateFeedback(accepted->joint_angles);
  fixture.ros_interface->handleServoJTarget(accepted);

  const auto snapshot = fixture.session->getSnapshot();
  EXPECT_EQ(Session::State::ACTIVE, snapshot.state);
  EXPECT_TRUE(
    requestServoJ(*fixture.ros_interface, false)->success);
}

TEST(MG400Node, InvalidServoSafetyParameterFailsLifecycleConfigure)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("auto_configure", false),
      rclcpp::Parameter("servo.safety.max_joint_step_rad", -0.1)});
  auto node = std::make_shared<mg400_node::MG400Node>(options);

  const auto state = node->configure();
  EXPECT_EQ("unconfigured", state.label());
}

TEST(ServoControlRosInterface, LateSubscriberReceivesTransientLocalControlState)
{
  auto node = makeNode();
  auto manager = std::make_shared<Manager>();
  manager->updateRobotStatus(true, RobotMode::ENABLE);
  RosInterface ros_interface(
    *node, manager, std::make_shared<SafetyState>(),
    std::make_shared<OperationalState>());
  const auto regular = manager->tryAcquire(Manager::State::REGULAR_MOTION);
  ASSERT_TRUE(regular.success) << regular.message;
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
  EXPECT_EQ(static_cast<int>(Manager::State::REGULAR_MOTION), received.load());
  EXPECT_TRUE(
    manager->release(Manager::State::REGULAR_MOTION, regular.lease_id).success);
  static_cast<void>(subscription);
}

TEST(ServoControlRosInterface, LifecycleStopFailureKeepsSessionAndLease)
{
  Fixture fixture;
  fixture.stop_strategy->push(
    StopStrategy::Status::CONFIRMATION_TIMEOUT, "stop confirmation failed");
  const auto started = requestServoJ(*fixture.ros_interface, true);
  ASSERT_TRUE(started->success);

  const auto stopped = fixture.ros_interface->stopForLifecycle("test deactivate");
  EXPECT_FALSE(stopped.success);
  EXPECT_TRUE(fixture.ros_interface->hasSession());
  EXPECT_EQ(started->lease_id, fixture.manager->getSnapshot().lease_id);
  EXPECT_FALSE(fixture.manager->getSnapshot().accepting_servo_targets);
  EXPECT_EQ(Session::State::FAULTED, fixture.session->getSnapshot().state);
  EXPECT_EQ("stop confirmation failed", fixture.session->getSnapshot().diagnostic);

  EXPECT_TRUE(
    requestServoJ(*fixture.ros_interface, false)->success);
}

TEST(ServoControlRosInterface, LifecycleStopCompletesBeforeSessionIsCleared)
{
  Fixture fixture;
  const auto started = requestServoJ(*fixture.ros_interface, true);
  ASSERT_TRUE(started->success);

  const auto stopped = fixture.ros_interface->stopForLifecycle("test deactivate");
  EXPECT_TRUE(stopped.success) << stopped.message;
  EXPECT_TRUE(fixture.ros_interface->hasSession());
  EXPECT_EQ(Session::State::IDLE, fixture.session->getSnapshot().state);
  EXPECT_EQ(Manager::NO_LEASE, fixture.manager->getSnapshot().lease_id);
  EXPECT_EQ(1U, fixture.stop_strategy->callCount());

  fixture.ros_interface->clearSession();
  EXPECT_FALSE(fixture.ros_interface->hasSession());
}

TEST(ServoControlRosInterface, RetiredConnectionEpochCannotAffectNewLease)
{
  Fixture fixture;
  const auto old = requestServoJ(*fixture.ros_interface, true);
  ASSERT_TRUE(old->success);

  fixture.manager->updateRobotStatus(false, RobotMode::INVALID);
  fixture.manager->updateRobotStatus(true, RobotMode::ENABLE);
  EXPECT_TRUE(fixture.ros_interface->retireSessionIfLeaseLost("test reconnect"));
  EXPECT_FALSE(fixture.ros_interface->hasSession());
  EXPECT_EQ(0U, fixture.stop_strategy->callCount());

  fixture.feedback_state->beginConnectionEpoch(Fixture::CONNECTION_EPOCH + 1);
  auto new_session = std::make_shared<Session>(
    fixture.manager, fixture.commander, fixture.stop_strategy,
    fixture.ros_interface->getSafetyViolationStateShared(),
    fixture.ros_interface->getOperationalErrorStateShared(), fixture.feedback_state,
    sessionOptions());
  fixture.session = new_session;
  fixture.ros_interface->installSession(new_session);
  const auto current = requestServoJ(*fixture.ros_interface, true);
  ASSERT_TRUE(current->success);
  EXPECT_NE(old->lease_id, current->lease_id);
  EXPECT_EQ(current->lease_id, fixture.manager->getSnapshot().lease_id);
  EXPECT_TRUE(
    requestServoJ(*fixture.ros_interface, false)->success);
}

TEST(ServoControlRosInterface, DisconnectedRetirementDoesNotMaskNodeConnectionError)
{
  Fixture fixture;
  const auto started = requestServoJ(*fixture.ros_interface, true);
  ASSERT_TRUE(started->success);

  fixture.manager->updateRobotStatus(false, RobotMode::INVALID);
  EXPECT_TRUE(fixture.ros_interface->retireSessionIfLeaseLost("test disconnect"));
  EXPECT_EQ(
    OperationalCode::NONE,
    fixture.ros_interface->getOperationalErrorStateShared()->getSnapshot().code);
  EXPECT_TRUE(
    fixture.ros_interface->getOperationalErrorStateShared()->reportError(
      OperationalCode::REALTIME_CONNECTION_LOST,
      "The MG400 connection was lost during Servo control"));
}

TEST(ServoControlRosInterface, PublishesErrorFieldsImmediatelyAndRearmsBothLatches)
{
  using ServoError = mg400_msgs::msg::ServoError;
  auto node = makeNode();
  auto manager = std::make_shared<Manager>();
  manager->updateRobotStatus(true, RobotMode::ENABLE);
  FakeMotionTcpInterface tcp;
  auto commander = std::make_shared<mg400_interface::MotionCommander>(&tcp);
  auto stop_strategy = std::make_shared<SequenceStopStrategy>();
  RosInterface ros_interface(
    *node, manager, std::make_shared<SafetyState>(),
    std::make_shared<OperationalState>());
  auto safety_state = ros_interface.getSafetyViolationStateShared();
  auto operational_state = ros_interface.getOperationalErrorStateShared();
  auto feedback_state = std::make_shared<FeedbackState>(61);
  auto session = std::make_shared<Session>(
    manager, commander, stop_strategy, safety_state, operational_state,
    feedback_state, sessionOptions());
  ros_interface.installSession(session);

  auto subscriber_node = std::make_shared<rclcpp::Node>("servo_error_subscriber");
  std::mutex message_mutex;
  ServoError latest;
  std::uint64_t received_count = 0;
  auto subscription =
    subscriber_node->create_subscription<ServoError>(
    "servo_error", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    [&message_mutex, &latest, &received_count](
      const ServoError::SharedPtr message)
    {
      std::lock_guard<std::mutex> lock(message_mutex);
      latest = *message;
      ++received_count;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(subscriber_node);

  ros_interface.publishServoError();
  ASSERT_TRUE(
    waitUntil(
      [&executor, &message_mutex, &latest]() {
        executor.spin_some();
        std::lock_guard<std::mutex> lock(message_mutex);
        return latest.safety_violation_code == ServoError::SAFETY_NONE &&
        latest.operational_error_code == ServoError::OPERATIONAL_NONE;
      }, 2s));
  {
    std::lock_guard<std::mutex> lock(message_mutex);
    EXPECT_TRUE(latest.safety_violation_message.empty());
    EXPECT_TRUE(latest.operational_error_message.empty());
  }

  ASSERT_TRUE(
    operational_state->reportError(
      OperationalCode::WATCHDOG_TIMEOUT, "Servo target watchdog expired"));
  ASSERT_TRUE(
    waitUntil(
      [&executor, &message_mutex, &latest]() {
        executor.spin_some();
        std::lock_guard<std::mutex> lock(message_mutex);
        return latest.operational_error_code == ServoError::OPERATIONAL_WATCHDOG_TIMEOUT;
      }, 2s));
  {
    std::lock_guard<std::mutex> lock(message_mutex);
    EXPECT_EQ("Servo target watchdog expired", latest.operational_error_message);
  }

  std::uint64_t count_before_violation;
  {
    std::lock_guard<std::mutex> lock(message_mutex);
    count_before_violation = received_count;
  }
  ASSERT_TRUE(
    safety_state->reportViolation(
      SafetyCode::SERVO_J_COMMAND_DISCONTINUITY,
      "Servo target is too far from the current position"));
  ASSERT_TRUE(
    waitUntil(
      [&executor, &message_mutex, &latest, &received_count, count_before_violation]() {
        executor.spin_some();
        std::lock_guard<std::mutex> lock(message_mutex);
        return received_count > count_before_violation &&
        latest.safety_violation_code == ServoError::SAFETY_SERVO_J_COMMAND_DISCONTINUITY;
      }, 2s));
  {
    std::lock_guard<std::mutex> lock(message_mutex);
    EXPECT_EQ(
      "Servo target is too far from the current position",
      latest.safety_violation_message);
  }

  std::uint64_t count_before_rearm;
  {
    std::lock_guard<std::mutex> lock(message_mutex);
    count_before_rearm = received_count;
  }
  const auto started = requestServoJ(ros_interface, true);
  ASSERT_TRUE(started->success) << started->message;
  ASSERT_TRUE(
    waitUntil(
      [&executor, &message_mutex, &latest, &received_count, count_before_rearm]() {
        executor.spin_some();
        std::lock_guard<std::mutex> lock(message_mutex);
        return received_count > count_before_rearm &&
        latest.safety_violation_code == ServoError::SAFETY_NONE &&
        latest.operational_error_code == ServoError::OPERATIONAL_NONE;
      }, 2s));
  {
    std::lock_guard<std::mutex> lock(message_mutex);
    EXPECT_TRUE(latest.safety_violation_message.empty());
    EXPECT_TRUE(latest.operational_error_message.empty());
  }

  EXPECT_TRUE(
    requestServoJ(ros_interface, false)->success);
  ros_interface.clearSession();
  executor.remove_node(subscriber_node);
  executor.remove_node(node->get_node_base_interface());
  static_cast<void>(subscription);
}

TEST(ServoControlRosInterface, NodeOwnedErrorLatchesSurviveRosInterfaceRecreation)
{
  auto node = makeNode();
  auto manager = std::make_shared<Manager>();
  manager->updateRobotStatus(true, RobotMode::ENABLE);
  auto safety_state = std::make_shared<mg400_interface::ServoSafetyViolationState>();
  auto operational_state = std::make_shared<OperationalState>();
  {
    RosInterface ros_interface(
      *node, manager, safety_state, operational_state);
    ASSERT_TRUE(
      safety_state->reportViolation(
        SafetyCode::SERVO_J_JOINT_LIMIT, "ServoJ target violates a joint limit"));
    ASSERT_TRUE(
      operational_state->reportError(
        OperationalCode::REALTIME_CONNECTION_LOST,
        "MG400 connection was lost during Servo control"));
  }

  RosInterface replacement(
    *node, manager, safety_state, operational_state);
  EXPECT_EQ(SafetyCode::SERVO_J_JOINT_LIMIT, safety_state->getSnapshot().code);
  EXPECT_EQ(
    OperationalCode::REALTIME_CONNECTION_LOST,
    operational_state->getSnapshot().code);
  EXPECT_EQ(
    safety_state, replacement.getSafetyViolationStateShared());
  EXPECT_EQ(
    operational_state, replacement.getOperationalErrorStateShared());
}

TEST(ServoControlRosInterface, ErrorLatchesSurviveStopAndSessionReplacement)
{
  Fixture fixture;
  const auto started = requestServoJ(*fixture.ros_interface, true);
  ASSERT_TRUE(started->success) << started->message;
  auto safety_state = fixture.ros_interface->getSafetyViolationStateShared();
  auto operational_state = fixture.ros_interface->getOperationalErrorStateShared();
  ASSERT_TRUE(
    safety_state->reportViolation(
      SafetyCode::SERVO_J_JOINT_LIMIT, "ServoJ target violates a joint limit"));
  ASSERT_TRUE(
    operational_state->reportError(
      OperationalCode::MOTION_RESPONSE_TIMEOUT, "Servo Motion response timed out"));

  ASSERT_TRUE(
    requestServoJ(*fixture.ros_interface, false)->success);
  EXPECT_EQ(SafetyCode::SERVO_J_JOINT_LIMIT, safety_state->getSnapshot().code);
  EXPECT_EQ(OperationalCode::MOTION_RESPONSE_TIMEOUT, operational_state->getSnapshot().code);

  fixture.ros_interface->clearSession();
  EXPECT_EQ(SafetyCode::SERVO_J_JOINT_LIMIT, safety_state->getSnapshot().code);
  EXPECT_EQ(OperationalCode::MOTION_RESPONSE_TIMEOUT, operational_state->getSnapshot().code);
  auto replacement = std::make_shared<Session>(
    fixture.manager, fixture.commander, fixture.stop_strategy,
    safety_state, operational_state,
    fixture.feedback_state, sessionOptions());
  fixture.session = replacement;
  fixture.ros_interface->installSession(replacement);

  const auto regular_motion = fixture.manager->tryAcquire(Manager::State::REGULAR_MOTION);
  ASSERT_TRUE(regular_motion.success) << regular_motion.message;
  const auto failed_start = requestServoJ(*fixture.ros_interface, true);
  EXPECT_FALSE(failed_start->success);
  EXPECT_EQ(SafetyCode::SERVO_J_JOINT_LIMIT, safety_state->getSnapshot().code);
  EXPECT_EQ(OperationalCode::MOTION_RESPONSE_TIMEOUT, operational_state->getSnapshot().code);
  ASSERT_TRUE(
    fixture.manager->release(
      Manager::State::REGULAR_MOTION, regular_motion.lease_id).success);

  const auto rearmed = requestServoJ(*fixture.ros_interface, true);
  ASSERT_TRUE(rearmed->success) << rearmed->message;
  EXPECT_EQ(SafetyCode::NONE, safety_state->getSnapshot().code);
  EXPECT_TRUE(safety_state->getSnapshot().message.empty());
  EXPECT_EQ(OperationalCode::NONE, operational_state->getSnapshot().code);
  EXPECT_TRUE(operational_state->getSnapshot().message.empty());
  EXPECT_TRUE(
    requestServoJ(*fixture.ros_interface, false)->success);

}

}  // namespace
