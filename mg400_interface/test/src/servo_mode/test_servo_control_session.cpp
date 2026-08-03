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
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <mg400_msgs/msg/robot_mode.hpp>

#include "mg400_interface/servo_mode/servo_control_session.hpp"
#include "mg400_interface/servo_mode/servo_stop_strategy.hpp"
#include "mg400_interface/command_utils.hpp"

namespace
{
using namespace std::chrono_literals;  // NOLINT
using Manager = mg400_interface::ControlStateManager;
using RealtimeDataSnapshot = mg400_interface::RealtimeDataSnapshot;
using ResetRobotStopStrategy = mg400_interface::ResetRobotStopStrategy;
using ServoControlSession = mg400_interface::ServoControlSession;
using ServoOperationalErrorCode = mg400_interface::ServoOperationalErrorCode;
using ServoOperationalErrorState = mg400_interface::ServoOperationalErrorState;
using ServoSafetyViolationCode = mg400_interface::ServoSafetyViolationCode;
using ServoSafetyViolationState = mg400_interface::ServoSafetyViolationState;
using ServoStopStrategy = mg400_interface::ServoStopStrategy;
using RobotMode = mg400_msgs::msg::RobotMode;

template<typename Predicate>
bool waitUntil(Predicate predicate, const std::chrono::milliseconds timeout = 500ms)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(1ms);
  }
  return predicate();
}

class FakeMotionTcpInterface : public mg400_interface::MotionTcpInterfaceBase
{
public:
  void sendCommand(const std::string & command) override
  {
    bool block = false;
    bool fail = false;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      fail = this->fail_next_send_;
      this->fail_next_send_ = false;
      if (fail) {
        throw std::runtime_error("simulated TCP send failure");
      }
      this->commands_.push_back(command);
      block = this->block_first_send_ && this->commands_.size() == 1;
    }
    this->cv_.notify_all();

    if (block) {
      std::unique_lock<std::mutex> lock(this->mutex_block_);
      this->cv_block_.wait(lock, [this]() {return this->release_first_send_;});
    }

  }

  std::string recvResponse(std::chrono::nanoseconds /*timeout*/) override
  {
    bool block = false;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      ++this->response_count_;
      block = this->block_first_response_ && this->response_count_ == 1;
    }
    if (block) {
      std::unique_lock<std::mutex> lock(this->mutex_response_block_);
      this->cv_response_block_.wait(
        lock, [this]() {return this->release_first_response_;});
    }

    std::lock_guard<std::mutex> lock(this->mutex_);
    if (this->fail_next_response_) {
      this->fail_next_response_ = false;
      throw std::runtime_error("simulated Motion TCP response timeout");
    }
    if (!this->responses_.empty()) {
      auto response = this->responses_.front();
      this->responses_.pop_front();
      return response;
    }
    if (this->commands_.empty()) {
      throw std::runtime_error("response requested before a command was sent");
    }
    return "0,{}," + this->commands_.back() + ";";
  }

  void respondNextWith(const std::string & response)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->responses_.push_back(response);
  }

  void failNextResponse()
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->fail_next_response_ = true;
  }

  void blockFirstResponse()
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->block_first_response_ = true;
  }

  void releaseFirstResponse()
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex_response_block_);
      this->release_first_response_ = true;
    }
    this->cv_response_block_.notify_all();
  }

  void blockFirstSend()
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->block_first_send_ = true;
  }

  void failNextSend()
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->fail_next_send_ = true;
  }

  void releaseFirstSend()
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex_block_);
      this->release_first_send_ = true;
    }
    this->cv_block_.notify_all();
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
  std::deque<std::string> responses_;
  bool block_first_send_{false};
  bool fail_next_send_{false};
  bool fail_next_response_{false};
  bool block_first_response_{false};
  std::size_t response_count_{0};

  std::mutex mutex_block_;
  std::condition_variable cv_block_;
  bool release_first_send_{false};

  std::mutex mutex_response_block_;
  std::condition_variable cv_response_block_;
  bool release_first_response_{false};
};

class FakeStopStrategy : public ServoStopStrategy
{
public:
  Result stop(const Clock::time_point & /*deadline*/) override
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    ++this->call_count_;
    if (this->results_.empty()) {
      return Result{Status::SUCCESS, "fake stop confirmed"};
    }
    auto result = this->results_.front();
    this->results_.pop_front();
    return result;
  }

  void addResult(Status status, const std::string & message)
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

class FunctionStopStrategy : public ServoStopStrategy
{
public:
  explicit FunctionStopStrategy(std::function<Result()> function)
  : function_(std::move(function)) {}

  Result stop(const Clock::time_point & /*deadline*/) override
  {
    return this->function_();
  }

private:
  std::function<Result()> function_;
};

struct SessionFixture
{
  static constexpr RealtimeDataSnapshot::ConnectionEpoch CONNECTION_EPOCH = 7;

  FakeMotionTcpInterface tcp;
  Manager::SharedPtr manager{std::make_shared<Manager>()};
  mg400_interface::MotionCommander::SharedPtr commander{
    std::make_shared<mg400_interface::MotionCommander>(&tcp)};
  std::shared_ptr<FakeStopStrategy> stop_strategy{std::make_shared<FakeStopStrategy>()};
  ServoSafetyViolationState::SharedPtr safety_state{
    std::make_shared<ServoSafetyViolationState>()};
  ServoOperationalErrorState::SharedPtr operational_state{
    std::make_shared<ServoOperationalErrorState>()};
  mutable std::mutex feedback_mutex;
  RealtimeDataSnapshot feedback_snapshot;

  SessionFixture()
  {
    manager->updateRobotStatus(true, RobotMode::ENABLE);
    feedback_snapshot.connection_epoch = CONNECTION_EPOCH;
  }

  std::unique_ptr<ServoControlSession> makeSession(
    const ServoControlSession::Options & options)
  {
    return std::make_unique<ServoControlSession>(
      manager, commander, stop_strategy, safety_state, operational_state,
      [this]() {return this->getFeedback();}, options);
  }

  void updateFeedback(
    const std::array<double, 4> & joints,
    const RealtimeDataSnapshot::Clock::time_point received_at =
    RealtimeDataSnapshot::Clock::now())
  {
    std::lock_guard<std::mutex> lock(this->feedback_mutex);
    for (std::size_t index = 0; index < joints.size(); ++index) {
      this->feedback_snapshot.data.q_actual[index] = joints[index] * mg400_interface::TO_DEGREE;
    }
    this->feedback_snapshot.received_at = received_at;
    this->feedback_snapshot.has_data = true;
  }

  void beginConnectionEpoch(const RealtimeDataSnapshot::ConnectionEpoch epoch)
  {
    std::lock_guard<std::mutex> lock(this->feedback_mutex);
    this->feedback_snapshot = RealtimeDataSnapshot{};
    this->feedback_snapshot.connection_epoch = epoch;
  }

  RealtimeDataSnapshot getFeedback() const
  {
    std::lock_guard<std::mutex> lock(this->feedback_mutex);
    return this->feedback_snapshot;
  }
};

ServoControlSession::Options quietOptions()
{
  ServoControlSession::Options options;
  options.send_period = 30ms;
  options.target_watchdog_timeout = 1s;
  options.stop_confirmation_timeout = 50ms;
  return options;
}

TEST(ServoControlSession, TargetQueueAdmissionRejectsWrongLease)
{
  SessionFixture fixture;
  auto session = fixture.makeSession(quietOptions());
  const auto started = session->start();
  ASSERT_TRUE(started.success) << started.message;
  fixture.updateFeedback({{0.1, 0.2, 0.3, 0.4}});

  EXPECT_FALSE(
    session->updateServoJTarget(
      started.lease_id + 1, {{0.1, 0.2, 0.3, 0.4}}));
  EXPECT_TRUE(session->updateServoJTarget(started.lease_id, {{0.1, 0.2, 0.3, 0.4}}));

  const auto snapshot = session->getSnapshot();
  EXPECT_EQ(ServoControlSession::State::ACTIVE, snapshot.state);
  EXPECT_TRUE(session->stop().success);
}

TEST(ServoControlSession, NonFiniteTargetUsesSafetyFaultStopPath)
{
  SessionFixture fixture;
  auto session = fixture.makeSession(quietOptions());
  const auto started = session->start();
  ASSERT_TRUE(started.success) << started.message;

  EXPECT_FALSE(
    session->updateServoJTarget(
      started.lease_id,
      {{0.1, std::numeric_limits<double>::quiet_NaN(), 0.3, 0.4}}));
  ASSERT_TRUE(
    waitUntil(
      [&session]() {
        return session->getSnapshot().state == ServoControlSession::State::IDLE;
      }));
  EXPECT_TRUE(fixture.tcp.commands().empty());
  EXPECT_EQ(
    ServoSafetyViolationCode::SERVO_J_JOINT_LIMIT,
    fixture.safety_state->getSnapshot().code);
  EXPECT_EQ(1U, fixture.stop_strategy->callCount());
}

TEST(ServoControlSession, KinematicViolationIsNotBufferedAndUsesFaultStopPath)
{
  {
    SessionFixture fixture;
    auto session = fixture.makeSession(quietOptions());
    const auto started = session->start();
    ASSERT_TRUE(started.success) << started.message;
    EXPECT_FALSE(session->updateServoJTarget(started.lease_id, {{3.0, 0.0, 0.2, 0.0}}));
    ASSERT_TRUE(
      waitUntil(
        [&session]() {
          return session->getSnapshot().state == ServoControlSession::State::IDLE;
        }));
    EXPECT_TRUE(fixture.tcp.commands().empty());
    EXPECT_EQ(
      ServoSafetyViolationCode::SERVO_J_JOINT_LIMIT,
      fixture.safety_state->getSnapshot().code);
    EXPECT_EQ(
      ServoOperationalErrorCode::NONE,
      fixture.operational_state->getSnapshot().code);
    EXPECT_EQ(1U, fixture.stop_strategy->callCount());
  }
}

TEST(ServoControlSession, SendsQueuedTargetsInArrivalOrder)
{
  SessionFixture fixture;
  auto options = quietOptions();
  options.send_period = 50ms;
  auto session = fixture.makeSession(options);
  const auto started = session->start();
  ASSERT_TRUE(started.success) << started.message;
  const std::array<double, 4> first{{0.0, 0.0, 0.2, 0.0}};
  const std::array<double, 4> second{{0.01, 0.01, 0.21, 0.01}};
  fixture.updateFeedback(first);
  ASSERT_TRUE(session->updateServoJTarget(started.lease_id, first));
  ASSERT_TRUE(session->updateServoJTarget(started.lease_id, second));

  ASSERT_TRUE(fixture.tcp.waitForCommandCount(2));
  const auto commands = fixture.tcp.commands();
  ASSERT_EQ(2U, commands.size());
  EXPECT_EQ("ServoJ(0.000,0.000,11.459,0.000)", commands[0]);
  EXPECT_EQ("ServoJ(0.573,0.573,12.032,0.573)", commands[1]);
  EXPECT_TRUE(session->stop().success);
}

TEST(ServoControlSession, QueuesTargetsReceivedWhileWaitingForMotionResponse)
{
  SessionFixture fixture;
  fixture.tcp.blockFirstResponse();
  auto options = quietOptions();
  options.send_period = 5ms;
  auto session = fixture.makeSession(options);
  const auto started = session->start();
  ASSERT_TRUE(started.success) << started.message;

  const std::array<double, 4> first{{0.0, 0.0, 0.2, 0.0}};
  const std::array<double, 4> second{{0.01, 0.0, 0.21, 0.0}};
  const std::array<double, 4> third{{0.02, 0.0, 0.22, 0.0}};
  fixture.updateFeedback(first);
  ASSERT_TRUE(session->updateServoJTarget(started.lease_id, first));
  ASSERT_TRUE(fixture.tcp.waitForCommandCount(1));

  ASSERT_TRUE(session->updateServoJTarget(started.lease_id, second));
  ASSERT_TRUE(session->updateServoJTarget(started.lease_id, third));
  EXPECT_EQ(1U, fixture.tcp.commands().size());
  fixture.tcp.releaseFirstResponse();

  ASSERT_TRUE(fixture.tcp.waitForCommandCount(3));
  const auto commands = fixture.tcp.commands();
  ASSERT_EQ(3U, commands.size());
  EXPECT_EQ("ServoJ(0.000,0.000,11.459,0.000)", commands[0]);
  EXPECT_EQ("ServoJ(0.573,0.000,12.032,0.000)", commands[1]);
  EXPECT_EQ("ServoJ(1.146,0.000,12.605,0.000)", commands[2]);
  EXPECT_TRUE(session->stop().success);
}

TEST(ServoControlSession, FullTargetQueueRejectsNewestTarget)
{
  SessionFixture fixture;
  fixture.tcp.blockFirstResponse();
  auto options = quietOptions();
  options.send_period = 5ms;
  options.target_queue_capacity = 2;
  auto session = fixture.makeSession(options);
  const auto started = session->start();
  ASSERT_TRUE(started.success) << started.message;

  const std::array<double, 4> first{{0.0, 0.0, 0.2, 0.0}};
  const std::array<double, 4> second{{0.01, 0.0, 0.21, 0.0}};
  const std::array<double, 4> rejected{{0.02, 0.0, 0.22, 0.0}};
  fixture.updateFeedback(first);
  ASSERT_TRUE(session->updateServoJTarget(started.lease_id, first));
  ASSERT_TRUE(fixture.tcp.waitForCommandCount(1));

  ASSERT_TRUE(session->updateServoJTarget(started.lease_id, second));
  EXPECT_FALSE(session->updateServoJTarget(started.lease_id, rejected));
  fixture.tcp.releaseFirstResponse();

  ASSERT_TRUE(fixture.tcp.waitForCommandCount(2));
  const auto commands = fixture.tcp.commands();
  ASSERT_EQ(2U, commands.size());
  EXPECT_EQ("ServoJ(0.000,0.000,11.459,0.000)", commands[0]);
  EXPECT_EQ("ServoJ(0.573,0.000,12.032,0.000)", commands[1]);
  EXPECT_TRUE(session->stop().success);
}

TEST(ServoControlSession, InitialServoJUsesFreshFeedbackAndAllowsInclusiveThreshold)
{
  SessionFixture fixture;
  auto options = quietOptions();
  options.send_period = 5ms;
  options.max_initial_joint_distance_rad = 0.05;
  auto session = fixture.makeSession(options);
  const auto started = session->start();
  ASSERT_TRUE(started.success) << started.message;

  const std::array<double, 4> current{{0.0, 0.0, 0.2, 0.0}};
  const std::array<double, 4> target{{0.05, 0.0, 0.2, 0.0}};
  fixture.updateFeedback(current);
  ASSERT_TRUE(session->updateServoJTarget(started.lease_id, target));
  ASSERT_TRUE(fixture.tcp.waitForCommandCount(1));
  EXPECT_EQ(ServoSafetyViolationCode::NONE, fixture.safety_state->getSnapshot().code);
  EXPECT_TRUE(session->stop().success);
}

TEST(ServoControlSession, InitialServoJDiscontinuityNeverReachesTcpAndFaultStopsOnce)
{
  SessionFixture fixture;
  auto options = quietOptions();
  options.send_period = 5ms;
  options.max_initial_joint_distance_rad = 0.05;
  auto session = fixture.makeSession(options);
  const auto started = session->start();
  ASSERT_TRUE(started.success) << started.message;

  fixture.updateFeedback({{0.0, 0.0, 0.2, 0.0}});
  ASSERT_TRUE(
    session->updateServoJTarget(started.lease_id, {{0.05001, 0.0, 0.2, 0.0}}));
  ASSERT_TRUE(
    waitUntil(
      [&session]() {
        return session->getSnapshot().state == ServoControlSession::State::IDLE;
      }));

  EXPECT_TRUE(fixture.tcp.commands().empty());
  EXPECT_EQ(1U, fixture.stop_strategy->callCount());
  const auto violation = fixture.safety_state->getSnapshot();
  EXPECT_EQ(ServoSafetyViolationCode::SERVO_J_COMMAND_DISCONTINUITY, violation.code);
  EXPECT_EQ(
    "Initial ServoJ target is too far from the current joint position", violation.message);
}

TEST(ServoControlSession, InitialCommandRejectsUnavailableOrInvalidFeedback)
{
  enum class Case {MISSING, STALE, WRONG_EPOCH, NON_FINITE};
  for (const auto test_case : {Case::MISSING, Case::STALE, Case::WRONG_EPOCH, Case::NON_FINITE}) {
    SessionFixture fixture;
    auto options = quietOptions();
    options.send_period = 5ms;
    options.feedback_timeout = 10ms;
    auto session = fixture.makeSession(options);
    const auto started = session->start();
    ASSERT_TRUE(started.success) << started.message;

    if (test_case == Case::STALE) {
      fixture.updateFeedback(
        {{0.0, 0.0, 0.2, 0.0}},
        RealtimeDataSnapshot::Clock::now() - 20ms);
    } else if (test_case == Case::WRONG_EPOCH) {
      fixture.beginConnectionEpoch(SessionFixture::CONNECTION_EPOCH + 1);
      fixture.updateFeedback({{0.0, 0.0, 0.2, 0.0}});
    } else if (test_case == Case::NON_FINITE) {
      fixture.updateFeedback(
        {{0.0, std::numeric_limits<double>::quiet_NaN(), 0.2, 0.0}});
    }
    ASSERT_TRUE(
      session->updateServoJTarget(started.lease_id, {{0.0, 0.0, 0.2, 0.0}}));
    ASSERT_TRUE(
      waitUntil(
        [&session]() {
          return session->getSnapshot().state == ServoControlSession::State::IDLE;
        }));
    EXPECT_TRUE(fixture.tcp.commands().empty());
    EXPECT_EQ(
      ServoSafetyViolationCode::REALTIME_FEEDBACK_UNAVAILABLE,
      fixture.safety_state->getSnapshot().code);
    EXPECT_EQ(1U, fixture.stop_strategy->callCount());
  }
}

TEST(ServoControlSession, SubsequentServoJComparesQueuedTargetWithLastTcpSuccess)
{
  SessionFixture fixture;
  auto options = quietOptions();
  options.send_period = 15ms;
  options.max_joint_step_rad = 0.02;
  auto session = fixture.makeSession(options);
  const auto started = session->start();
  ASSERT_TRUE(started.success) << started.message;

  const std::array<double, 4> first{{0.0, 0.0, 0.2, 0.0}};
  fixture.updateFeedback(first);
  ASSERT_TRUE(session->updateServoJTarget(started.lease_id, first));
  ASSERT_TRUE(fixture.tcp.waitForCommandCount(1));

  // Feedback is deliberately moved: it must not be the comparison basis now.
  fixture.updateFeedback({{1.0, 1.0, 1.0, 1.0}});
  ASSERT_TRUE(
    session->updateServoJTarget(started.lease_id, {{0.01, 0.0, 0.2, 0.0}}));
  ASSERT_TRUE(
    session->updateServoJTarget(started.lease_id, {{0.03001, 0.0, 0.2, 0.0}}));

  ASSERT_TRUE(
    waitUntil(
      [&session]() {
        return session->getSnapshot().state == ServoControlSession::State::IDLE;
      }));
  EXPECT_EQ(2U, fixture.tcp.commands().size());
  EXPECT_EQ(
    ServoSafetyViolationCode::SERVO_J_COMMAND_DISCONTINUITY,
    fixture.safety_state->getSnapshot().code);
  EXPECT_EQ(1U, fixture.stop_strategy->callCount());
}

TEST(ServoControlSession, SubsequentServoJAllowsInclusiveStepThreshold)
{
  SessionFixture fixture;
  auto options = quietOptions();
  options.send_period = 5ms;
  options.max_joint_step_rad = 0.02;
  auto session = fixture.makeSession(options);
  const auto started = session->start();
  ASSERT_TRUE(started.success) << started.message;

  const std::array<double, 4> first{{0.0, 0.0, 0.2, 0.0}};
  fixture.updateFeedback(first);
  ASSERT_TRUE(session->updateServoJTarget(started.lease_id, first));
  ASSERT_TRUE(fixture.tcp.waitForCommandCount(1));
  fixture.updateFeedback({{1.0, 1.0, 1.0, 1.0}});
  ASSERT_TRUE(
    session->updateServoJTarget(started.lease_id, {{0.02, 0.0, 0.2, 0.0}}));
  ASSERT_TRUE(fixture.tcp.waitForCommandCount(2));
  EXPECT_EQ(ServoSafetyViolationCode::NONE, fixture.safety_state->getSnapshot().code);
  EXPECT_TRUE(session->stop().success);
}

TEST(ServoControlSession, TcpSendFailureUsesFaultStopPath)
{
  SessionFixture fixture;
  fixture.tcp.failNextSend();
  auto options = quietOptions();
  options.send_period = 5ms;
  auto session = fixture.makeSession(options);
  const auto started = session->start();
  ASSERT_TRUE(started.success) << started.message;

  const std::array<double, 4> target{{0.0, 0.0, 0.2, 0.0}};
  fixture.updateFeedback(target);
  ASSERT_TRUE(session->updateServoJTarget(started.lease_id, target));
  ASSERT_TRUE(
    waitUntil(
      [&session]() {
        return session->getSnapshot().state == ServoControlSession::State::IDLE;
      }));
  EXPECT_TRUE(fixture.tcp.commands().empty());
  EXPECT_EQ(1U, fixture.stop_strategy->callCount());
  EXPECT_EQ(ServoSafetyViolationCode::NONE, fixture.safety_state->getSnapshot().code);
  EXPECT_EQ(
    ServoOperationalErrorCode::MOTION_TCP_SEND_FAILED,
    fixture.operational_state->getSnapshot().code);
}

TEST(ServoControlSession, MotionResponseTimeoutUsesFaultStopPath)
{
  SessionFixture fixture;
  fixture.tcp.failNextResponse();
  auto options = quietOptions();
  options.send_period = 5ms;
  auto session = fixture.makeSession(options);
  const auto started = session->start();
  ASSERT_TRUE(started.success) << started.message;

  const std::array<double, 4> target{{0.0, 0.0, 0.2, 0.0}};
  fixture.updateFeedback(target);
  ASSERT_TRUE(session->updateServoJTarget(started.lease_id, target));
  ASSERT_TRUE(
    waitUntil(
      [&session]() {
        return session->getSnapshot().state == ServoControlSession::State::IDLE;
      }));

  EXPECT_EQ(1U, fixture.tcp.commands().size());
  EXPECT_EQ(1U, fixture.stop_strategy->callCount());
  const auto error = fixture.operational_state->getSnapshot();
  EXPECT_EQ(ServoOperationalErrorCode::MOTION_TCP_SEND_FAILED, error.code);
  EXPECT_NE(std::string::npos, error.message.find("Motion TCP Servo command failed"));
  EXPECT_NE(std::string::npos, error.message.find("response timeout"));
}

TEST(ServoControlSession, ControllerErrorResponseUsesFaultStopPath)
{
  SessionFixture fixture;
  fixture.tcp.respondNextWith(
    "-10000,{},ServoJ(0.000,0.000,11.459,0.000);");
  auto options = quietOptions();
  options.send_period = 5ms;
  auto session = fixture.makeSession(options);
  const auto started = session->start();
  ASSERT_TRUE(started.success) << started.message;

  const std::array<double, 4> target{{0.0, 0.0, 0.2, 0.0}};
  fixture.updateFeedback(target);
  ASSERT_TRUE(session->updateServoJTarget(started.lease_id, target));
  ASSERT_TRUE(
    waitUntil(
      [&session]() {
        return session->getSnapshot().state == ServoControlSession::State::IDLE;
      }));

  EXPECT_EQ(1U, fixture.tcp.commands().size());
  EXPECT_EQ(1U, fixture.stop_strategy->callCount());
  const auto error = fixture.operational_state->getSnapshot();
  EXPECT_EQ(ServoOperationalErrorCode::MOTION_TCP_SEND_FAILED, error.code);
  EXPECT_NE(std::string::npos, error.message.find("ErrorID=-10000"));
}

TEST(ServoControlSession, InvalidMotionResponseUsesFaultStopPath)
{
  SessionFixture fixture;
  fixture.tcp.respondNextWith("invalid response;");
  auto options = quietOptions();
  options.send_period = 5ms;
  auto session = fixture.makeSession(options);
  const auto started = session->start();
  ASSERT_TRUE(started.success) << started.message;

  const std::array<double, 4> target{{0.0, 0.0, 0.2, 0.0}};
  fixture.updateFeedback(target);
  ASSERT_TRUE(session->updateServoJTarget(started.lease_id, target));
  ASSERT_TRUE(
    waitUntil(
      [&session]() {
        return session->getSnapshot().state == ServoControlSession::State::IDLE;
      }));

  EXPECT_EQ(1U, fixture.tcp.commands().size());
  EXPECT_EQ(1U, fixture.stop_strategy->callCount());
  const auto error = fixture.operational_state->getSnapshot();
  EXPECT_EQ(ServoOperationalErrorCode::MOTION_TCP_SEND_FAILED, error.code);
  EXPECT_NE(std::string::npos, error.message.find("Invalid Motion TCP response"));
}

TEST(ServoControlSession, SuccessfulRestartResetsPreviousTcpCommandToInitialState)
{
  SessionFixture fixture;
  auto options = quietOptions();
  options.send_period = 5ms;
  options.max_joint_step_rad = 0.01;
  auto session = fixture.makeSession(options);

  const std::array<double, 4> first{{0.0, 0.0, 0.2, 0.0}};
  auto started = session->start();
  ASSERT_TRUE(started.success) << started.message;
  fixture.updateFeedback(first);
  ASSERT_TRUE(session->updateServoJTarget(started.lease_id, first));
  ASSERT_TRUE(fixture.tcp.waitForCommandCount(1));
  ASSERT_TRUE(session->stop().success);

  ASSERT_TRUE(
    fixture.safety_state->reportViolation(
      ServoSafetyViolationCode::SERVO_J_COUPLED_LIMIT, "old safety fault"));
  ASSERT_TRUE(
    fixture.operational_state->reportError(
      ServoOperationalErrorCode::MOTION_TCP_SEND_FAILED, "old operational fault"));

  const std::array<double, 4> restarted_target{{0.5, 0.0, 0.2, 0.0}};
  started = session->start();
  ASSERT_TRUE(started.success) << started.message;
  EXPECT_EQ(ServoSafetyViolationCode::NONE, fixture.safety_state->getSnapshot().code);
  EXPECT_EQ(ServoOperationalErrorCode::NONE, fixture.operational_state->getSnapshot().code);
  fixture.updateFeedback(restarted_target);
  ASSERT_TRUE(session->updateServoJTarget(started.lease_id, restarted_target));
  ASSERT_TRUE(fixture.tcp.waitForCommandCount(2));
  EXPECT_EQ(ServoSafetyViolationCode::NONE, fixture.safety_state->getSnapshot().code);
  EXPECT_TRUE(session->stop().success);
}

TEST(ServoControlSession, ADelayedSendDoesNotCauseCatchUpBurst)
{
  SessionFixture fixture;
  fixture.tcp.blockFirstSend();
  auto options = quietOptions();
  options.send_period = 10ms;
  auto session = fixture.makeSession(options);
  const auto started = session->start();
  ASSERT_TRUE(started.success) << started.message;
  const std::array<double, 4> target{{0.1, 0.2, 0.3, 0.4}};
  fixture.updateFeedback(target);
  ASSERT_TRUE(session->updateServoJTarget(started.lease_id, target));

  ASSERT_TRUE(fixture.tcp.waitForCommandCount(1));
  std::this_thread::sleep_for(50ms);
  fixture.tcp.releaseFirstSend();
  std::this_thread::sleep_for(4ms);
  EXPECT_EQ(1U, fixture.tcp.commands().size());
  ASSERT_TRUE(fixture.tcp.waitForCommandCount(2));
  EXPECT_TRUE(session->stop().success);
}

TEST(ServoControlSession, TargetAfterAnElapsedWatchdogCannotReviveABlockedWorker)
{
  SessionFixture fixture;
  fixture.tcp.blockFirstSend();
  auto options = quietOptions();
  options.send_period = 5ms;
  options.target_watchdog_timeout = 20ms;
  auto session = fixture.makeSession(options);
  const auto started = session->start();
  ASSERT_TRUE(started.success) << started.message;
  fixture.updateFeedback({{0.1, 0.2, 0.3, 0.4}});
  ASSERT_TRUE(session->updateServoJTarget(started.lease_id, {{0.1, 0.2, 0.3, 0.4}}));
  ASSERT_TRUE(fixture.tcp.waitForCommandCount(1));

  std::this_thread::sleep_for(25ms);
  EXPECT_FALSE(session->updateServoJTarget(started.lease_id, {{0.5, 0.6, 0.7, 0.8}}));
  fixture.tcp.releaseFirstSend();

  ASSERT_TRUE(
    waitUntil(
      [&session]() {
        return session->getSnapshot().state == ServoControlSession::State::IDLE;
      }));
  EXPECT_EQ(1U, fixture.stop_strategy->callCount());
  EXPECT_EQ(
    ServoOperationalErrorCode::WATCHDOG_TIMEOUT,
    fixture.operational_state->getSnapshot().code);
}

TEST(ServoControlSession, MissingFirstTargetTriggersWatchdogAndReleasesOnlyAfterConfirmedStop)
{
  SessionFixture fixture;
  auto options = quietOptions();
  options.target_watchdog_timeout = 25ms;
  auto session = fixture.makeSession(options);
  const auto started = session->start();
  ASSERT_TRUE(started.success) << started.message;

  ASSERT_TRUE(
    waitUntil(
      [&session]() {
        return session->getSnapshot().state == ServoControlSession::State::IDLE;
      }));
  EXPECT_EQ(1U, fixture.stop_strategy->callCount());
  EXPECT_EQ(Manager::NO_LEASE, fixture.manager->getSnapshot().lease_id);
  EXPECT_TRUE(fixture.tcp.commands().empty());
  EXPECT_EQ(
    ServoOperationalErrorCode::WATCHDOG_TIMEOUT,
    fixture.operational_state->getSnapshot().code);
}

TEST(ServoControlSession, FailedWatchdogStopRetainsLeaseAndSameLeaseCanRetry)
{
  SessionFixture fixture;
  fixture.stop_strategy->addResult(
    ServoStopStrategy::Status::CONFIRMATION_TIMEOUT, "mode confirmation failed");
  fixture.stop_strategy->addResult(ServoStopStrategy::Status::SUCCESS, "retry confirmed");
  auto options = quietOptions();
  options.target_watchdog_timeout = 20ms;
  auto session = fixture.makeSession(options);
  const auto started = session->start();
  ASSERT_TRUE(started.success) << started.message;

  ASSERT_TRUE(
    waitUntil(
      [&session]() {
        return session->getSnapshot().state == ServoControlSession::State::FAULTED;
      }));
  auto manager_snapshot = fixture.manager->getSnapshot();
  EXPECT_EQ(started.lease_id, manager_snapshot.lease_id);
  EXPECT_FALSE(manager_snapshot.accepting_servo_targets);

  const auto retried = session->stop();
  EXPECT_TRUE(retried.success) << retried.message;
  EXPECT_EQ(2U, fixture.stop_strategy->callCount());
  EXPECT_EQ(Manager::NO_LEASE, fixture.manager->getSnapshot().lease_id);
  EXPECT_EQ(
    ServoOperationalErrorCode::WATCHDOG_TIMEOUT,
    fixture.operational_state->getSnapshot().code);
}

TEST(ServoControlSession, ForeignAndStaleStopCannotAffectANewerLease)
{
  SessionFixture fixture;
  auto session = fixture.makeSession(quietOptions());
  const auto old = session->start();
  ASSERT_TRUE(old.success) << old.message;

  fixture.manager->updateRobotStatus(false, RobotMode::INVALID);
  fixture.manager->updateRobotStatus(true, RobotMode::ENABLE);
  const auto current = fixture.manager->tryAcquire(Manager::State::SERVO_J);
  ASSERT_TRUE(current.success) << current.message;
  ASSERT_NE(old.lease_id, current.lease_id);

  const auto stale_stop = session->stop();
  EXPECT_FALSE(stale_stop.success);
  const auto snapshot = fixture.manager->getSnapshot();
  EXPECT_EQ(current.lease_id, snapshot.lease_id);
  EXPECT_EQ(Manager::State::SERVO_J, snapshot.control_state);
  EXPECT_EQ(0U, fixture.stop_strategy->callCount());

  ASSERT_TRUE(fixture.manager->beginServoStop(current.lease_id).success);
  ASSERT_TRUE(
    fixture.manager->release(Manager::State::SERVO_J, current.lease_id).success);
}

TEST(ServoControlSession, StaleStopCompletionCannotReleaseLeaseAcquiredDuringStrategy)
{
  FakeMotionTcpInterface tcp;
  auto manager = std::make_shared<Manager>();
  manager->updateRobotStatus(true, RobotMode::ENABLE);
  auto commander = std::make_shared<mg400_interface::MotionCommander>(&tcp);
  Manager::LeaseId newer_lease = Manager::NO_LEASE;
  auto strategy = std::make_shared<FunctionStopStrategy>(
    [manager, &newer_lease]() {
      manager->updateRobotStatus(false, RobotMode::INVALID);
      manager->updateRobotStatus(true, RobotMode::ENABLE);
      const auto acquired = manager->tryAcquire(Manager::State::SERVO_J);
      if (acquired.success) {
        newer_lease = acquired.lease_id;
      }
      return ServoStopStrategy::Result{
        ServoStopStrategy::Status::SUCCESS, "old ResetRobot completion"};
    });
  auto safety_state = std::make_shared<ServoSafetyViolationState>();
  auto operational_state = std::make_shared<ServoOperationalErrorState>();
  RealtimeDataSnapshot feedback_snapshot;
  feedback_snapshot.connection_epoch = 1;
  ServoControlSession session(
    manager, commander, strategy, safety_state, operational_state,
    [&feedback_snapshot]() {return feedback_snapshot;},
    quietOptions());
  const auto old = session.start();
  ASSERT_TRUE(old.success) << old.message;

  const auto stopped = session.stop();
  EXPECT_FALSE(stopped.success);
  ASSERT_NE(Manager::NO_LEASE, newer_lease);
  const auto snapshot = manager->getSnapshot();
  EXPECT_EQ(newer_lease, snapshot.lease_id);
  EXPECT_EQ(Manager::State::SERVO_J, snapshot.control_state);

  ASSERT_TRUE(manager->beginServoStop(newer_lease).success);
  ASSERT_TRUE(manager->release(Manager::State::SERVO_J, newer_lease).success);
}

TEST(ResetRobotStopStrategy, RequiresResetSuccessAndEnableConfirmation)
{
  std::atomic<int> reset_count{0};
  std::atomic<int> read_count{0};
  ResetRobotStopStrategy strategy(
    [&reset_count]() {++reset_count;},
    [&read_count](std::uint64_t & mode) {
      mode = ++read_count >= 2 ? RobotMode::ENABLE : RobotMode::RUNNING;
      return true;
    });

  const auto result = strategy.stop(std::chrono::steady_clock::now() + 100ms);
  EXPECT_TRUE(result.success()) << result.message;
  EXPECT_EQ(1, reset_count.load());
  EXPECT_GE(read_count.load(), 2);
}

TEST(ResetRobotStopStrategy, ResetFailureIsNotReportedAsStopped)
{
  ResetRobotStopStrategy strategy(
    []() {throw std::runtime_error("controller rejected ResetRobot");},
    [](std::uint64_t &) {return true;});

  const auto result = strategy.stop(std::chrono::steady_clock::now() + 100ms);
  EXPECT_EQ(ServoStopStrategy::Status::RESET_FAILED, result.status);
  EXPECT_FALSE(result.success());
}

TEST(ResetRobotStopStrategy, EnableConfirmationUsesTheProvidedSteadyDeadline)
{
  ResetRobotStopStrategy::Options options;
  options.confirmation_poll_period = 1ms;
  ResetRobotStopStrategy strategy(
    []() {},
    [](std::uint64_t & mode) {
      mode = RobotMode::RUNNING;
      return true;
    }, options);

  const auto result = strategy.stop(std::chrono::steady_clock::now() + 10ms);
  EXPECT_EQ(ServoStopStrategy::Status::CONFIRMATION_TIMEOUT, result.status);
  EXPECT_FALSE(result.success());
}

}  // namespace
