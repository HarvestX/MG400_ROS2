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

#include <gtest/gtest.h>
#include <mg400_plugin/motion_api/command_queue.hpp>
#include <rcl_action/goal_handle.h>
#include <stdexcept>
#include <thread>

namespace mg400_plugin
{
class CommandQueueTestAccess
{
public:
  static void configure(
    CommandQueue & queue, const mg400_interface::MG400Interface::SharedPtr & interface,
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr & clock,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logger,
    const mg400_interface::MotionCommander::SharedPtr & commander)
  {
    queue.mg400_interface_ = interface;
    queue.node_clock_if_ = clock;
    queue.node_logging_if_ = logger;
    queue.commander_ = commander;
  }
  static void execute(CommandQueue & queue, const std::shared_ptr<CommandQueue::GoalHandle> & goal)
  {
    queue.execute(goal);
  }
  static void accept(CommandQueue & queue, const std::shared_ptr<CommandQueue::GoalHandle> & goal)
  {
    queue.handle_accepted(goal);
  }
};
}  // namespace mg400_plugin

namespace
{
using Queue = mg400_plugin::CommandQueue;
using Action = Queue::ActionT;
using Response = Action::Impl::GetResultService::Response;

class Goal : public Queue::GoalHandle
{
  static std::shared_ptr<rcl_action_goal_handle_t> makeHandle()
  {
    auto handle = std::shared_ptr<rcl_action_goal_handle_t>(
      new rcl_action_goal_handle_t(rcl_action_get_zero_initialized_goal_handle()),
      [](rcl_action_goal_handle_t * p) {
        EXPECT_EQ(RCL_RET_OK, rcl_action_goal_handle_fini(p));
        delete p;
      });
    rcl_action_goal_info_t info{};
    if (rcl_action_goal_handle_init(
        handle.get(), &info,
        rcl_get_default_allocator()) != RCL_RET_OK)
    {
      throw std::runtime_error("goal initialization failed");
    }
    return handle;
  }

public:
  Goal(
    const std::shared_ptr<Action::Goal> & goal,
    std::function<void(const rclcpp_action::GoalUUID &, std::shared_ptr<void>)> terminal)
  : Queue::GoalHandle(makeHandle(), {}, goal, terminal, [](const auto &) {},
      [](const auto &) {})
  {
    execute();
  }
  void cancel() {this->_cancel_goal();}
};

class Clock : public rclcpp::node_interfaces::NodeClockInterface
{
public:
  int injection = 0;
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>();
  rclcpp::Clock::SharedPtr get_clock() override
  {
    switch (injection) {
      case 1: throw std::runtime_error("injected command worker");
      case 2: throw 42;
      case 3: throw mg400_interface::TcpSocketException("injected TCP failure");
      default: return clock;
    }
  }
  rclcpp::Clock::ConstSharedPtr get_clock() const override {return clock;}
};

class Logger : public rclcpp::node_interfaces::NodeLoggingInterface
{
public:
  rclcpp::Logger get_logger() const override {return rclcpp::get_logger("queue_test");}
  const char * get_logger_name() const override {return "queue_test";}
};

class Motion : public mg400_interface::MotionTcpInterfaceBase
{
public:
  std::shared_ptr<mg400_interface::SoftwareFault> fault;
  int commands = 0;
  bool throw_on_send = false;
  void sendCommand(const std::string &) override
  {
    ++commands;
    if (throw_on_send) {throw std::runtime_error("injected motion send failure");}
    fault->run(
      "receiver during command batch",
      []() {throw std::runtime_error("receiver failed");});
  }
};

class CommandQueueFault : public testing::Test
{
protected:
  Queue queue;
  mg400_interface::MG400Interface::SharedPtr interface =
    std::make_shared<mg400_interface::MG400Interface>("127.0.0.1");
  std::shared_ptr<Clock> clock = std::make_shared<Clock>();
  Motion motion;
  int deliveries = 0;
  std::shared_ptr<Response> response;
  bool delivery_throws = false;

  void SetUp() override
  {
    interface->realtime_tcp_interface =
      std::make_shared<mg400_interface::RealtimeFeedbackTcpInterface>(
      "127.0.0.1", "", interface->getSoftwareFault());
    motion.fault = interface->getSoftwareFault();
    mg400_plugin::CommandQueueTestAccess::configure(
      queue, interface, clock, std::make_shared<Logger>(),
      std::make_shared<mg400_interface::MotionCommander>(&motion));
  }

  std::shared_ptr<Goal> goal(
    const std::shared_ptr<Action::Goal> & request =
    std::make_shared<Action::Goal>())
  {
    return std::make_shared<Goal>(
      request, [this](const auto &, std::shared_ptr<void> value) {
        ++deliveries;
        response = std::static_pointer_cast<Response>(value);
        if (delivery_throws) {
          throw std::runtime_error("injected result transport failure");
        }
      });
  }

  void run(const std::shared_ptr<Goal> & goal)
  {
    // Use the real worker on a thread. No node, DDS endpoint or robot is created.
    std::thread worker([&]() {mg400_plugin::CommandQueueTestAccess::execute(queue, goal);});
    worker.join();
  }
};

TEST_F(CommandQueueFault, EmptyGoalSucceedsWithoutSoftwareFault)
{
  run(goal());
  ASSERT_NE(nullptr, response);
  EXPECT_EQ(action_msgs::msg::GoalStatus::STATUS_SUCCEEDED, response->status);
  EXPECT_TRUE(response->result.result);
  EXPECT_FALSE(response->result.software_fault);
  EXPECT_FALSE(interface->getSoftwareFault()->failed());
}

TEST_F(CommandQueueFault, StandardExceptionAbortsAndLogsBoundary)
{
  clock->injection = 1;
  testing::internal::CaptureStderr();
  run(goal());
  const auto log = testing::internal::GetCapturedStderr();
  ASSERT_NE(nullptr, response);
  EXPECT_EQ(action_msgs::msg::GoalStatus::STATUS_ABORTED, response->status);
  EXPECT_FALSE(response->result.result);
  EXPECT_TRUE(response->result.software_fault);
  EXPECT_NE(std::string::npos, log.find("boundary=CommandQueue::execute"));
  EXPECT_NE(std::string::npos, log.find("injected command worker"));
}

TEST_F(CommandQueueFault, UnknownExceptionDoesNotEscapeWorker)
{
  clock->injection = 2;
  run(goal());
  ASSERT_NE(nullptr, response);
  EXPECT_EQ(action_msgs::msg::GoalStatus::STATUS_ABORTED, response->status);
  EXPECT_TRUE(response->result.software_fault);
}

TEST_F(CommandQueueFault, TcpExceptionIsAnOrdinaryAbort)
{
  clock->injection = 3;
  run(goal());
  ASSERT_NE(nullptr, response);
  EXPECT_EQ(action_msgs::msg::GoalStatus::STATUS_ABORTED, response->status);
  EXPECT_FALSE(response->result.software_fault);
  EXPECT_FALSE(interface->getSoftwareFault()->failed());
}

TEST_F(CommandQueueFault, AlreadyLatchedFaultRejectsAcceptedWork)
{
  interface->getSoftwareFault()->run("receiver", []() {throw 42;});
  auto handle = goal();
  mg400_plugin::CommandQueueTestAccess::accept(queue, handle);
  ASSERT_NE(nullptr, response);
  EXPECT_EQ(action_msgs::msg::GoalStatus::STATUS_ABORTED, response->status);
  EXPECT_TRUE(response->result.software_fault);
  EXPECT_EQ(0, motion.commands);
}

TEST_F(CommandQueueFault, CancellationDoesNotHideSoftwareFault)
{
  clock->injection = 1;
  auto handle = goal();
  handle->cancel();
  run(handle);
  ASSERT_NE(nullptr, response);
  EXPECT_EQ(action_msgs::msg::GoalStatus::STATUS_CANCELED, response->status);
  EXPECT_FALSE(response->result.result);
  EXPECT_TRUE(response->result.software_fault);
}

TEST_F(CommandQueueFault, OrdinaryCancellationHasNoSoftwareFault)
{
  auto handle = goal();
  handle->cancel();
  run(handle);
  ASSERT_NE(nullptr, response);
  EXPECT_EQ(action_msgs::msg::GoalStatus::STATUS_CANCELED, response->status);
  EXPECT_FALSE(response->result.result);
  EXPECT_FALSE(response->result.software_fault);
}

TEST_F(CommandQueueFault, ReceiverFaultStopsRemainingCommandsInBatch)
{
  auto request = std::make_shared<Action::Goal>();
  mg400_msgs::msg::Command command;
  command.command_type = mg400_msgs::msg::Command::CT_JOINT_MOV_J;
  command.joint_mov_j_params.joint_angles[0] = 0.1;
  request->commands = {command, command, command};
  run(goal(request));
  EXPECT_EQ(1, motion.commands);
  ASSERT_NE(nullptr, response);
  EXPECT_EQ(action_msgs::msg::GoalStatus::STATUS_ABORTED, response->status);
  EXPECT_TRUE(response->result.software_fault);
}


TEST_F(CommandQueueFault, SendExceptionRecordsOperationAndStopsBatch)
{
  motion.throw_on_send = true;
  auto request = std::make_shared<Action::Goal>();
  mg400_msgs::msg::Command command;
  command.command_type = mg400_msgs::msg::Command::CT_JOINT_MOV_J;
  command.joint_mov_j_params.joint_angles[0] = 0.1;
  request->commands = {command, command};
  testing::internal::CaptureStderr();
  run(goal(request));
  const auto log = testing::internal::GetCapturedStderr();
  EXPECT_EQ(1, motion.commands);
  ASSERT_NE(nullptr, response);
  EXPECT_EQ(action_msgs::msg::GoalStatus::STATUS_ABORTED, response->status);
  EXPECT_TRUE(response->result.software_fault);
  EXPECT_NE(std::string::npos, log.find("operation=CommandQueue::sendJointMovJ"));
  EXPECT_NE(std::string::npos, log.find("operation_callsite="));
  EXPECT_NE(std::string::npos, log.find("injected motion send failure"));
}

TEST_F(CommandQueueFault, DeliveryFailureIsContainedWithoutSecondTerminalTransition)
{
  delivery_throws = true;
  auto handle = goal();
  run(handle);
  EXPECT_EQ(1, deliveries);
  EXPECT_FALSE(handle->is_active());
  EXPECT_TRUE(interface->getSoftwareFault()->failed());
  run(handle);
  EXPECT_EQ(1, deliveries);
}
}  // namespace
