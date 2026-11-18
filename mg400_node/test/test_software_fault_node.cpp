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
#include <mg400_node/mg400_node.hpp>
#include <functional>
#include <thread>

namespace mg400_node
{
using namespace std::chrono_literals;

class SoftwareFaultNodeTest : public testing::Test
{
protected:
  std::shared_ptr<MG400Node> node;
  std::shared_ptr<rclcpp::Node> server;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service;
  int requests = 0;

  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {
        rclcpp::Parameter("auto_configure", false),
        rclcpp::Parameter("auto_connect", false),
        rclcpp::Parameter("software_fault_service", "/test_mg400_emergency")});
    node = std::make_shared<MG400Node>(options);
    node->fault_timer_->cancel();
    // Test seam: inject the real interface latch without constructing sockets
    // or loading robot command plugins. No robot connection is possible.
    node->interface_ = std::make_shared<mg400_interface::MG400Interface>("127.0.0.1");
    node->fault_stop_acknowledged_ = true;
    server = std::make_shared<rclcpp::Node>("fault_test_server");
    executor.add_node(node->get_node_base_interface());
    executor.add_node(server);
  }

  void TearDown() override
  {
    executor.remove_node(node->get_node_base_interface());
    executor.remove_node(server);
    node->interface_.reset();
    node.reset();
    server.reset();
  }

  void fail()
  {
    node->interface_->getSoftwareFault()->run(
      "test", []() {throw std::runtime_error("injected");});
  }

  bool spinUntil(const std::function<bool()> & predicate)
  {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(1ms);
    }
    return predicate();
  }

  void serve(bool accepted)
  {
    service = server->create_service<std_srvs::srv::Trigger>(
      "/test_mg400_emergency",
      [this, accepted](std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        ++requests;
        response->success = accepted;
      });
    ASSERT_TRUE(spinUntil([this]() {return node->fault_client_->service_is_ready();}));
  }

  void tick() {node->onSoftwareFaultTimer();}
  bool pending() {return node->fault_request_.has_value();}
  bool ready()
  {
    return pending() && node->fault_request_->wait_for(0s) == std::future_status::ready;
  }
  bool accepted() {return node->fault_notification_accepted_;}
  bool faulted() {return node->hasSoftwareFault();}
  void retryNow() {node->next_notification_ = {};}
  void expire() {node->fault_request_started_ -= 6s;}
  int64_t requestId() {return node->fault_request_->request_id;}
  bool requestWasRemoved(int64_t id) {return !node->fault_client_->remove_pending_request(id);}
  CallbackReturn cleanup() {return node->on_cleanup(State());}
  CallbackReturn configure() {return node->on_configure(State());}
  CallbackReturn deactivate() {return node->on_deactivate(State());}
  void stopPending(std::future<bool> future)
  {
    node->fault_stop_acknowledged_ = false;
    node->fault_stop_ = std::move(future);
  }
  bool stopAccepted() {return node->fault_stop_acknowledged_;}
  void enableStop() {node->fault_stop_acknowledged_ = false;}
  bool stopInFlight() {return node->fault_stop_.valid();}
  void disableDestination() {node->fault_client_.reset();}
};

TEST_F(SoftwareFaultNodeTest, HealthyNodeDoesNotNotify)
{
  serve(true);
  tick();
  EXPECT_FALSE(pending());
  EXPECT_EQ(0, requests);
}

TEST_F(SoftwareFaultNodeTest, AcceptedRequestDoesNotClearLocalFault)
{
  serve(true);
  fail();
  tick();
  ASSERT_TRUE(pending());
  ASSERT_TRUE(spinUntil([this]() {return ready();}));
  tick();
  EXPECT_TRUE(accepted());
  EXPECT_TRUE(faulted());
  tick();
  EXPECT_FALSE(pending());
  EXPECT_EQ(1, requests);
}

TEST_F(SoftwareFaultNodeTest, RejectedRequestRetriesAndRetainsFault)
{
  serve(false);
  fail();
  tick();
  ASSERT_TRUE(spinUntil([this]() {return ready();}));
  tick();
  EXPECT_FALSE(accepted());
  tick();
  EXPECT_FALSE(pending());
  retryNow();
  tick();
  ASSERT_TRUE(spinUntil([this]() {return ready();}));
  tick();
  EXPECT_EQ(2, requests);
  EXPECT_TRUE(faulted());
}

TEST_F(SoftwareFaultNodeTest, UnavailableServiceCanRecoverWithoutRestart)
{
  fail();
  tick();
  EXPECT_FALSE(pending());
  EXPECT_TRUE(faulted());
  serve(true);
  retryNow();
  tick();
  ASSERT_TRUE(spinUntil([this]() {return ready();}));
  tick();
  EXPECT_TRUE(accepted());
}

TEST_F(SoftwareFaultNodeTest, TimeoutRemovesPendingRequestBeforeRetry)
{
  serve(true);
  fail();
  tick();
  ASSERT_TRUE(pending());
  const auto old_id = requestId();
  // Do not spin the server: the request remains unanswered.
  tick();
  EXPECT_EQ(old_id, requestId());
  expire();
  tick();
  EXPECT_FALSE(pending());
  EXPECT_TRUE(requestWasRemoved(old_id));
  EXPECT_FALSE(accepted());
  retryNow();
  tick();
  ASSERT_TRUE(pending());
  EXPECT_NE(old_id, requestId());
  ASSERT_TRUE(spinUntil([this]() {return ready();}));
  tick();
  EXPECT_TRUE(accepted());
  EXPECT_TRUE(faulted());
}

TEST_F(SoftwareFaultNodeTest, BlockedStopDoesNotBlockNotification)
{
  serve(true);
  fail();
  std::promise<bool> stop;
  stopPending(stop.get_future());
  tick();
  const bool response_ready = spinUntil([this]() {return ready();});
  tick();
  const bool notified = accepted();
  // Release even if the assertions fail; the node owns and joins this work.
  stop.set_value(false);
  tick();
  EXPECT_TRUE(response_ready);
  EXPECT_TRUE(notified);
  EXPECT_FALSE(stopAccepted());
  EXPECT_TRUE(faulted());
}

TEST_F(SoftwareFaultNodeTest, LifecycleRecoveryCannotEraseFault)
{
  fail();
  EXPECT_EQ(CallbackReturn::FAILURE, cleanup());
  EXPECT_EQ(CallbackReturn::FAILURE, configure());
  EXPECT_EQ(CallbackReturn::FAILURE, deactivate());
  EXPECT_TRUE(faulted());
}

TEST_F(SoftwareFaultNodeTest, MissingDestinationRetainsFaultAndStillAttemptsStop)
{
  disableDestination();
  enableStop();
  fail();
  tick();
  EXPECT_FALSE(accepted());
  EXPECT_FALSE(pending());
  ASSERT_TRUE(stopInFlight());
  EXPECT_TRUE(spinUntil([this]() {tick(); return !stopInFlight();}));
  EXPECT_FALSE(stopAccepted());
  EXPECT_TRUE(faulted());
}
}  // namespace mg400_node
