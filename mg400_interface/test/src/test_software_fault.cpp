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
#include <stdexcept>
#include <thread>
#include <mg400_interface/mg400_interface.hpp>
#include <mg400_interface/software_fault.hpp>
#include <mg400_interface/exception_context.hpp>

TEST(SoftwareFault, NormalOperationRemainsAvailable)
{
  mg400_interface::SoftwareFault fault;
  int calls = 0;
  EXPECT_TRUE(fault.run("test", [&]() {++calls;}));
  EXPECT_TRUE(fault.run("test", [&]() {++calls;}));
  EXPECT_EQ(2, calls);
  EXPECT_FALSE(fault.failed());
}

TEST(SoftwareFault, StandardExceptionRecordsContextAndLatches)
{
  mg400_interface::SoftwareFault fault;
  testing::internal::CaptureStderr();
  EXPECT_FALSE(fault.run("receiver/update", []() {throw std::runtime_error("injected");}));
  const auto log = testing::internal::GetCapturedStderr();
  EXPECT_NE(std::string::npos, log.find("boundary=receiver/update"));
  EXPECT_NE(std::string::npos, log.find("what=injected"));
  EXPECT_TRUE(fault.failed());
  bool called = false;
  EXPECT_FALSE(fault.run("next", [&]() {called = true;}));
  EXPECT_FALSE(called);
}

TEST(SoftwareFault, UnknownExceptionDoesNotEscapeWorker)
{
  mg400_interface::SoftwareFault fault;
  std::thread worker([&]() {fault.run("worker", []() {throw 42;});});
  worker.join();
  EXPECT_TRUE(fault.failed());
}

TEST(SoftwareFault, AllocationFailureIsContained)
{
  mg400_interface::SoftwareFault fault;
  EXPECT_FALSE(fault.run("allocation", []() {throw std::bad_alloc();}));
  EXPECT_TRUE(fault.failed());
}

TEST(SoftwareFault, InterfaceCannotReactivateAfterFault)
{
  mg400_interface::MG400Interface interface("127.0.0.1");
  interface.getSoftwareFault()->run("injected", []() {throw std::runtime_error("test");});
  // These calls must reject before accessing unconfigured sockets.
  EXPECT_FALSE(interface.ok());
  EXPECT_FALSE(interface.activate());
}

TEST(SoftwareFault, ReceiverSharesFaultAndDoesNotStartAfterFailure)
{
  auto fault = std::make_shared<mg400_interface::SoftwareFault>();
  mg400_interface::RealtimeFeedbackTcpInterface receiver("127.0.0.1", "", fault);
  fault->run("injected", []() {throw std::runtime_error("test");});
  receiver.init();
  EXPECT_FALSE(receiver.isActive());
  EXPECT_FALSE(receiver.isConnected());
  receiver.disConnect();
  // Destruction after a refused start must not dereference a missing thread.
}

namespace
{
int stop_calls = 0;
bool stop_throws = false;
}

extern "C" void __wrap__ZN15mg400_interface18DashboardCommander13emergencyStopEv(
  mg400_interface::DashboardCommander *)
{
  ++stop_calls;
  if (stop_throws) {
    throw std::runtime_error("injected stop failure");
  }
}

TEST(SoftwareFault, StopIsAttemptedDespiteLatchedUnavailableState)
{
  mg400_interface::MG400Interface interface("127.0.0.1");
  interface.dashboard_commander =
    std::make_shared<mg400_interface::DashboardCommander>(nullptr);
  stop_calls = 0;
  stop_throws = false;
  EXPECT_FALSE(interface.stopAfterSoftwareFault());
  EXPECT_EQ(0, stop_calls);
  interface.getSoftwareFault()->run("test", []() {throw 42;});
  EXPECT_TRUE(interface.stopAfterSoftwareFault());
  EXPECT_EQ(1, stop_calls);
  EXPECT_FALSE(interface.ok());
  EXPECT_FALSE(interface.activate());

  stop_throws = true;
  EXPECT_FALSE(interface.stopAfterSoftwareFault());
  EXPECT_EQ(2, stop_calls);
  EXPECT_TRUE(interface.getSoftwareFault()->failed());
  stop_throws = false;
}


TEST(SoftwareFault, OriginContextAndBoundaryAreBothRetained)
{
  mg400_interface::SoftwareFault fault;
  testing::internal::CaptureStderr();
  EXPECT_FALSE(
    fault.run(
      "receiver", []() {
        mg400_interface::withExceptionContext(
          "estimator/update", __FILE__, __LINE__,
          []() {throw std::runtime_error("origin failure");});
      }));
  const auto log = testing::internal::GetCapturedStderr();
  EXPECT_NE(std::string::npos, log.find("operation=estimator/update"));
  EXPECT_NE(std::string::npos, log.find("boundary=receiver"));
  EXPECT_NE(std::string::npos, log.find("what=origin failure"));
  EXPECT_TRUE(fault.failed());
}

TEST(SoftwareFault, MissingStopCommanderCannotClearFault)
{
  mg400_interface::MG400Interface interface("127.0.0.1");
  interface.getSoftwareFault()->run("test", []() {throw 42;});
  EXPECT_FALSE(interface.stopAfterSoftwareFault());
  EXPECT_TRUE(interface.getSoftwareFault()->failed());
}
