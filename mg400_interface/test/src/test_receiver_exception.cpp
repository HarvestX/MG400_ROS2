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
#include <atomic>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <mg400_interface/tcp_interface/realtime_feedback_tcp_interface.hpp>

namespace
{
std::atomic<int> injection{0};
std::atomic<bool> entered{false};
}

// Test-only linker substitution: no connection to a robot or network is made.
extern "C" void
__wrap__ZN15mg400_interface16TcpSocketHandler7connectERKNSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEE(
  mg400_interface::TcpSocketHandler *, const std::chrono::nanoseconds &)
{
  entered.store(true);
  switch (injection.load()) {
    case 0:
      throw std::runtime_error("injected receiver exception");
    case 1:
      throw 42;
    default:
      throw mg400_interface::TcpSocketException("injected communication failure");
  }
}

class ReceiverException : public testing::TestWithParam<int>
{
};

TEST_P(ReceiverException, RealWorkerContainsExceptionAndCanBeJoined)
{
  injection.store(GetParam());
  entered.store(false);
  auto fault = std::make_shared<mg400_interface::SoftwareFault>();
  mg400_interface::RealtimeFeedbackTcpInterface receiver("127.0.0.1", "", fault);
  testing::internal::CaptureStderr();
  receiver.init();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!entered.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  receiver.disConnect();
  const auto log = testing::internal::GetCapturedStderr();
  ASSERT_TRUE(entered.load());
  EXPECT_EQ(GetParam() != 2, fault->failed());
  EXPECT_FALSE(receiver.isActive());
  EXPECT_FALSE(receiver.isConnected());
  if (GetParam() != 2) {
    EXPECT_NE(std::string::npos, log.find("boundary=RealtimeFeedbackTcpInterface::recvData"));
    EXPECT_NE(
      std::string::npos, log.find(
        GetParam() == 0 ? "injected receiver exception" : "non-standard exception"));
    entered.store(false);
    receiver.init();
    EXPECT_FALSE(entered.load());
  }
}

INSTANTIATE_TEST_SUITE_P(
  StandardUnknownAndCommunication, ReceiverException, testing::Values(
    0, 1,
    2));


TEST(ReceiverLifetime, DestructionJoinsWorkerAfterException)
{
  injection.store(0);
  entered.store(false);
  auto fault = std::make_shared<mg400_interface::SoftwareFault>();
  {
    mg400_interface::RealtimeFeedbackTcpInterface receiver("127.0.0.1", "", fault);
    receiver.init();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!fault->failed() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_TRUE(fault->failed());
    // No explicit disconnect: a finished but joinable worker must be reaped.
  }
  EXPECT_TRUE(entered.load());
}
