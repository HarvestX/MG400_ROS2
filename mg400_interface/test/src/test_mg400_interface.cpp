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

#include <string>

#include <mg400_interface/mg400_interface.hpp>

TEST(TestMG400Interface, ServoModeActiveState)
{
  auto interface = mg400_interface::MG400Interface("127.0.0.1");

  EXPECT_FALSE(interface.isServoModeActive());

  interface.setServoModeActive(true);
  EXPECT_TRUE(interface.isServoModeActive());

  interface.setServoModeActive(false);
  EXPECT_FALSE(interface.isServoModeActive());
}

TEST(TestMG400Interface, ServoModeExitRequestWithoutCallbackClearsActiveState)
{
  auto interface = mg400_interface::MG400Interface("127.0.0.1");

  interface.setServoModeActive(true);
  interface.requestServoModeExit("test");

  EXPECT_FALSE(interface.isServoModeActive());
}

TEST(TestMG400Interface, ServoModeExitRequestInvokesCallback)
{
  auto interface = mg400_interface::MG400Interface("127.0.0.1");
  bool called = false;
  std::string reason;
  bool warn = false;

  interface.setServoModeActive(true);
  interface.setServoModeExitCallback(
    [&](const std::string & callback_reason, const bool callback_warn) {
      called = true;
      reason = callback_reason;
      warn = callback_warn;
      interface.setServoModeActive(false);
    });

  interface.requestServoModeExit("dashboard stop", true);

  EXPECT_TRUE(called);
  EXPECT_EQ("dashboard stop", reason);
  EXPECT_TRUE(warn);
  EXPECT_FALSE(interface.isServoModeActive());
}

TEST(TestMG400Interface, DashboardStopCommandExitFlag)
{
  auto interface = mg400_interface::MG400Interface("127.0.0.1");

  EXPECT_TRUE(interface.shouldExitServoModeOnDashboardStopCommand());

  interface.setServoExitOnDashboardStopCommand(false);
  EXPECT_FALSE(interface.shouldExitServoModeOnDashboardStopCommand());

  interface.setServoExitOnDashboardStopCommand(true);
  EXPECT_TRUE(interface.shouldExitServoModeOnDashboardStopCommand());
}
