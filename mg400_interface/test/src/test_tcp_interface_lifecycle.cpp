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

#include "mg400_interface/tcp_interface/dashboard_tcp_interface.hpp"
#include "mg400_interface/tcp_interface/realtime_feedback_tcp_interface.hpp"

namespace
{

TEST(TestTcpInterfaceLifecycle, DashboardDisconnectBeforeInitIsSafe)
{
  mg400_interface::DashboardTcpInterface interface("127.0.0.1");

  EXPECT_NO_THROW(interface.disConnect());
  EXPECT_NO_THROW(interface.disConnect());
}

TEST(TestTcpInterfaceLifecycle, RealtimeDisconnectBeforeInitIsSafe)
{
  mg400_interface::RealtimeFeedbackTcpInterface interface("127.0.0.1");

  EXPECT_NO_THROW(interface.disConnect());
  EXPECT_NO_THROW(interface.disConnect());
}

}  // namespace
