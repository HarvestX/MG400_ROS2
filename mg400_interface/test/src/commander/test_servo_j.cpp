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
#include <cmath>
#include <limits>

#include <gmock/gmock.h>

#include "mg400_interface/commander/motion_commander.hpp"

namespace
{

class MockMotionTcp : public mg400_interface::MotionTcpInterfaceBase
{
public:
  MOCK_METHOD(void, sendCommand, (const std::string &), (override));
};

TEST(ServoJCommander, FourAxisFormatMatchesMG400Example)
{
  MockMotionTcp tcp;
  mg400_interface::MotionCommander commander(&tcp);
  EXPECT_CALL(
    tcp, sendCommand(
      testing::StrEq(
        "ServoJ(90.000000,-45.000000,0.000000,180.000000,t=0.0500)\n")));
  commander.servoJ({M_PI_2, -M_PI_4, 0.0, M_PI}, 0.05);
}

TEST(ServoJCommander, RejectsInvalidTarget)
{
  MockMotionTcp tcp;
  mg400_interface::MotionCommander commander(&tcp);
  EXPECT_CALL(tcp, sendCommand(testing::_)).Times(0);
  EXPECT_THROW(
    commander.servoJ({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0}, 0.1),
    std::invalid_argument);
  EXPECT_THROW(
    commander.servoJ({0.0, 0.0, 0.0, 0.0}, 0.001),
    std::invalid_argument);
}

}  // namespace
