// Copyright 2022 HarvestX Inc.
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

#include <gmock/gmock.h>
#include <mg400_interface/commander/realtime_commander.hpp>

using ::testing::_;
using ::testing::AtLeast;
using ::testing::StrEq;


class MockTcpInterface : public mg400_interface::TcpInterfaceBase
{
public:
  MockTcpInterface()
  : mg400_interface::TcpInterfaceBase() {}

  MOCK_METHOD(void, sendCommand, (const std::string &), (override));
};


class RealtimeCommander : public ::testing::Test
{
protected:
  std::unique_ptr<mg400_interface::RealtimeCommander> commander;
  MockTcpInterface mock;
  virtual void SetUp()
  {
    this->commander =
      std::make_unique<mg400_interface::RealtimeCommander>(&this->mock);
  }

  virtual void TearDown() {}
};

TEST_F(RealtimeCommander, MovJ) {
  EXPECT_CALL(
    mock, sendCommand(
      StrEq(
        "MovJ(1.000, 2.000, 3.000, 4.000, 5.000, 6.000)"))).Times(AtLeast(1));

  commander->movJ(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
}

TEST_F(RealtimeCommander, MovJog) {
  EXPECT_CALL(
    mock, sendCommand(
      StrEq(
        "MoveJog(+j1)"))).Times(AtLeast(1));

  commander->moveJog("+j1");
}
