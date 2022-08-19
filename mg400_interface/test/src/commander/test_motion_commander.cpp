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
#include <mg400_interface/commander/motion_commander.hpp>

using ::testing::_;
using ::testing::StrEq;


class MockTcpInterface : public mg400_interface::MotionTcpInterfaceBase
{
public:
  MockTcpInterface()
  : mg400_interface::MotionTcpInterfaceBase() {}

  MOCK_METHOD(void, sendCommand, (const std::string &), (override));
};


class TestMotionCommander : public ::testing::Test
{
protected:
  std::unique_ptr<mg400_interface::MotionCommander> commander;
  MockTcpInterface mock;
  virtual void SetUp()
  {
    this->commander =
      std::make_unique<mg400_interface::MotionCommander>(&this->mock);
  }

  virtual void TearDown() {}
};

TEST_F(TestMotionCommander, MovJ) {
  EXPECT_CALL(
    mock, sendCommand(
      StrEq(
        "MovJ(1.000,2.000,3.000,180.000,180.000,180.000)"))).Times(1);
  commander->movJ(1.0e-3, 2.0e-3, 3.0e-3, M_PI, M_PI, M_PI);
}

TEST_F(TestMotionCommander, MovL) {
  EXPECT_CALL(
    mock, sendCommand(
      StrEq(
        "MovL(1.000,2.000,3.000,180.000,180.000,180.000)"))).Times(1);
  commander->movL(1.0e-3, 2.0e-3, 3.0e-3, M_PI, M_PI, M_PI);
}


TEST_F(TestMotionCommander, JointMovJ)
{
  EXPECT_CALL(
    mock, sendCommand(
      StrEq(
        "JointMovJ(90.000,90.000,90.000,90.000,90.000,90.000)"))).Times(1);
  commander->jointMovJ(M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2);
}

TEST_F(TestMotionCommander, MovLIO) {
  EXPECT_CALL(
    mock, sendCommand(
      StrEq(
        "MovLIO(-500.000,100.000,200.000,"
        "90.000,0.000,90.000,{0,50,1,0})"))).Times(1);
  commander->movLIO(
    -500e-3, 100e-3, 200e-3, M_PI_2, 0, M_PI_2,
    mg400_interface::DistanceMode::PERCENTAGE, 50,
    mg400_interface::DOIndex::D1, mg400_interface::DOStatus::LOW);
}

TEST_F(TestMotionCommander, MovJIO) {
  EXPECT_CALL(
    mock, sendCommand(
      StrEq(
        "MovJIO(-500.000,100.000,200.000,"
        "90.000,0.000,90.000,{0,50,1,0})"))).Times(1);
  commander->movJIO(
    -500e-3, 100e-3, 200e-3, M_PI_2, 0, M_PI_2,
    mg400_interface::DistanceMode::PERCENTAGE, 50,
    mg400_interface::DOIndex::D1, mg400_interface::DOStatus::LOW);
}
/*
TEST_F(TestMotionCommander, Arc) {
  EXPECT_CALL(
    mock, sendCommand(
      StrEq(
        "Arc(-350.000,-200.000,200.000,"
        "90.000,0.000,90.000,"
        "-300.000,-250.000,200.000,"
        "90.000,0.000,90.000)"))).Times(1);
  commander->arc(
    -350e-3, -200e-3, 200e-3,
    M_PI_2, 0, M_PI_2,
    -300e-3, -250e-3, 200e-3,
    M_PI_2, 0, M_PI_2);
}
*/

TEST_F(TestMotionCommander, MoveJog) {
  EXPECT_CALL(
    mock, sendCommand(
      StrEq(
        "MoveJog(j1+)"))).Times(1);
  commander->moveJog(mg400_interface::JogMode::J1_POSITIVE);
}


TEST_F(TestMotionCommander, Sync) {
  EXPECT_CALL(
    mock, sendCommand(
      StrEq(
        "Sync()"))).Times(1);
  commander->sync();
}

TEST_F(TestMotionCommander, RelMovJUser) {
  EXPECT_CALL(
    mock, sendCommand(
      StrEq(
        "RelMovJUser(10.000,10.000,10.000,"
        "0.000,0.000,0.000,0)"))).Times(1);
  commander->relMovJUser(
    10e-3, 10e-3, 10e-3, 0, 0, 0,
    mg400_interface::UserIndex::USER0);
}

TEST_F(TestMotionCommander, RelMovLUser) {
  EXPECT_CALL(
    mock, sendCommand(
      StrEq(
        "RelMovLUser(10.000,10.000,10.000,"
        "0.000,0.000,0.000,0)"))).Times(1);
  commander->relMovLUser(
    10e-3, 10e-3, 10e-3, 0, 0, 0,
    mg400_interface::UserIndex::USER0);
}

TEST_F(TestMotionCommander, RelJointMovJ) {
  EXPECT_CALL(
    mock, sendCommand(
      StrEq(
        "RelJointMovJ(90.000,90.000,90.000,"
        "0.000,0.000,0.000)"))).Times(1);
  commander->relJointMovJ(
    M_PI_2, M_PI_2, M_PI_2,
    0, 0, 0);
}
