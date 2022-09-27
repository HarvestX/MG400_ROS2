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

#include <gtest/gtest.h>

#include <mg400_interface/error_msg_generator.hpp>

class TestErrorMsgGenerator : public ::testing::Test
{
protected:
  std::unique_ptr<mg400_interface::ErrorMsgGenerator> generator_;
  virtual void SetUp()
  {
    const std::string target = "alarm_controller.json";
    this->generator_ =
      std::make_unique<mg400_interface::ErrorMsgGenerator>(target);
    this->generator_->loadJsonFile();
  }
  virtual void TearDown()
  {}
};

TEST_F(TestErrorMsgGenerator, getErrorID) {
  const std::string expected = "Inverse kinematics error with no solution";
  const int target_id = 17;
  const std::string actual = this->generator_->get(target_id);
  ASSERT_EQ(expected, actual);
}

TEST_F(TestErrorMsgGenerator, getErrorID2) {
  const std::string expected = "No input parameters for MoveJIO instruction";
  const int target_id = 36119;
  const std::string actual = this->generator_->get(target_id);
  ASSERT_EQ(expected, actual);
}

TEST_F(TestErrorMsgGenerator, accessInvalidID) {
  const int not_exist_id = 5;
  ASSERT_THROW(this->generator_->get(not_exist_id), std::out_of_range);
}
