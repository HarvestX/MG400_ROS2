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

#include <cerrno>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>

#include "mg400_interface/exception_context.hpp"

using mg400_interface::withExceptionContext;

TEST(ExceptionContext, NormalValueAndVoidDoNotLog)
{
  testing::internal::CaptureStderr();
  EXPECT_EQ(withExceptionContext("value", "test.cpp", 11, []() {return 42;}), 42);
  int called = 0;
  withExceptionContext("void", "test.cpp", 12, [&]() {++called;});
  EXPECT_EQ(called, 1);
  EXPECT_EQ(testing::internal::GetCapturedStderr(), "");
}

TEST(ExceptionContext, PreservesReferenceAndMoveOnlyResult)
{
  int value = 7;
  int & reference = withExceptionContext(
    "reference", "test.cpp", 20, [&]() -> int & {return value;});
  EXPECT_EQ(&reference, &value);
  auto pointer = withExceptionContext(
    "move", "test.cpp", 21, []() {return std::make_unique<int>(9);});
  EXPECT_EQ(*pointer, 9);
}

TEST(ExceptionContext, PreservesExceptionIdentityAndErrno)
{
  const std::exception * original = nullptr;
  testing::internal::CaptureStderr();
  try {
    withExceptionContext(
      "allocate_packet", "receiver.cpp", 73, [&]() {
        try {
          throw std::runtime_error("packet failure");
        } catch (const std::exception & error) {
          original = &error;
          errno = EDOM;
          throw;
        }
      });
    FAIL() << "Exception was swallowed";
  } catch (const std::runtime_error & error) {
    EXPECT_EQ(&error, original);
    EXPECT_EQ(errno, EDOM);
  }
  const auto log = testing::internal::GetCapturedStderr();
  EXPECT_NE(log.find("operation=allocate_packet"), std::string::npos);
  EXPECT_NE(log.find("operation_callsite=receiver.cpp:73"), std::string::npos);
  EXPECT_NE(log.find("what=packet failure"), std::string::npos);
}

TEST(ExceptionContext, RecordsAndRethrowsBadAlloc)
{
  testing::internal::CaptureStderr();
  EXPECT_THROW(
    withExceptionContext(
      "packet", "receiver.cpp", 81, []() {throw std::bad_alloc();}),
    std::bad_alloc);
  const auto log = testing::internal::GetCapturedStderr();
  EXPECT_NE(log.find("operation=packet"), std::string::npos);
  EXPECT_NE(log.find("bad_alloc"), std::string::npos);
}

TEST(ExceptionContext, RecordsUnknownWithoutChangingPayload)
{
  testing::internal::CaptureStderr();
  try {
    withExceptionContext("foreign", "test.cpp", 91, []() {throw 123;});
    FAIL() << "Exception was swallowed";
  } catch (int value) {
    EXPECT_EQ(value, 123);
  }
  const auto log = testing::internal::GetCapturedStderr();
  EXPECT_NE(log.find("type=unknown"), std::string::npos);
  EXPECT_NE(log.find("operation_callsite=test.cpp:91"), std::string::npos);
}

TEST(ExceptionContext, KeepsInnerAndOuterOperationContext)
{
  testing::internal::CaptureStderr();
  EXPECT_THROW(
    withExceptionContext(
      "receiver", "receiver.cpp", 100, []() {
        withExceptionContext(
          "solver", "estimator.cpp", 110, []() {throw std::runtime_error("solver failure");});
      }),
    std::runtime_error);
  const auto log = testing::internal::GetCapturedStderr();
  const auto inner = log.find("operation=solver");
  const auto outer = log.find("operation=receiver");
  ASSERT_NE(inner, std::string::npos);
  ASSERT_NE(outer, std::string::npos);
  EXPECT_LT(inner, outer);
}
