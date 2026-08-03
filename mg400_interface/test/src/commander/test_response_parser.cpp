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
#include <mg400_interface/commander/response_parser.hpp>

TEST(ResponseParser, ParseResponse) {
  const std::string packet = "0,{5},RobotMode();";

  mg400_interface::DashboardResponse response;
  mg400_interface::ResponseParser::parseResponse(packet, response);

  ASSERT_EQ(response.error_id, 0);
  ASSERT_EQ(response.ret_val, "{5}");
  ASSERT_EQ(response.func_name, "RobotMode()");
}

TEST(ResponseParser, takeErrorMessage) {
  const std::string packet =
    R"(0,{[
	[
		69,
		72
	],
	[1],
	[],
	[],
	[],
	[]
]
},GetErrorID();)";

  mg400_interface::DashboardResponse response;
  mg400_interface::ResponseParser::parseResponse(packet, response);

  auto res =
    mg400_interface::ResponseParser::takeErrorMessage(response.ret_val);

  ASSERT_EQ(res.at(0).at(0), 69);
  ASSERT_EQ(res.at(0).at(1), 72);
  ASSERT_EQ(res.at(1).at(0), 1);

  ASSERT_TRUE(res.at(2).empty());
  ASSERT_TRUE(res.at(3).empty());
  ASSERT_TRUE(res.at(4).empty());
  ASSERT_TRUE(res.at(5).empty());
}

TEST(ResponseParser, takeAngleArray)
{
  const std::string packet = "0,"
    "{0.000000,0.000000,45.000000,45.000000,0.000000,0.000000}"
    ",GetAngle();";

  mg400_interface::DashboardResponse response;
  mg400_interface::ResponseParser::parseResponse(packet, response);

  const auto res = mg400_interface::ResponseParser::takeAngleArray(
    response.ret_val);
  ASSERT_DOUBLE_EQ(res.at(0), 0.0);
  ASSERT_DOUBLE_EQ(res.at(1), 0.0);
  ASSERT_DOUBLE_EQ(res.at(2), 0.25 * M_PI);
  ASSERT_DOUBLE_EQ(res.at(3), 0.25 * M_PI);
  ASSERT_DOUBLE_EQ(res.at(4), 0.0);
  ASSERT_DOUBLE_EQ(res.at(5), 0.0);
}

TEST(ResponseParser, takePoseArray)
{
  const std::string packet =
    "0,"
    "{350.000000,0.000000,0.000000,0.000000,0.000000,0.000000}"
    ",GetPose();";

  mg400_interface::DashboardResponse response;
  mg400_interface::ResponseParser::parseResponse(packet, response);

  const auto res = mg400_interface::ResponseParser::takePoseArray(
    response.ret_val);
  ASSERT_DOUBLE_EQ(res.at(0), 0.35);
  ASSERT_DOUBLE_EQ(res.at(1), 0.0);
  ASSERT_DOUBLE_EQ(res.at(2), 0.0);
  ASSERT_DOUBLE_EQ(res.at(3), 0.0);
  ASSERT_DOUBLE_EQ(res.at(4), 0.0);
  ASSERT_DOUBLE_EQ(res.at(5), 0.0);
}

TEST(ResponseParser, takeCartesianPoseArray)
{
  const std::string packet =
    "0,"
    "{350.000000,100.000000,200.000000,90.000000,0.000000,-180.000000}"
    ",PositiveSolution();";

  mg400_interface::DashboardResponse response;
  mg400_interface::ResponseParser::parseResponse(packet, response);

  const auto res = mg400_interface::ResponseParser::takeCartesianPoseArray(
    response.ret_val);
  ASSERT_DOUBLE_EQ(res.at(0), 0.35);
  ASSERT_DOUBLE_EQ(res.at(1), 0.1);
  ASSERT_DOUBLE_EQ(res.at(2), 0.2);
  ASSERT_DOUBLE_EQ(res.at(3), 0.5 * M_PI);
  ASSERT_DOUBLE_EQ(res.at(4), 0.0);
  ASSERT_DOUBLE_EQ(res.at(5), -M_PI);
}

TEST(ResponseParser, takeCartesianPoseArray4)
{
  const std::string packet =
    "-1,"
    "{109.499985,0.000000,296.278992,0.000000}"
    ",PositiveSolution();";

  mg400_interface::DashboardResponse response;
  mg400_interface::ResponseParser::parseResponse(packet, response);

  const auto res = mg400_interface::ResponseParser::takeCartesianPoseArray4(
    response.ret_val);
  ASSERT_DOUBLE_EQ(res.at(0), 0.109499985);
  ASSERT_DOUBLE_EQ(res.at(1), 0.0);
  ASSERT_DOUBLE_EQ(res.at(2), 0.296278992);
  ASSERT_DOUBLE_EQ(res.at(3), 0.0);
}

TEST(ResponseParser, takeAngleArray4)
{
  const std::string packet =
    "-1,"
    "{0.000000,0.000000,-90.000000,0.000000}"
    ",InverseSolution();";

  mg400_interface::DashboardResponse response;
  mg400_interface::ResponseParser::parseResponse(packet, response);

  const auto res = mg400_interface::ResponseParser::takeAngleArray4(
    response.ret_val);
  ASSERT_DOUBLE_EQ(res.at(0), 0.0);
  ASSERT_DOUBLE_EQ(res.at(1), 0.0);
  ASSERT_DOUBLE_EQ(res.at(2), -0.5 * M_PI);
  ASSERT_DOUBLE_EQ(res.at(3), 0.0);
}

TEST(ResponseParser, takeInt)
{
  const std::string packet =
    "0,"
    "{1}"
    ",DI(2);";

  mg400_interface::DashboardResponse response;
  mg400_interface::ResponseParser::parseResponse(packet, response);

  const auto res = mg400_interface::ResponseParser::takeInt(response.ret_val);
  ASSERT_EQ(res, 1);
}

TEST(ResponseParser, ParsesEmptyReturnValueForMotionResponse)
{
  const std::string packet = "0,{},ServoJ(1.000,2.000,3.000,4.000);";
  mg400_interface::DashboardResponse response{};

  ASSERT_TRUE(mg400_interface::ResponseParser::parseResponse(packet, response));
  EXPECT_EQ(0, response.error_id);
  EXPECT_EQ("{}", response.ret_val);
  EXPECT_EQ("ServoJ(1.000,2.000,3.000,4.000)", response.func_name);
}

TEST(ResponseParser, ParsesNonzeroErrorId)
{
  const std::string packet = "-10000,{},ServoJ(1.000,2.000,3.000,4.000);";
  mg400_interface::DashboardResponse response{};

  ASSERT_TRUE(mg400_interface::ResponseParser::parseResponse(packet, response));
  EXPECT_EQ(-10000, response.error_id);
}

TEST(ResponseParser, RejectsIncompleteAndMalformedResponsesWithoutThrowing)
{
  const std::vector<std::string> invalid_packets{
    "",
    "0,",
    "0,{},ServoJ()",
    "not-an-error-id,{},ServoJ();",
    "0,{,ServoJ();",
    "0,ServoJ();",
    "0,{},;"};

  for (const auto & packet : invalid_packets) {
    mg400_interface::DashboardResponse response{42, "stale", "stale"};
    EXPECT_FALSE(mg400_interface::ResponseParser::parseResponse(packet, response)) << packet;
    EXPECT_EQ(0, response.error_id);
    EXPECT_TRUE(response.ret_val.empty());
    EXPECT_TRUE(response.func_name.empty());
  }
}

TEST(ResponseParser, RejectsDataAfterTerminator)
{
  mg400_interface::DashboardResponse response{};

  EXPECT_FALSE(
    mg400_interface::ResponseParser::parseResponse(
      "0,{},ServoJ();0,{},Sync();", response));
  EXPECT_FALSE(
    mg400_interface::ResponseParser::parseResponse(
      "0,{},ServoJ();unexpected", response));
}
