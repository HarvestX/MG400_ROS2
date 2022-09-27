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

  ASSERT_TRUE(response.result);
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
