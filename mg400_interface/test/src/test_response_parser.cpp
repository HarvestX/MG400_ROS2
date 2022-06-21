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
#include <mg400_interface/response_parser.hpp>

TEST(MessageHandler, ParseErrorMessage) {
  const std::string response =
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

  std::cout << response << std::endl;
  auto res =
    mg400_interface::ResponseParser::parseErrorMessage(response);

  ASSERT_EQ(res.at(0).at(0), 69);
  ASSERT_EQ(res.at(0).at(1), 72);
  ASSERT_EQ(res.at(1).at(0), 1);

  ASSERT_TRUE(res.at(2).empty());
  ASSERT_TRUE(res.at(3).empty());
  ASSERT_TRUE(res.at(4).empty());
  ASSERT_TRUE(res.at(5).empty());
}
