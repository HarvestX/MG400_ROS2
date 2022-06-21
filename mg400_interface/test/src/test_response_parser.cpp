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
  const char * response = R"(0,{[
	[
		69,
		72
	],
	[],
	[],
	[],
	[],
	[]
]
},GetErrorID();)";

  std::cout << response << std::endl;
  ASSERT_FALSE(true);
}
