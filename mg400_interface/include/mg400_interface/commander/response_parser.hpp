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

#pragma once


#include <array>
#include <vector>
#include <string>
#include <regex>

#include "mg400_interface/command_utils.hpp"

namespace mg400_interface
{
typedef struct
{
  bool result;
  std::string ret_val;
  std::string func_name;
} DashboardResponse;

class ResponseParser
{
public:
  static bool parseResponse(const std::string &, DashboardResponse &);
  static std::array<std::vector<int>, 6> takeErrorMessage(const std::string &);
  static std::vector<double> takePoseArray(const std::string &);
  static std::vector<double> takeAngleArray(const std::string &);
  static int takeInt(const std::string &);
};
}  // namespace mg400_interface
