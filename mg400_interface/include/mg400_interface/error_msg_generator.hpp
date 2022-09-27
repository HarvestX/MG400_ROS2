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

#include <string>
#include <memory>
#include <map>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>

namespace mg400_interface
{

class ErrorMsgGenerator
{
private:
  const rclcpp::Logger logger_;
  const std::string filename_;
  std::map<int, std::string> error_map_;

public:
  ErrorMsgGenerator() = delete;
  explicit ErrorMsgGenerator(const std::string &);

  std::string get(const int);
  bool loadJsonFile();
};
}  // namespace mg400_interface
