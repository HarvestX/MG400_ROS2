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


#include "mg400_interface/error_msg_generator.hpp"

namespace mg400_interface
{
ErrorMsgGenerator::ErrorMsgGenerator(
  const std::string & target_filename
)
: logger_(rclcpp::get_logger("ErrorMsgGenerator")),
  filename_(
    ament_index_cpp::get_package_share_directory("mg400_interface") +
    "/resources/" + target_filename)
{
  if (target_filename.empty()) {
    RCLCPP_ERROR(this->logger_, "Json file name not given");
  }

  RCLCPP_INFO(
    this->logger_, "Target file: %s",
    this->filename_.c_str());

  this->loadJsonFile();
}

std::string ErrorMsgGenerator::get(const int id)
{
  return this->error_map_.at(id);
}


bool ErrorMsgGenerator::loadJsonFile()
{
  std::ifstream json_file(this->filename_);
  nlohmann::json json_data;
  json_file >> json_data;

  for (auto data : json_data) {
    RCLCPP_DEBUG(
      this->logger_,
      "ID %d, Message: %s",
      data["id"].get<int>(),
      data["en"]["description"].get<std::string>().c_str());

    this->error_map_.emplace(
      data["id"].get<int>(),
      data["en"]["description"].get<std::string>());
  }

  return true;
}


}  // namespace mg400_interface
