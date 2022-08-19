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

#include "mg400_interface/commander/response_parser.hpp"

namespace mg400_interface
{
std::array<std::vector<int>, 6> ResponseParser::parseErrorMessage(
  const std::string & response)
{
  std::array<std::vector<int>, 6> ret = {};
  // copy to mutable variable
  std::string s = response;

  s.erase(std::remove_if(s.begin(), s.end(), ::isspace), s.end());

  std::smatch m;
  if (!std::regex_search(s, m, std::regex(R"(\{\[(.*?)\]\})"))) {
    return ret;
  }


  s = m[1].str();
  auto iter = s.cbegin();
  auto end = s.cend();
  const std::regex re(R"(\[(.*?)\])");

  int i = 0;
  while (std::regex_search(iter, end, m, re)) {
    iter = m[0].second;

    std::stringstream ss(m[1].str());
    std::vector<int> tmp;
    while (ss.good()) {
      std::string substr;
      std::getline(ss, substr, ',');
      try {
        tmp.emplace_back(std::stoi(substr));
      } catch (std::invalid_argument &) {
      }
    }
    ret.at(i) = tmp;
    i++;
  }
  return ret;
}

std::array<std::vector<double>, 6> ResponseParser::parseAngleorPose(
  const std::string & response)
{
  std::array<std::vector<double>, 6> ret = {};
  // copy to mutable variable
  std::string s = response;

  s.erase(std::remove_if(s.begin(), s.end(), ::isspace), s.end());

  std::smatch m;
  if (!std::regex_search(s, m, std::regex(R"(\{\[(.*?)\]\})"))) {
    return ret;
  }


  s = m[1].str();
  auto iter = s.cbegin();
  auto end = s.cend();
  const std::regex re(R"(\[(.*?)\])");

  int i = 0;
  while (std::regex_search(iter, end, m, re)) {
    iter = m[0].second;

    std::stringstream ss(m[1].str());
    std::vector<double> tmp;
    while (ss.good()) {
      std::string substr;
      std::getline(ss, substr, ',');
      try {
        tmp.emplace_back(std::stof(substr));
      } catch (std::invalid_argument &) {
      }
    }
    ret.at(i) = tmp;
    i++;
  }
  return ret;
}
}  // namespace mg400_interface
