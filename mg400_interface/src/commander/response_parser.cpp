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

#include <sstream>

namespace mg400_interface
{
namespace
{

std::vector<double> takeScaledArray(
  const std::string & response,
  const std::vector<double> & scales)
{
  std::vector<double> ret(scales.size());
  std::string buf;
  size_t idx = 0;
  for (auto s : response) {
    if (s == '{' || s == ' ' || s == '\n' || s == '\t') {
      continue;
    }

    if (s == ',' || s == '}') {
      if (buf.empty()) {
        continue;
      }
      ret.at(idx) = std::stod(buf) * scales.at(idx);
      buf.clear();
      idx++;
      continue;
    }

    buf.push_back(s);
  }

  if (idx != scales.size()) {
    throw std::runtime_error("Unexpected array length: " + std::to_string(idx));
  }
  return ret;
}

}  // namespace

bool ResponseParser::parseResponse(
  const std::string & packet,
  DashboardResponse & response)
{
  // Remove white space
  std::string substr;
  std::string buf;
  int search_mode_counter = 0;
  for (char s : packet) {
    switch (search_mode_counter) {
      case 0:
        if (s == ',') {
          response.error_id = std::stoi(buf);
          buf.clear();
          // go to next search mode
          search_mode_counter++;
          continue;
        }
        break;
      case 1:
        if (buf.back() == '}' && s == ',') {
          response.ret_val = buf;
          buf.clear();
          search_mode_counter++;
          continue;
        }
        break;
      case 2:
        if (s == ';') {
          response.func_name = buf;
          return true;
        }
        break;
      case 3:
      // fall through
      default:
        return false;
    }
    if (s == '\n') {
      continue;
    }
    if (s == ' ') {
      continue;
    }

    buf.push_back(s);
  }
  return false;
}

size_t ResponseParser::countArrayElements(const std::string & response)
{
  size_t count = 0;
  bool in_value = false;
  for (auto s : response) {
    if (s == '{' || s == ' ' || s == '\n' || s == '\t') {
      continue;
    }
    if (s == ',' || s == '}') {
      if (in_value) {
        count++;
        in_value = false;
      }
      continue;
    }
    in_value = true;
  }
  return count;
}

std::array<std::vector<int>, 6> ResponseParser::takeErrorMessage(
  const std::string & response)
{
  std::array<std::vector<int>, 6> ret = {};
  // copy to mutable variable
  std::string s = response;

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

std::vector<double> ResponseParser::takePoseArray(
  const std::string & response)
{
  return takeScaledArray(
    response, {TO_M, TO_M, TO_M, TO_M, TO_M, TO_M});
}

std::vector<double> ResponseParser::takeCartesianPoseArray(
  const std::string & response)
{
  return takeScaledArray(
    response, {TO_M, TO_M, TO_M, TO_RADIAN, TO_RADIAN, TO_RADIAN});
}

std::vector<double> ResponseParser::takeCartesianPoseArray4(
  const std::string & response)
{
  return takeScaledArray(response, {TO_M, TO_M, TO_M, TO_RADIAN});
}

std::vector<double> ResponseParser::takeAngleArray(
  const std::string & response)
{
  return takeScaledArray(
    response, {TO_RADIAN, TO_RADIAN, TO_RADIAN, TO_RADIAN, TO_RADIAN, TO_RADIAN});
}

std::vector<double> ResponseParser::takeAngleArray4(
  const std::string & response)
{
  return takeScaledArray(
    response, {TO_RADIAN, TO_RADIAN, TO_RADIAN, TO_RADIAN});
}

int ResponseParser::takeInt(const std::string & response)
{
  std::string buf;
  for (auto s : response) {
    if (s == '{') {
      continue;
    }
    if (s == '}') {
      break;
    }
    buf.push_back(s);
  }
  return std::stoi(buf);
}
}  // namespace mg400_interface
