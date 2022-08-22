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

int ResponseParser::parseRobotModeorDI(const std::string & response)
{
  std::regex re(R"(\{.*?\})");
  std::string s = response, c;
  std::smatch m;

  if (!std::regex_search(s, m, re)) {
    return 0;
  }
  c = m.str();
  c = c.substr(1, c.length() - 2);
  return atoi(c.c_str());
}

std::vector<int> ResponseParser::parsearray(const std::string & response, const int count)
{
  std::vector<int> ret(count + 1);
  std::regex re1(R"(\{.*?\})"), re2(R"(\.*?\,)");
  std::string s = response, s1, s2;
  std::smatch m;

  std::regex_search(s, m, re2);
  s1 = m.str();
  s1 = s1.substr(0, s1.length() - 2);
  ret[0] = atoi(s1.c_str());

  if (!std::regex_search(s, m, re1)) {
    return std::vector<int>(0);
  }
  s1 = m.str();
  s1 = s1.substr(1, s1.length() - 2);

  for (int i = 1; i <= count; i++) {
    if (!std::regex_search(s1, m, re2)) {
      s2 = m.str();
      ret[i] = atoi(s2.c_str());
      continue;
    }
    s2 = m.str();
    s2 = s2.substr(0, s2.length() - 2);
    ret[i] = atoi(s2.c_str());
  }

  return ret;
}

int ResponseParser::parseOnlyErrorID(const std::string & response)
{
  std::regex re(R"(\.*?\,)");
  std::string s = response;
  std::smatch m;

  std::regex_search(s, m, re);
  s = m.str();
  s = s.substr(0, s.length() - 2);

  return atoi(s.c_str());
}
}  // namespace mg400_interface
