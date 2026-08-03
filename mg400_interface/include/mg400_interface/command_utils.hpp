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

#ifndef __MG400_INTERFACE_COMMAND_UTILS_HPP__
#define __MG400_INTERFACE_COMMAND_UTILS_HPP__

#include <cctype>
#include <cmath>
#include <string>

namespace mg400_interface
{

using si_mm = double;
using si_m = double;
using si_rad = double;
using si_degree = double;


constexpr const double TO_RADIAN = M_PI / 180.0;
constexpr const double TO_DEGREE = 180.0 / M_PI;

constexpr const double TO_M = 1e-3;
constexpr const double TO_MM = 1e3;

inline si_rad degree2rad(const si_degree val)
{
  return val * TO_RADIAN;
}

inline si_degree rad2degree(const si_rad val)
{
  return val * TO_DEGREE;
}

inline si_mm m2mm(const si_m val)
{
  return val * TO_MM;
}

inline si_m mm2m(const si_mm val)
{
  return val * TO_M;
}

inline std::string normalizeNegativeZero(const std::string & command)
{
  std::string normalized = command;
  std::size_t position = 0;

  while ((position = normalized.find("-0", position)) != std::string::npos) {
    const auto is_identifier_character = [](const char character) {
        const auto value = static_cast<unsigned char>(character);
        return std::isalnum(value) != 0 || character == '_' || character == '.';
      };
    const bool starts_numeric_token =
      position == 0 || !is_identifier_character(normalized[position - 1]);

    std::size_t end = position + 2;
    bool is_zero = true;
    if (end < normalized.size() && normalized[end] == '.') {
      const auto fraction_begin = ++end;
      while (end < normalized.size() && normalized[end] == '0') {
        ++end;
      }
      is_zero = end > fraction_begin;
    }
    const bool ends_numeric_token =
      end == normalized.size() || !is_identifier_character(normalized[end]);

    if (starts_numeric_token && is_zero && ends_numeric_token) {
      normalized.erase(position, 1);
    } else {
      position = end;
    }
  }
  return normalized;
}

}  // namespace mg400_interface
#endif
