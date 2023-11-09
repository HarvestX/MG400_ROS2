// Copyright 2023 HarvestX Inc.
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

#include "mg400_common/mg400_ik_util.hpp"

namespace mg400_common
{
MG400IKUtil::MG400IKUtil()
: LINK1({LINK1_X, LINK1_Y, LINK1_Z}),
  LINK2({LINK2_X, LINK2_Y, LINK2_Z}),
  LINK3({LINK3_X, LINK3_Y, LINK3_Z}),
  LINK4({LINK4_X, LINK4_Y, LINK4_Z})
{
}

bool MG400IKUtil::InMG400Range(const std::vector<double> & angles)
{
  double j1 = angles[0];
  double j2 = angles[1];
  double j3 = angles[2];
  double j4 = angles[3];

  bool is_j1_within_limit = (J1_MIN + MARGIN <= j1 && j1 <= J1_MAX - MARGIN);
  bool is_j2_within_limit =
    (j3 > 0) ? (J2_MIN + MARGIN <= j2 &&
    j2 <= J2_MAX - MARGIN) : (J2_MIN_NO_COLLISION + MARGIN <= j2 && j2 <= J2_MAX - MARGIN);
  bool is_j3_within_limit = (J3_MIN + MARGIN <= j3 && j3 <= J3_MAX - MARGIN);
  bool is_j31_within_limit = (J31_MIN + MARGIN <= (j3 - j2) && (j3 - j2) <= J31_MAX - MARGIN);
  bool is_j4_within_limit = (J4_MIN + MARGIN <= j4 && j4 <= J4_MAX - MARGIN);

  return is_j1_within_limit && is_j2_within_limit &&
         is_j3_within_limit && is_j31_within_limit && is_j4_within_limit;
}

std::vector<double> MG400IKUtil::InverseKinematics(
  const std::vector<double> & tool_vec)
{
  double p_x = tool_vec[0];
  double p_y = tool_vec[1];
  double p_z = tool_vec[2];
  double Rx = tool_vec[3];

  double pp_x = sqrt(pow(p_x, 2) + pow(p_y, 2)) - this->LINK4[0] - this->LINK1[0];
  double pp_z = p_z - this->LINK4[2] - this->LINK1[2];

  double length2 = this->LINK2.norm();
  double length3 = this->LINK3.norm();

  double j_1 = atan2(p_y, p_x);

  double val1 =
    (pow(pp_x, 2) + pow(pp_z, 2) - pow(length2, 2) - pow(length3, 2)) / (2 * length2 * length3);
  if (val1 < -1 || val1 > 1) {
    throw std::runtime_error("Inverse kinematics error");
  }

  double j_3_1 = acos(val1);
  double j_2 = M_PI / 2 - atan2(pp_z, pp_x) - asin(
    length3 / sqrt(pow(pp_x, 2) + pow(pp_z, 2)) * sin(j_3_1));

  double j_3 = j_2 + j_3_1 - M_PI / 2;
  double j_4 = Rx - j_1;

  std::vector<double> angles = {j_1, j_2, j_3, j_4, 0.0, 0.0};
  for (auto & angle : angles) {
    angle = round(angle * pow(10, ROUND_DECIMALS)) / pow(10, ROUND_DECIMALS);
  }
  if (!this->InMG400Range(angles)) {
    throw std::runtime_error("Outside of workspace.");
  }

  return angles;
}

std::vector<double> MG400IKUtil::ToolCoordToBaseCoord(
  const std::vector<double> & vec,
  const std::vector<double> & tool_coord)
{
  std::vector<double> pos(3);
  for (int i = 0; i < 3; ++i) {
    pos[i] = vec[i] - tool_coord[i];
  }
  double ang = vec[3] + tool_coord.back();

  return std::vector<double>{pos[0], pos[1], pos[2], ang, 0.0, 0.0};
}

}  // namespace mg400_common
