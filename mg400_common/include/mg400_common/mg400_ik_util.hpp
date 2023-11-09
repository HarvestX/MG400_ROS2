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

#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>

namespace mg400_common
{

class MG400IKUtil
{
private:
  const double MARGIN = 0.5 * M_PI / 180.0;

  const double J1_MIN = -160 * M_PI / 180.0;
  const double J1_MAX = 160 * M_PI / 180.0;
  const double J2_MIN = -25 * M_PI / 180.0;
  const double J2_MIN_NO_COLLISION = -19 * M_PI / 180.0;
  const double J2_MAX = 85 * M_PI / 180.0;
  const double J3_MIN = -25 * M_PI / 180.0;
  const double J3_MAX = 105 * M_PI / 180.0;
  const double J31_MIN = -60 * M_PI / 180.0;
  const double J31_MAX = 60 * M_PI / 180.0;
  const double J4_MIN = -180 * M_PI / 180.0;
  const double J4_MAX = 180 * M_PI / 180.0;
  const double ROUND_DECIMALS = 6;

  const double LINK1_X = 0.0435;
  const double LINK1_Y = 0.0;
  const double LINK1_Z = 0.0;

  const double LINK2_X = 0.0;
  const double LINK2_Y = 0.0;
  const double LINK2_Z = 0.175;

  const double LINK3_X = 0.175;
  const double LINK3_Y = 0.0;
  const double LINK3_Z = 0.0;

  const double LINK4_X = 0.066;
  const double LINK4_Y = 0.0;
  const double LINK4_Z = -0.0538;

  const Eigen::Vector3d LINK1;
  const Eigen::Vector3d LINK2;
  const Eigen::Vector3d LINK3;
  const Eigen::Vector3d LINK4;

public:
  MG400IKUtil();
  bool InMG400Range(const std::vector<double> &);
  std::vector<double> InverseKinematics(const std::vector<double> &);
  std::vector<double> ToolCoordToBaseCoord(
    const std::vector<double> &,
    const std::vector<double> &);
};

}  // namespace mg400_common
