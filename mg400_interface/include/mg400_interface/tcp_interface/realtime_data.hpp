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

#include <inttypes.h>

namespace mg400_interface
{
#pragma pack(push, 1)
struct RealTimeData
{
  uint16_t len;                         // 0000 ~ 0001 //
  uint16_t reserved1[3];                // 0002 ~ 0007 //
  uint64_t digital_input_bits;          // 0008 ~ 0015 //
  uint64_t digital_outputs;             // 0016 ~ 0023 //
  uint64_t robot_mode;                  // 0024 ~ 0031 //
  uint64_t notused1;                    // 0032 ~ 0039 //
  uint64_t reserved2;                   // 0040 ~ 0047 //
  // 0048 ~ 0095 //
  uint64_t test_value;                  // 0048 ~ 0055 //
  double reserved3;                     // 0056 ~ 0063 //
  double speed_scaling;                 // 0064 ~ 0071 //
  double notused2;                      // 0072 ~ 0079 //
  double notused3;                      // 0080 ~ 0087 //
  double notused4;                      // 0088 ~ 0095 //
  // 0096 ~ 0143 //
  double notused5;                      // 0096 ~ 0103 //
  double reserved4;                     // 0104 ~ 0111 //
  double reserved5;                     // 0112 ~ 0119 //
  double notused6[3];                   // 0120 ~ 0143 //
  // 0144 ~ 0191 //
  double notused7[3];                   // 0144 ~ 0167 //
  double notused8[3];                   // 0168 ~ 0191 //
  // 0192 ~ 1439 //
  double q_target[6];                   // 0192 ~ 0239 //
  double qd_target[6];                  // 0240 ~ 0287 //
  double qdd_target[6];                 // 0288 ~ 0335 //
  double i_target[6];                   // 0336 ~ 0383 //
  double m_target[6];                   // 0384 ~ 0431 //
  double q_actual[6];                   // 0432 ~ 0479 //
  double qd_actual[6];                  // 0480 ~ 0527 //
  double i_actual[6];                   // 0528 ~ 0575 //
  double actual_i_TCP_force[6];         // 0576 ~ 0623 //
  double tool_vector_actual[6];         // 0624 ~ 0671 //
  double TCP_speed_actual[6];           // 0672 ~ 0719 //
  double TCP_force[6];                  // 0720 ~ 0767 //
  double tool_vector_target[6];         // 0768 ~ 0815 //
  double TCP_speed_target[6];           // 0816 ~ 0863 //
  double notused9[6];                   // 0864 ~ 0911 //
  double notused10[6];                  // 0912 ~ 0959 //
  double notused11[6];                  // 0960 ~ 1007 //
  uint8_t notused12[4];                 // 1008 ~ 1011 //
  uint8_t notused13[26];                // 1012 ~ 1037 //
  uint8_t reserved6[82];                // 1038 ~ 1119 //
  double notused14[6];                  // 1120 ~ 1167 //
  double load;                          // 1168 ~ 1175 //
  double center_x;                      // 1176 ~ 1183 //
  double center_y;                      // 1184 ~ 1191 //
  double centor_z;                      // 1192 ~ 1199 //
  double notused15[6];                  // 1200 ~ 1247 //
  double notused16[6];                  // 1248 ~ 1295 //
  double notused17;                     // 1296 ~ 1303 //
  double notused18[6];                  // 1304 ~ 1351 //
  double notused19[4];                  // 1352 ~ 1383 //
  double notused20[4];                  // 1384 ~ 1415 //
  uint8_t reserved7[24];                // 1416 ~ 1440 //
};
#pragma pack(pop)

}  // namespace mg400_interface
