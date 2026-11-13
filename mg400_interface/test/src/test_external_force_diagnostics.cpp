// Copyright 2026 HarvestX Inc.
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

#include <gtest/gtest.h>

#include <cmath>

#include "mg400_interface/external_force_estimator.hpp"

using mg400_interface::ExternalForceEstimator;
using mg400_interface::RealTimeData;

static ExternalForceEstimator::Config uncompensatedConfig()
{
  ExternalForceEstimator::Config config;
  config.auto_bias_sample_count = 0;
  config.joint_current_bias.fill(0.0);
  config.use_posture_compensation = false;
  config.use_friction_compensation = false;
  config.use_target_current_compensation = false;
  return config;
}

TEST(ExternalForceDiagnostics, ZeroCurrentProducesZeroWrench)
{
  ExternalForceEstimator estimator(uncompensatedConfig());
  RealTimeData data{};
  data.enable_status = 1;
  data.q_actual[1] = 30.0;
  data.q_actual[2] = 45.0;
  ASSERT_TRUE(estimator.update(data));
  for (const auto value : estimator.getEstimatedExternalForce()) {
    EXPECT_DOUBLE_EQ(value, 0.0);
  }
}

TEST(ExternalForceDiagnostics, DisabledSamplesDoNotReplaceLastEstimate)
{
  ExternalForceEstimator estimator(uncompensatedConfig());
  RealTimeData data{};
  data.enable_status = 1;
  data.q_actual[1] = 30.0;
  data.q_actual[2] = 45.0;
  ASSERT_TRUE(estimator.update(data));
  const auto last = estimator.getEstimatedExternalForce();
  data.enable_status = 0;
  data.i_actual[0] = 100.0;
  EXPECT_FALSE(estimator.update(data));
  EXPECT_EQ(estimator.getEstimatedExternalForce(), last);
}

TEST(ExternalForceDiagnostics, DisabledSamplesDoNotAdvanceCalibration)
{
  auto config = uncompensatedConfig();
  config.auto_bias_sample_count = 2;
  ExternalForceEstimator estimator(config);
  RealTimeData data{};
  EXPECT_FALSE(estimator.update(data));
  EXPECT_FALSE(estimator.isReady());
  data.enable_status = 1;
  EXPECT_FALSE(estimator.update(data));
  EXPECT_FALSE(estimator.isReady());
  EXPECT_TRUE(estimator.update(data));
  EXPECT_TRUE(estimator.isReady());
}
