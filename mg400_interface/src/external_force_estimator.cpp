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

#include "mg400_interface/external_force_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "mg400_common/kinematics.hpp"
#include "mg400_interface/command_utils.hpp"

namespace mg400_interface
{

ExternalForceEstimator::ExternalForceEstimator()
: ExternalForceEstimator(Config{}) {}

ExternalForceEstimator::ExternalForceEstimator(const Config & config)
: config_(config),
  period_sec_(1.0 / std::max(1.0, config.update_rate_hz)),
  auto_bias_({0.0, 0.0, 0.0, 0.0}),
  auto_bias_accumulator_({0.0, 0.0, 0.0, 0.0}),
  auto_bias_samples_collected_(0),
  prev_wrench_(Eigen::Vector4d::Zero()),
  has_prev_wrench_(false),
  filtered_wrench_(Eigen::Vector4d::Zero()),
  has_filtered_wrench_(false),
  last_external_force_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  steady_clock_(RCL_STEADY_TIME)
{
  for (std::size_t joint = 0; joint < kJointDim; ++joint) {
    this->posture_weights_[joint] = Eigen::VectorXd::Zero(kFeatureDim);
    for (std::size_t k = 0; k < kFeatureDim; ++k) {
      this->posture_weights_[joint](static_cast<Eigen::Index>(k)) =
        this->config_.posture_coefficients.at(joint * kFeatureDim + k);
    }
  }
}

rclcpp::Logger ExternalForceEstimator::getLogger()
{
  return rclcpp::get_logger("ExternalForceEstimator");
}

bool ExternalForceEstimator::isReady() const
{
  return this->config_.auto_bias_sample_count <= 0 ||
         this->auto_bias_samples_collected_ >= this->config_.auto_bias_sample_count;
}

const std::array<double, 6> & ExternalForceEstimator::getEstimatedExternalForce() const
{
  return this->last_external_force_;
}

bool ExternalForceEstimator::update(const RealTimeData & data)
{
  this->updateAutoBias(data);

  if (!this->isReady()) {
    RCLCPP_INFO_THROTTLE(
      getLogger(), this->steady_clock_, 2000,
      "Calibrating current bias (%d/%d)",
      this->auto_bias_samples_collected_, this->config_.auto_bias_sample_count);
    return false;
  }

  const Eigen::Vector4d joints = this->toJointAngles(data);
  const Eigen::Vector4d measured_joint_torque = this->toJointTorque(data);
  const Eigen::Matrix4d jacobian = this->buildTaskJacobian(joints);

  const Eigen::Vector4d ext_joint_torque =
    this->estimateDeterministicExternalJointTorque(data, joints, measured_joint_torque);
  const auto [estimated_wrench, min_sv] =
    this->solveWrenchWithMinSingularValue(jacobian, ext_joint_torque);

  const Eigen::Vector4d guarded_wrench =
    this->applyDeterministicWrenchGuards(estimated_wrench, min_sv);

  // Apply EMA filter.
  if (!this->has_filtered_wrench_) {
    this->filtered_wrench_ = guarded_wrench;
    this->has_filtered_wrench_ = true;
  } else {
    this->filtered_wrench_ =
      this->config_.wrench_filter_alpha * guarded_wrench +
      (1.0 - this->config_.wrench_filter_alpha) * this->filtered_wrench_;
  }

  Eigen::Vector4d output_wrench = this->toOutputFrameWrench(this->filtered_wrench_);
  output_wrench = this->applyOutputDeadband(output_wrench);

  // Store as TCP_Force-compatible array: [Fx, Fy, Fz, 0, 0, Tz].
  this->last_external_force_ = {
    output_wrench(0), output_wrench(1), output_wrench(2),
    0.0, 0.0, output_wrench(3)};

  return true;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void ExternalForceEstimator::updateAutoBias(const RealTimeData & data)
{
  if (this->config_.auto_bias_sample_count <= 0 ||
    this->auto_bias_samples_collected_ >= this->config_.auto_bias_sample_count)
  {
    return;
  }

  for (std::size_t i = 0; i < this->auto_bias_accumulator_.size(); ++i) {
    this->auto_bias_accumulator_[i] +=
      data.i_actual[i] - this->getModelCurrent(data, i);
  }
  ++this->auto_bias_samples_collected_;

  if (this->auto_bias_samples_collected_ == this->config_.auto_bias_sample_count) {
    for (std::size_t i = 0; i < this->auto_bias_.size(); ++i) {
      this->auto_bias_[i] = this->auto_bias_accumulator_[i] /
        static_cast<double>(this->config_.auto_bias_sample_count);
    }
    RCLCPP_INFO(
      getLogger(),
      "Auto-bias calibration completed with %d samples. "
      "auto_bias: [%.6f, %.6f, %.6f, %.6f]",
      this->config_.auto_bias_sample_count,
      this->auto_bias_[0], this->auto_bias_[1],
      this->auto_bias_[2], this->auto_bias_[3]);
  }
}

Eigen::Vector4d ExternalForceEstimator::toJointAngles(const RealTimeData & data) const
{
  Eigen::Vector4d joints;
  for (std::size_t i = 0; i < kJointDim; ++i) {
    joints(static_cast<Eigen::Index>(i)) =
      this->config_.joint_signs[i] * data.q_actual[i] * TO_RADIAN;
  }
  return joints;
}

Eigen::Vector4d ExternalForceEstimator::toJointTorque(const RealTimeData & data) const
{
  Eigen::Vector4d torque;
  for (std::size_t i = 0; i < kJointDim; ++i) {
    const double effective_current =
      data.i_actual[i] - this->getModelCurrent(data, i) -
      this->config_.joint_current_bias[i] - this->auto_bias_[i];
    torque(static_cast<Eigen::Index>(i)) =
      this->config_.joint_torque_signs[i] *
      this->config_.torque_constants[i] * effective_current;
  }
  return torque;
}

double ExternalForceEstimator::getModelCurrent(
  const RealTimeData & data, const std::size_t index) const
{
  if (!this->config_.use_target_current_compensation) {
    return 0.0;
  }
  return this->config_.target_current_scale[index] * data.i_target[index];
}

Eigen::Vector4d ExternalForceEstimator::getJointVelocityRad(const RealTimeData & data) const
{
  Eigen::Vector4d qd;
  for (std::size_t i = 0; i < kJointDim; ++i) {
    qd(static_cast<Eigen::Index>(i)) =
      this->config_.joint_signs[i] * data.qd_actual[i] * TO_RADIAN;
  }
  return qd;
}

Eigen::Vector4d ExternalForceEstimator::applyJointTorqueDeadband(
  const Eigen::Vector4d & torque) const
{
  Eigen::Vector4d output = torque;
  for (std::size_t i = 0; i < kJointDim; ++i) {
    const double v = output(static_cast<Eigen::Index>(i));
    const double db = this->config_.joint_torque_deadband[i];
    if (std::abs(v) <= db) {
      output(static_cast<Eigen::Index>(i)) = 0.0;
    } else {
      output(static_cast<Eigen::Index>(i)) = std::copysign(std::abs(v) - db, v);
    }
  }
  return output;
}

Eigen::Vector4d ExternalForceEstimator::estimateDeterministicExternalJointTorque(
  const RealTimeData & data,
  const Eigen::Vector4d & joints,
  const Eigen::Vector4d & measured_joint_torque) const
{
  Eigen::Vector4d external_joint_torque = measured_joint_torque;

  if (this->config_.use_posture_compensation) {
    external_joint_torque -= this->estimateDeterministicPostureTorque(joints);
  }

  if (this->config_.use_friction_compensation) {
    const Eigen::Vector4d qd = this->getJointVelocityRad(data);
    for (std::size_t i = 0; i < kJointDim; ++i) {
      const double qd_i = qd(static_cast<Eigen::Index>(i));
      const double viscous = this->config_.friction_viscous_coeff[i] * qd_i;
      const double coulomb =
        this->config_.friction_coulomb_coeff[i] *
        std::tanh(qd_i / this->config_.friction_velocity_epsilon);
      external_joint_torque(static_cast<Eigen::Index>(i)) -= (viscous + coulomb);
    }
  }

  return this->applyJointTorqueDeadband(external_joint_torque);
}

Eigen::Vector4d ExternalForceEstimator::estimateDeterministicPostureTorque(
  const Eigen::Vector4d & joints) const
{
  Eigen::Vector4d posture_torque = Eigen::Vector4d::Zero();
  const Eigen::VectorXd phi = this->makePoseFeatures(joints);
  for (std::size_t i = 0; i < kJointDim; ++i) {
    posture_torque(static_cast<Eigen::Index>(i)) =
      this->posture_weights_[i].dot(phi);
  }
  return posture_torque;
}

Eigen::VectorXd ExternalForceEstimator::makePoseFeatures(const Eigen::Vector4d & joints) const
{
  const double q1 = joints(0);
  const double q2 = joints(1);
  const double q3 = joints(2);
  const double q4 = joints(3);

  Eigen::VectorXd phi(kFeatureDim);
  phi <<
    1.0,
    std::sin(q1), std::cos(q1),
    std::sin(q2), std::cos(q2),
    std::sin(q3), std::cos(q3),
    std::sin(q4), std::cos(q4),
    std::sin(q3 - q2), std::cos(q3 - q2);
  return phi;
}

Eigen::Matrix4d ExternalForceEstimator::buildTaskJacobian(const Eigen::Vector4d & joints) const
{
  Eigen::Matrix4d jacobian;
  for (std::size_t i = 0; i < kJointDim; ++i) {
    Eigen::Vector4d q_plus = joints;
    Eigen::Vector4d q_minus = joints;
    q_plus(static_cast<Eigen::Index>(i)) += this->config_.jacobian_diff_step;
    q_minus(static_cast<Eigen::Index>(i)) -= this->config_.jacobian_diff_step;

    const Eigen::Vector4d pose_plus =
      mg400_common::kinematics::fk_for_flange_from_origin(q_plus);
    const Eigen::Vector4d pose_minus =
      mg400_common::kinematics::fk_for_flange_from_origin(q_minus);

    Eigen::Vector4d delta = pose_plus - pose_minus;
    delta(3) = mg400_common::kinematics::normalize_angle(pose_plus(3) - pose_minus(3));

    jacobian.col(static_cast<Eigen::Index>(i)) =
      delta / (2.0 * this->config_.jacobian_diff_step);
  }
  return jacobian;
}

std::pair<Eigen::Vector4d, double>
ExternalForceEstimator::solveWrenchWithMinSingularValue(
  const Eigen::Matrix4d & jacobian,
  const Eigen::Vector4d & torque)
{
  const Eigen::Matrix4d map = jacobian.transpose();
  const Eigen::JacobiSVD<Eigen::Matrix4d> svd(
    map, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto singular_values = svd.singularValues();
  const double min_singular_value = singular_values(singular_values.size() - 1);

  if (min_singular_value < this->config_.min_singular_value) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), this->steady_clock_, 2000,
      "Jacobian near-singular. min singular value = %.6e", min_singular_value);
  }

  Eigen::Vector4d damped_inverse_sigma = Eigen::Vector4d::Zero();
  for (Eigen::Index i = 0; i < singular_values.size(); ++i) {
    const double sigma = singular_values(i);
    const double denom =
      sigma * sigma + this->config_.damping_lambda * this->config_.damping_lambda;
    if (denom <= std::numeric_limits<double>::epsilon()) {
      continue;
    }
    damped_inverse_sigma(i) = sigma / denom;
  }

  const Eigen::Vector4d wrench =
    svd.matrixV() * damped_inverse_sigma.asDiagonal() *
    svd.matrixU().transpose() * torque;
  return {wrench, min_singular_value};
}

Eigen::Vector4d ExternalForceEstimator::applyDeterministicWrenchGuards(
  const Eigen::Vector4d & wrench,
  const double min_singular_value)
{
  Eigen::Vector4d output = wrench;

  // Singularity guard: reuse previous wrench when near singular.
  if (min_singular_value < this->config_.deterministic_min_singular_value) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), this->steady_clock_, 2000,
      "Singularity guard: min singular value = %.6e. Reusing previous wrench.",
      min_singular_value);
    if (this->has_prev_wrench_) {
      output = this->prev_wrench_;
    } else {
      output = Eigen::Vector4d::Zero();
    }
  }

  // Force norm clamp.
  const double force_norm = output.head<3>().norm();
  if (this->config_.deterministic_max_force_norm > 0.0 &&
    force_norm > this->config_.deterministic_max_force_norm)
  {
    output.head<3>() *= this->config_.deterministic_max_force_norm / force_norm;
  }

  // Torque-z clamp.
  if (this->config_.deterministic_max_torque_norm > 0.0) {
    output(3) = std::clamp(
      output(3),
      -this->config_.deterministic_max_torque_norm,
      this->config_.deterministic_max_torque_norm);
  }

  // Rate-of-change limiting.
  if (this->has_prev_wrench_) {
    const double max_force_delta =
      this->config_.deterministic_max_force_rate * this->period_sec_;
    if (max_force_delta > 0.0) {
      Eigen::Vector3d force_delta = output.head<3>() - this->prev_wrench_.head<3>();
      const double delta_norm = force_delta.norm();
      if (delta_norm > max_force_delta) {
        force_delta *= max_force_delta / delta_norm;
        output.head<3>() = this->prev_wrench_.head<3>() + force_delta;
      }
    }

    const double max_torque_delta =
      this->config_.deterministic_max_torque_rate * this->period_sec_;
    if (max_torque_delta > 0.0) {
      const double torque_delta = output(3) - this->prev_wrench_(3);
      if (std::abs(torque_delta) > max_torque_delta) {
        output(3) =
          this->prev_wrench_(3) + std::copysign(max_torque_delta, torque_delta);
      }
    }
  }

  this->prev_wrench_ = output;
  this->has_prev_wrench_ = true;
  return output;
}

Eigen::Vector4d ExternalForceEstimator::toOutputFrameWrench(
  const Eigen::Vector4d & wrench) const
{
  Eigen::Vector4d output = wrench;
  if (this->config_.invert_force_y_axis) {
    output(1) = -output(1);
  }
  return output;
}

Eigen::Vector4d ExternalForceEstimator::applyOutputDeadband(
  const Eigen::Vector4d & wrench) const
{
  Eigen::Vector4d output = wrench;

  if (this->config_.output_force_deadband > 0.0) {
    const double f_norm = output.head<3>().norm();
    if (f_norm <= this->config_.output_force_deadband) {
      output.head<3>().setZero();
    } else {
      output.head<3>() *=
        (f_norm - this->config_.output_force_deadband) / f_norm;
    }
  }

  if (this->config_.output_torque_deadband > 0.0) {
    const double tz = output(3);
    if (std::abs(tz) <= this->config_.output_torque_deadband) {
      output(3) = 0.0;
    } else {
      output(3) = tz - std::copysign(this->config_.output_torque_deadband, tz);
    }
  }

  return output;
}

}  // namespace mg400_interface
