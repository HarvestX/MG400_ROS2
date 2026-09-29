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

#ifndef MG400_INTERFACE__EXTERNAL_FORCE_ESTIMATOR_HPP_
#define MG400_INTERFACE__EXTERNAL_FORCE_ESTIMATOR_HPP_

#include <array>
#include <cstddef>
#include <memory>
#include <utility>

#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "mg400_interface/tcp_interface/realtime_data.hpp"

namespace mg400_interface
{

class ExternalForceEstimator
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ExternalForceEstimator)
  RCLCPP_UNIQUE_PTR_DEFINITIONS(ExternalForceEstimator)

  static constexpr std::size_t kJointDim = 4;
  static constexpr std::size_t kFeatureDim = 11;

  // ---------------------------------------------------------------------------
  // Default parameter constants
  // ---------------------------------------------------------------------------

  /// Subtract scaled i_target from i_actual before current-to-torque conversion.
  static constexpr bool kDefaultUseTargetCurrentCompensation = true;

  /// Per-joint scale factors for i_target compensation.
  static constexpr std::array<double, kJointDim> kDefaultTargetCurrentScale = {
    1.0, 1.0, 1.0, 1.0};

  /// Per-joint current-to-torque conversion gains [Nm/A].
  static constexpr std::array<double, kJointDim> kDefaultTorqueConstants = {
    3.407905, 3.304349, 5.653824, 0.747993};

  /// Per-joint sign for q / qd conversion from robot degree units to rad / rad/s.
  static constexpr std::array<double, kJointDim> kDefaultJointSigns = {
    1.0, 1.0, 1.0, 1.0};

  /// Per-joint sign applied after current-to-torque conversion.
  static constexpr std::array<double, kJointDim> kDefaultJointTorqueSigns = {
    -1.0, 1.0, 1.0, 1.0};

  /// Static per-joint current offset subtracted from i_actual [A].
  static constexpr std::array<double, kJointDim> kDefaultJointCurrentBias = {
    0.02280366480262477, 0.10339346459323494,
    0.03570217326469629, -0.040302602092404216};

  /// Startup auto-bias sample count.  0 disables the calibration phase.
  static constexpr int kDefaultAutoBiasSampleCount = 0;

  /// Flip estimated origin-axis Fy before rotating into the flange frame.
  static constexpr bool kDefaultInvertForceYAxis = false;

  /// EMA smoothing factor for the estimated wrench (0 = heavily smoothed, 1 = raw).
  static constexpr double kDefaultWrenchFilterAlpha = 0.2;

  /// Finite-difference step size for the numerical Jacobian [rad].
  static constexpr double kDefaultJacobianDiffStep = 1.0e-5;

  /// Minimum singular value below which a Jacobian singularity warning is emitted.
  static constexpr double kDefaultMinSingularValue = 1.0e-4;

  /// Damped least-squares regularization lambda for the wrench solve.
  static constexpr double kDefaultDampingLambda = 1.0e-4;

  /// Per-joint soft deadband applied to measured torque after compensation [Nm].
  static constexpr std::array<double, kJointDim> kDefaultJointTorqueDeadband = {
    0.02, 0.02, 0.02, 0.01};

  /// Enable posture torque compensation using the pre-trained linear model.
  static constexpr bool kDefaultUsePostureCompensation = true;

  /// Posture model coefficients — kJointDim * kFeatureDim values in joint-major order.
  // clang-format off
  static constexpr std::array<double, kJointDim * kFeatureDim> kDefaultPostureCoefficients = {
    // Joint 1
    -1.1853875498107305, -0.44341274311649853, -1.0658279008481168,
    3.204027132251769, -1.3173846040787363, -1.8617091009075417,
    3.0268825349859654, -0.0112976272261891, 0.019260534771652793,
    3.410266010277075, 0.07399515404584066,
    // Joint 2
    -1.829870074601965, -0.028909285724239696, 1.3808278492625254,
    -5.418219001353804, 2.5519957985367427, 4.99098382658275,
    -2.5086094305804916, -0.09000690343203943, -0.060179073315142606,
    -5.417735694239037, 0.7806433929036944,
    // Joint 3
    0.48666647828460385, -0.08022589114980869, 0.48406319239267276,
    -6.749308894232707, 6.002852415810434, 5.99898694737706,
    -6.065468022453522, 0.09180239094917708, -0.6014370182808327,
    -9.847093299185461, 0.2055410043637128,
    // Joint 4
    0.6153182679113095, 0.000560163217054054, -0.055799026102198314,
    -0.5191406436354344, -0.11958056283835503, 0.18687030872410393,
    -0.2907749910359217, -0.03817899419332558, 0.009713596243691822,
    -0.3228650806839767, -0.04411703620781324};
  // clang-format on

  /// Enable viscous + Coulomb friction torque compensation.
  static constexpr bool kDefaultUseFrictionCompensation = true;

  /// Per-joint Coulomb friction coefficient [Nm].
  static constexpr std::array<double, kJointDim> kDefaultFrictionCoulombCoeff = {
    0.0, 0.0, 0.04, 0.0};

  /// Per-joint viscous friction coefficient [Nm / (rad/s)].
  static constexpr std::array<double, kJointDim> kDefaultFrictionViscousCoeff = {
    0.0, 0.0, 0.0, 0.0};

  /// Smoothing scale for the tanh Coulomb friction approximation [rad/s].
  static constexpr double kDefaultFrictionVelocityEpsilon = 0.01;

  /// Minimum singular value threshold; the previous wrench is reused when below this.
  static constexpr double kDefaultDeterministicMinSingularValue = 5.0e-4;

  /// Maximum output force vector norm [N].
  static constexpr double kDefaultDeterministicMaxForceNorm = 200.0;

  /// Maximum output torque-z magnitude [Nm].
  static constexpr double kDefaultDeterministicMaxTorqueNorm = 20.0;

  /// Maximum force slew rate [N/s].
  static constexpr double kDefaultDeterministicMaxForceRate = 250.0;

  /// Maximum torque-z slew rate [Nm/s].
  static constexpr double kDefaultDeterministicMaxTorqueRate = 50.0;

  /// Force-vector output deadband magnitude [N].
  static constexpr double kDefaultOutputForceDeadband = 1.0;

  /// Torque-z output deadband magnitude [Nm].
  static constexpr double kDefaultOutputTorqueDeadband = 0.1;

  /// Nominal update rate used to scale rate-of-change limits [Hz].
  static constexpr double kDefaultUpdateRateHz = 125.0;

  // ---------------------------------------------------------------------------
  // Configuration structure
  // ---------------------------------------------------------------------------

  /// All tunable parameters for ExternalForceEstimator.
  /// Default values match kDefault* constants above.
  struct Config
  {
    bool use_target_current_compensation = kDefaultUseTargetCurrentCompensation;
    std::array<double, kJointDim> target_current_scale = kDefaultTargetCurrentScale;
    std::array<double, kJointDim> torque_constants = kDefaultTorqueConstants;
    std::array<double, kJointDim> joint_signs = kDefaultJointSigns;
    std::array<double, kJointDim> joint_torque_signs = kDefaultJointTorqueSigns;
    std::array<double, kJointDim> joint_current_bias = kDefaultJointCurrentBias;
    int auto_bias_sample_count = kDefaultAutoBiasSampleCount;
    bool invert_force_y_axis = kDefaultInvertForceYAxis;
    double wrench_filter_alpha = kDefaultWrenchFilterAlpha;
    double jacobian_diff_step = kDefaultJacobianDiffStep;
    double min_singular_value = kDefaultMinSingularValue;
    double damping_lambda = kDefaultDampingLambda;
    std::array<double, kJointDim> joint_torque_deadband = kDefaultJointTorqueDeadband;
    bool use_posture_compensation = kDefaultUsePostureCompensation;
    std::array<double, kJointDim * kFeatureDim> posture_coefficients =
      kDefaultPostureCoefficients;
    bool use_friction_compensation = kDefaultUseFrictionCompensation;
    std::array<double, kJointDim> friction_coulomb_coeff = kDefaultFrictionCoulombCoeff;
    std::array<double, kJointDim> friction_viscous_coeff = kDefaultFrictionViscousCoeff;
    double friction_velocity_epsilon = kDefaultFrictionVelocityEpsilon;
    double deterministic_min_singular_value = kDefaultDeterministicMinSingularValue;
    double deterministic_max_force_norm = kDefaultDeterministicMaxForceNorm;
    double deterministic_max_torque_norm = kDefaultDeterministicMaxTorqueNorm;
    double deterministic_max_force_rate = kDefaultDeterministicMaxForceRate;
    double deterministic_max_torque_rate = kDefaultDeterministicMaxTorqueRate;
    double output_force_deadband = kDefaultOutputForceDeadband;
    double output_torque_deadband = kDefaultOutputTorqueDeadband;
    double update_rate_hz = kDefaultUpdateRateHz;
  };

  // ---------------------------------------------------------------------------
  // Public interface
  // ---------------------------------------------------------------------------

  /// Construct with default configuration.
  ExternalForceEstimator();
  /// Construct with custom configuration.
  explicit ExternalForceEstimator(const Config & config);

  /// Process one realtime data sample and update the estimated wrench.
  ///
  /// Only samples with nonzero EnableStatus are used, including for auto-bias.
  /// RobotMode and alarm status do not gate estimation.
  /// Returns false when disabled or while auto-bias calibration is ongoing;
  /// getEstimatedExternalForce() is a current estimate only when this returns true.
  bool update(const RealTimeData & data);

  /// Most recently computed estimated external force / torque as a six-element array
  /// [Fx, Fy, Fz, 0, 0, Tz] in N / Nm, expressed in mg400_end_effector_flange
  /// with torque about the flange origin. Tx and Ty are unestimated, stored as zero.
  /// Origin-axis EMA filtering, flange-frame rotation, and output deadband are applied.
  const std::array<double, 6> & getEstimatedExternalForce() const;

  /// Returns true once the auto-bias calibration phase has completed.
  bool isReady() const;

  static rclcpp::Logger getLogger();

private:
  void updateAutoBias(const RealTimeData & data);

  Eigen::Vector4d toJointAngles(const RealTimeData & data) const;
  Eigen::Vector4d toJointTorque(const RealTimeData & data) const;
  double getModelCurrent(const RealTimeData & data, std::size_t index) const;
  Eigen::Vector4d getJointVelocityRad(const RealTimeData & data) const;

  Eigen::Vector4d applyJointTorqueDeadband(const Eigen::Vector4d & torque) const;

  Eigen::Vector4d estimateDeterministicExternalJointTorque(
    const RealTimeData & data,
    const Eigen::Vector4d & joints,
    const Eigen::Vector4d & measured_joint_torque) const;

  Eigen::Vector4d estimateDeterministicPostureTorque(const Eigen::Vector4d & joints) const;
  Eigen::VectorXd makePoseFeatures(const Eigen::Vector4d & joints) const;

  Eigen::Matrix4d buildTaskJacobian(const Eigen::Vector4d & joints) const;

  std::pair<Eigen::Vector4d, double> solveWrenchWithMinSingularValue(
    const Eigen::Matrix4d & jacobian,
    const Eigen::Vector4d & torque);

  Eigen::Vector4d applyDeterministicWrenchGuards(
    const Eigen::Vector4d & wrench,
    double min_singular_value);

  Eigen::Vector4d toOutputFrameWrench(
    const Eigen::Vector4d & wrench, double flange_yaw) const;
  Eigen::Vector4d applyOutputDeadband(const Eigen::Vector4d & wrench) const;

  Config config_;
  double period_sec_;

  /// Pre-computed per-joint posture compensation weight vectors.
  std::array<Eigen::VectorXd, kJointDim> posture_weights_;

  // Auto-bias calibration state.
  std::array<double, kJointDim> auto_bias_;
  std::array<double, kJointDim> auto_bias_accumulator_;
  int auto_bias_samples_collected_;

  // Wrench estimation state in the fixed robot-origin axes, at the flange point.
  Eigen::Vector4d prev_wrench_;
  bool has_prev_wrench_;
  Eigen::Vector4d filtered_wrench_;
  bool has_filtered_wrench_;

  /// Last result returned by getEstimatedExternalForce().
  std::array<double, 6> last_external_force_;

  /// Steady clock used for rate-throttled log messages.
  rclcpp::Clock steady_clock_;
};

}  // namespace mg400_interface

#endif  // MG400_INTERFACE__EXTERNAL_FORCE_ESTIMATOR_HPP_
