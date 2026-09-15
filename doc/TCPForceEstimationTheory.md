# MG400 TCP force estimation: theory and calculation

Integrated from [TCPForceEstimation.md](TCPForceEstimation.md) and [math.md](math.md).

## 1. Calculate joint torques from actual current

For each joint $i\in\{1,2,3,4\}$, convert actual current in A into joint torque in N m:

$$
\boxed{\tau_{\mathrm{current},i}=s_iK_iI_{\mathrm{actual},i}}
$$

This is the torque derived from actual current before subtracting the target
current or other compensation terms.

| Symbol | Meaning |
| --- | --- |
| $s_i$ | Joint torque sign, $+1$ or $-1$ |
| $K_i$ | Effective current-to-joint-torque gain, N m/A, including transmission conversion |

For a simple geared actuator, $K_i\approx N_i\eta_iK_{t,i}$, where $N_i$ is the
reduction ratio, $\eta_i$ the efficiency, and $K_{t,i}$ the motor torque constant.
Gains and signs must match the estimator's joint coordinates; the default
$K_i=1\ \mathrm{N\,m/A}$ alone does not establish a calibrated torque scale.

## 2. Estimate joint torques due to external force

Subtract the expected effort without external contact from the current-derived
torque. For stationary or slow motion with negligible inertial effects, use:

$$
\boxed{
\hat\tau_{\mathrm{ext},i}
=\tau_{\mathrm{current},i}
-s_iK_iI_{\mathrm{target},i}
-s_iK_i(b_i+\bar b_i)
-p_i(q)-f_i(\dot q_i)
}
$$

Every subtracted term is a torque in N m. In particular, the target current is
converted to torque by $s_iK_i$ before subtraction.

| Compensation term | Meaning |
| --- | --- |
| $s_iK_iI_{\mathrm{target},i}$ | Posture-dependent baseline torque, including gravity-related holding effort |
| $s_iK_i(b_i+\bar b_i)$ | Current offsets converted to torque: fixed bias $b_i$ and optional startup bias $\bar b_i$, both in A |
| $p_i(q)$ | Remaining posture-dependent baseline torque |
| $f_i(\dot q_i)$ | Velocity-dependent friction torque |

The startup bias is calculated once and held fixed; it is zero by default.
Use actual and target currents from the same feedback sample, with the same
units and scaling. With the definition

$$
I_{\mathrm{actual},i}-I_{\mathrm{target},i}=\Delta I_i,
$$

the combined expression is:

$$
\hat\tau_{\mathrm{ext},i}
=s_iK_i(\Delta I_i-b_i-\bar b_i)-p_i(q)-f_i(\dot q_i).
$$

### Target current and posture correction

In enabled, stopped tests, target current changed with posture and remained
constant under external force, while actual current responded to the force.

Fit $p_i(q)$ to the no-contact residual after target-current, bias, and friction
subtraction. It corrects remaining gravity-related posture effects. With perfect
target-current compensation and correct bias/friction compensation, $p_i(q)=0$.

### Friction

$$
f_i(\dot q_i)=v_i\dot q_i+c_i\tanh\left(\frac{\dot q_i}{\epsilon}\right).
$$

Here $\dot q_i$ is velocity (rad/s), $v_i$ the viscous coefficient (N m/(rad/s)),
$c_i$ the Coulomb coefficient (N m), and $\epsilon>0$ the smoothing scale (rad/s).
Since $f_i(0)=0$, static friction can remain.

The residual is before deadbands and uses the sign of torque applied by the robot;
the environment's torque on the robot has the opposite sign.

Implementation: [calculation](../mg400_interface/src/external_force_estimator.cpp),
[parameters](../mg400_interface/include/mg400_interface/external_force_estimator.hpp).
`target_current_scale` is planned for removal and is omitted from the theory.

## 3. Convert joint torques to endpoint force using the Jacobian

Let $q=[q_1,q_2,q_3,q_4]^T$ be the joint angles in radians and
$\xi(q)=[x,y,z,\psi]^T$ the endpoint position in metres and yaw in radians.
The $4\times4$ Jacobian relates small joint and endpoint displacements:

$$
J(q)=\frac{\partial\xi}{\partial q},
\qquad
d\xi=J(q)\,dq.
$$

In `buildTaskJacobian()`, each column is approximated by **central differences**,
using forward kinematics at two nearby model configurations:

$$
J_{:,i}(q)\approx
\frac{\xi(q+h e_i)-\xi(q-h e_i)}{2h}.
$$

Here $e_i$ has 1 in entry $i$ and 0 elsewhere, so only joint $i$ changes.
The step $h$ is `jacobian_diff_step`, defaulting to $10^{-5}$ rad.
Repeat for J1–J4 to obtain all four columns. Before division, wrap the yaw
difference into $[-\pi,\pi]$ to handle angle discontinuities.
These perturbations are applied only to the model inputs.

Define the endpoint wrench as $w=[F_x,F_y,F_z,T_z]^T$, with forces in N and
yaw moment in N m. Using Section 2's convention for torque applied by the robot,
equal virtual work for any $dq$ gives:

$$
\tau_{\mathrm{ext}}^T dq=w^T d\xi=w^T J(q)\,dq
\quad\Longrightarrow\quad
\boxed{\tau_{\mathrm{ext}}=J(q)^T w}.
$$

This is the static relation between joint torque and endpoint wrench
([Modern Robotics: Statics](https://modernrobotics.northwestern.edu/nu-gm-book-resource/5-2-statics-of-open-chains/)).
Stack the four estimated torques from Section 2 into $\hat\tau_{\mathrm{ext}}$.
For an invertible, well-conditioned Jacobian:

$$
\boxed{\hat w=\left(J(q)^T\right)^{-1}\hat\tau_{\mathrm{ext}}}.
$$

The implementation uses damped least squares to limit noise amplification
near singularities. It estimates
$F_x,F_y,F_z,T_z$; $T_x$ and $T_y$ are unestimated.

The current Jacobian is evaluated at the flange, with components along the
robot-origin axes. For a displaced tool tip, use its Jacobian or shift the flange
wrench to that point.

## 4. Handle estimates near singularities

Near a singular posture, some wrench directions produce little joint torque,
so inversion amplifies measurement and model errors. The estimator applies
damping on every solve and reuses a previous estimate when conditioning is too poor.

### Damped solve

Let $A=J^T=U\,\mathrm{diag}(\sigma_j)V^T$ be its singular-value decomposition.
Let $\tau_{\mathrm{db}}$ be the joint-torque estimates after their input deadband.
The solver uses:

$$
\hat w_{\lambda}
=V\,\mathrm{diag}\left(\frac{\sigma_j}{\sigma_j^2+\lambda^2}\right)
U^T\tau_{\mathrm{db}}.
$$

The damping parameter $\lambda$ defaults to $10^{-4}$. Each gain approaches zero
as $\sigma_j\to0$, suppressing poorly observed directions. Damping limits noise
amplification but cannot recover unobservable wrench directions.

### Reuse the previous estimate

The smallest singular value $\sigma_{\min}$ is compared with
`deterministic_min_singular_value`, defaulting to $5\times10^{-4}$:

| Condition | Value passed to magnitude and rate limits |
| --- | --- |
| $\sigma_{\min}\ge5\times10^{-4}$ | New damped estimate |
| $\sigma_{\min}<5\times10^{-4}$ | Previous estimate after magnitude/rate limits; zero if no previous value exists |

A separate `min_singular_value` threshold, defaulting to $10^{-4}$, emits a
solver warning. The reuse branch also emits its own warning. These numerical
thresholds use the implementation's metre/radian scaling.

Exponential smoothing and output deadbands follow, so the published value can
continue settling while the previous estimate is reused.
**A reused value carries historical information; it is not a fresh force measurement.**

## 5. Process the estimate before publishing

After singularity handling, the following operations run in order. Let
$F=[F_x,F_y,F_z]^T$; force limits apply to the vector magnitude.

| Step | Default behavior |
| --- | --- |
| Magnitude limits | Cap $\lVert F\rVert$ at 200 N and $T_z$ at $\pm20$ N m |
| Rate limits | After initialization, limit force-vector changes to 250 N/s and $T_z$ changes to 50 N m/s |
| First-order low-pass filter | $-3$ dB cutoff: approximately 4.46 Hz at 125 Hz updates ($\alpha=0.2$: 20% new estimate + 80% previous filtered estimate) |
| Output deadbands | Force threshold: 1 N; yaw-moment threshold: 1 N m. Set magnitudes at or below the threshold to zero; otherwise subtract the threshold from the magnitude |

Rate limits use the configured update period, defaulting to $1/125$ s:
at most 2 N of force-vector change and 0.4 N m of yaw-moment change per update.

Output deadbands are applied afterward, preserving force direction and moment
sign. For example, a force magnitude of 3 N immediately before the output
deadband becomes 2 N. Thus the published value can be smaller and respond
more slowly than the estimate obtained from the Jacobian.
