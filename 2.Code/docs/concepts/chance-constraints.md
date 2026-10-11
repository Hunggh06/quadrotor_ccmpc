# 4 · Chance constraints

> **In short:** "the probability of hitting obstacle *o* at node *k* is at
> most ε" is replaced by a deterministic distance constraint whose radius grows
> with the projected uncertainty: `r_tight = r_base + Φ⁻¹(1-ε)·σ`. After the
> solve, the result is re-checked and given a risk certificate. (Stage 4.)
>
> **Before this:** [covariance propagation](covariance-propagation.md) ·
> **Next:** [risk budget](risk-budget.md) · **Code:**
> `quadrotor_mpc/control/nmpc/chance_constraints.py`,
> `quadrotor_mpc/control/nmpc/chance_constrained.py`

The native controller supports spherical obstacles only. Covariance
propagation stays outside the do-mpc nonlinear program, and the deterministic
controller remains available with the same plant, estimator and seed. How ε is
chosen per cell is the job of the [risk budget](risk-budget.md); what happens
when a solution needs slack is decided by the
[safety supervisor](safety-fallback.md).

![Keep-out radius along the horizon for the three controllers](../../../assets/readme/tightening.png)

*Required distance to the static gate along the 20-node horizon in one tick of
`mujoco_native_ccmpc.yaml`: fixed for the deterministic controller, growing
with propagated uncertainty for individual risk, and growing further for joint
risk, which spends a smaller ε per cell.*

## Constraint

For horizon step \(i\) and obstacle \(j\), define the relative mean position:

\[
d_{i,j} = \mu_{p,i} - \mu_{o,i,j},
\qquad
n_{i,j} = \frac{d_{i,j}}{\|d_{i,j}\|}.
\]

The relative position covariance is:

\[
P_{\mathrm{rel},i,j}
=
P_{p,i}^{(3\times3)}
+
P_{o,i,j}^{(3\times3)}.
\]

Independence between vehicle and obstacle estimation errors is assumed. The
standard deviation projected onto the collision normal is:

\[
\sigma_{i,j}
=
\sqrt{
n_{i,j}^{T}P_{\mathrm{rel},i,j}n_{i,j}
}.
\]

For allocated violation probability \(\epsilon_{i,j}\):

\[
\beta_{i,j} = \Phi^{-1}(1-\epsilon_{i,j}),
\qquad
r_{\mathrm{tight},i,j}
=
r_{\mathrm{drone}}+r_{\mathrm{obs},j}+r_{\mathrm{margin}}
+\beta_{i,j}\sigma_{i,j}.
\]

The deterministic nonlinear constraint supplied to do-mpc is:

\[
\|p_i-\mu_{o,i,j}\|
\ge
r_{\mathrm{tight},i,j}-s_{i,j},
\qquad s_{i,j}\ge0.
\]

do-mpc uses the same \(10^{-6}\,\mathrm{m}^2\) smoothing term as the residual
telemetry. The recorded slack is the minimum nonnegative slack required by the
optimized nominal solution.

## Per-tick order

1. Shift the previous optimized state/control trajectory.
2. On the first tick, initialize a straight-line position seed.
3. Linearize the 12D quaternion error-state dynamics around the seed.
4. Propagate vehicle and 6D obstacle covariance over \(N+1\) points.
5. Compute collision normals, projected sigma and tightened radii.
6. Allocate risk and fill do-mpc TVPs for mean, sigma, beta, risk and radius.
7. Solve the native NMPC once.
8. Re-project the same covariances on the **optimized** trajectory's collision
   normals (post-solve profile, every solve) and evaluate margins and slack
   against those radii. The enforced radii used the seed normals; any
   shortfall caused by the solver moving the trajectory is reported as slack.
9. Record chance residual, slack, risk, covariance and solver classification.

This is one outer covariance iteration. Re-solving repeatedly after updating
the nominal trajectory is intentionally deferred.

## Configuration

```yaml
controller:
  covariance_propagation:
    enabled: true
    mode: open_loop

  chance_constraints:
    enabled: true
    type: spherical
    individual_epsilon: 0.05
    risk_budget:
      semantics: joint
      allocation: uniform
      total_epsilon: 0.10
    soft_constraint: true
    slack_penalty: 1000000.0
    slack_tolerance_m: 1.0e-6
```

Run the complete estimator/covariance/CC-NMPC scenario:

```bash
MUJOCO_GL=glfw quadrotor-mpc-native \
  --config config/mujoco_native_ccmpc.yaml
```

## Status semantics

- `SOLVED_SAFE`: optimized chance residuals are nonnegative within tolerance.
- `SOLVED_WITH_SLACK`: a solution exists, but one or more chance constraints
  require positive slack. The chance guarantee is not satisfied.
- `SOLVED_DETERMINISTIC`: chance tightening is disabled.

Fallback and emergency handling happen after the solve, in the
[safety supervisor](safety-fallback.md).

## Assumptions and limitations

- Gaussian local position errors.
- Vehicle and obstacle errors are independent; cross-covariance is zero.
- Collision normal is frozen from the shifted nominal trajectory for the
  enforced constraint; the post-solve check re-projects on the solution's
  normals but does not re-solve.
- Row 0 of every horizon array is the measured current state. It carries zero
  risk and the untightened base radius; positive slack there means the vehicle
  is already inside the deterministic safety distance.
- In `individual` mode, every horizon-step/obstacle pair uses the same
  `individual_epsilon`; this is not a joint statement.
- In `joint` mode, the uniform allocator accounts for every constrained node
  \(1..N\) and obstacle (\(N\,n_{\mathrm{obs}}\) cells). The budget applies to one prediction horizon, not the
  complete receding-horizon episode.
- Spherical obstacle geometry only.
- Positive slack (a cell above its allocated share at the optimized
  trajectory) is always reported and never hidden. For `joint` semantics the
  horizon claim is decided by the post-solve risk certificate below; without a
  certificate, positive slack blocks the claim.

## Post-solve risk certificate

After every chance solve the controller evaluates, on the optimized
trajectory and the solution's collision normals,

\[
\hat\epsilon_{k,o}=1-\Phi\!\left(\frac{d_{k,o}-r_{\mathrm{base},o}}{\sigma_{k,o}}\right),
\qquad k=1..N,
\]

the linearized Gaussian probability of crossing the *untightened* safety
sphere. `joint`: `CERTIFIED` iff \(\sum_{k,o}\hat\epsilon_{k,o}\le\epsilon_{\mathrm{total}}\)
(the Boole bound the joint claim relies on); `individual`: iff every
\(\hat\epsilon_{k,o}\le\epsilon_{\mathrm{ind}}\). The result is exposed as
`achieved_risk_total`, `achieved_risk_max` and `risk_certificate_status`.

Diagnosis behind this rule (October 2026, joint, seeds 1000–1001): every
remaining positive-slack cell was a re-projection shortfall (enforced radius
met, solution-normal radius missed), 98% of them at the terminal node,
median 0.1–0.4 mm, max ~11 mm; node 0 never carried slack. The horizon risk
sum stayed far below the budget (median 0.002–0.004, max 0.05 vs 0.10).
A direction-robust tightening (\(\sigma_{\max}\)) would remove these cells but
adds up to 74 mm of radius (p95 14 mm), so the certificate was preferred.

## 9-state ODE track: ellipsoidal residual

The ODE track uses rotated ellipsoids instead of spheres. With
`S = Omega^(1/2)`, `y = S(p - p_obs)` and `n = y / ||y||`, the logged residual
is

```text
g = ||y|| - 1 - Phi^-1(1-delta) * sqrt(n' S (Sigma + Sigma_obs) S' n)
```

`g >= 0` is locally safe under the Gaussian approximation. A nonzero soft slack
must always be reported: it means the optimization stayed numerically usable
but the hard probability statement was relaxed. Code:
`quadrotor_mpc/control/ccmpc/risk.py`.
