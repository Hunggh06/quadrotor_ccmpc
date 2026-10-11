# Test strategy

> **In short:** what is tested at each level, and the checks each pipeline
> stage must pass before its output is trusted.
>
> **Related:** [claims and statistics](claims-and-statistics.md),
> [results history](results-history.md), [release checklist](../guides/release.md)

Run the whole suite from `2.Code/`:

```bash
uv run pytest -q
```

## Test layers

1. Unit tests: dynamics equilibrium, linearization, rotated matrix square root and covariance PSD.
2. Integration tests: deterministic repeatability, artifact schema and adaptive reports.
3. Scenario regression: hover, point-to-point, static and moving obstacles.
4. Statistical validation: paired Monte Carlo and parameter sweeps.
5. Cross-plant validation: 9-state ODE controller model versus 13-state MuJoCo plant.
6. MuJoCo model validation: sourced mass/inertia, MJCF compilation, hover equilibrium,
   collision contacts and native-viewer lifecycle.
7. Native interaction validation: legacy/new obstacle schemas, horizon agreement,
   command ordering, reset/pause/single-step behavior and recording round-trip.
8. Controller-contract validation: state/covariance dimensions, quaternion
   normalization, positive-semidefinite covariance, horizon shape agreement and
   normalized solver diagnostics.
9. Native estimation validation: seeded reset repeatability, dropout behavior,
   quaternion local-error consistency, ESEKF update contraction, covariance PSD,
   position-only obstacle velocity recovery and truth-free tracker horizons.
10. Horizon covariance validation: initial-belief agreement, local-error
    linearization, vehicle/obstacle horizon dimensions, symmetry, PSD, open-loop
    growth, feedback-aware behavior and numeric recording round-trip.
11. Native spherical chance constraints: Gaussian quantile, relative covariance
    projection, TVP tightening, risk/slack shape agreement, status semantics and
    deterministic-versus-CC-MPC paired regression.
12. Native risk-budget management: legacy individual semantics, uniform joint
    allocation, exact sum audit, per-cell quantiles, configuration round-trip,
    telemetry/recording persistence and joint-versus-individual paired regression.
13. Native safety-console integration: deterministic/CC-MPC policy projection,
    guarantee/slack/fallback/deadline labeling, risk-budget failure visibility,
    transition deduplication, reset semantics and configuration round-trip.
14. Native Monte Carlo validation: paired seeds, covariance-level scaling,
    Wilson intervals, paired deltas, resumable artifacts and explicit
    probability-claim/timing gates.

For a native release, run both `config/mujoco_native.yaml` and
`config/mujoco_native_dynamic.yaml`. Check that the reported current obstacle
position equals the first point of its displayed prediction, replay loads with
`allow_pickle=False`, and closing either interactive window terminates cleanly.

Before claiming a chance-constraint probability, verify Gaussian assumptions, covariance
calibration, obstacle prediction error, finite-sample collision rate and nonzero slack. FOV and soft
constraints require their own residual plots and must not be inferred from trajectory appearance.

## Checks per pipeline stage

### Controller interface (Stage 1)

The deterministic baseline must stay numerically equivalent after any change
to the belief interface; reference values are in
[results history](results-history.md#stage-1--controller-interface-regression-dynamic-scenario).

The runtime integration tests also assert that `ControlSolution` diagnostics
reach `CoupledStep`, and that goal completion, `Run again`, reset, pause,
single-step and explicit stop preserve their v1.2.1 behavior.

### Sensing and estimation (Stage 2)

Run `config/mujoco_native_estimation.yaml` and report separately:

- vehicle position, velocity, attitude and body-rate RMSE;
- normalized estimation error squared when calibration is evaluated;
- vehicle and obstacle dropout counts;
- covariance minimum eigenvalue;
- obstacle position/velocity RMSE;
- deterministic versus estimated closed-loop safety metrics.

The controller must receive tracker-extrapolated obstacle means. Ground-truth
motion is allowed only in the sensor simulator, collision/clearance metrics and
validation logs.

### Covariance propagation (Stage 3)

Run `config/mujoco_native_estimation.yaml` and verify:

- `predicted_error_covariance_horizon.shape == (ticks, N+1, 12, 12)`;
- `predicted_obstacle_covariance_horizon.shape == (ticks, N+1, n_obs, 6, 6)`;
- horizon index zero matches the belief supplied to that controller tick;
- every covariance is finite, symmetric and positive semidefinite;
- terminal position sigma responds to process-noise configuration;
- deterministic control, clearance and collision regression are unchanged,
  because propagation alone does not tighten a constraint.

The `feedback_lqr` mode is checked as a propagation approximation, not as a
replacement controller. No chance-constraint guarantee may be claimed from
propagation alone.

### Chance constraints (Stage 4)

Run `config/mujoco_native_estimation.yaml` and
`config/mujoco_native_ccmpc.yaml` with the same seed. Verify:

- the deterministic run preserves the estimation-only final error and clearance;
- `tightened_safety_radius >= nominal_safety_radius`;
- `tightening == Phi^-1(1-epsilon) * projected_sigma`;
- `chance_residual + slack >= 0` within solver tolerance;
- positive slack is classified as `SOLVED_WITH_SLACK`;
- a run containing positive slack is not reported as satisfying the chance
  guarantee;
- CC-MPC produces a measurably different constraint/clearance from the paired
  deterministic run;
- no covariance, residual, control or state contains NaN.

### Risk budget (Stage 5)

Run `config/mujoco_native_ccmpc.yaml` and verify:

- `risk_semantics == joint` and `risk_allocation_method == uniform`;
- each solve allocates over `N * n_obstacles` scalar constraints on nodes
  `1..N`; node 0 carries zero risk;
- `sum(risk_allocation_horizon[t]) <= risk_budget_total[t] + tolerance`;
- `risk_budget_allocated == risk_budget_total == 0.10` for the supplied
  three-obstacle scenario;
- each TVP quantile equals `Phi^-1(1-risk_allocation[k,o])`;
- changing back to `individual` reproduces the individual-mode allocations
  and trajectory;
- joint allocation produces stronger tightening than individual allocation for
  the same belief/covariance;
- `BUDGET_OK` is not interpreted as a safety guarantee when slack is positive.

The joint budget is for one receding prediction horizon. Monte Carlo validation
with a confidence interval is required before making an episode-level collision
rate claim.


### Safety supervisor and fallback (Stage 6)

Verify the supervisor independently of the optimizer using fault injection:

- backend `success=false` rejects the primary solution;
- solver exceptions do not terminate the plant loop;
- command-bound, risk-budget, nonlinear residual, slack-limit and deadline
  gates each produce a specific rejection reason;
- the command source escalates through
  `HOLD_LAST_ACCEPTED -> POSITION_HOLD_PD -> EMERGENCY_HOVER`;
- every fallback command remains inside configured actuator bounds;
- Reset and Run again clear the last command, fallback reference and rejection
  count;
- telemetry/recording/replay preserve primary and fallback status;
- positive slack above the numerical guarantee tolerance is never labeled
  guarantee-eligible.

### Desktop safety panel (Stage 7)

Verify the presentation model independently of Qt:

- deterministic mode reports `DETERMINISTIC`, `DISABLED` risk/slack and
  `MONITOR ONLY`, never a chance guarantee;
- an accepted zero-slack joint-risk solution reports `PRIMARY NMPC`,
  `GUARANTEE ELIGIBLE`, `BUDGET_OK`, `HARD-SAFE` and `ON TIME`;
- positive slack reports `DEGRADED` and `NOT GUARANTEED`;
- fallback, deadline miss and risk-budget failure use high-visibility warning
  or danger states;
- repeated telemetry in the same state does not duplicate transition alerts;
- Reset and Run again clear history, alerts and prior transition state;
- replay projects the recorded sample fields through the same view model;
- all four native YAML configurations round-trip with bounded alert history.

With the optional UI dependencies, also launch
`config/mujoco_native_ccmpc.yaml` and check that the Qt panel stays responsive
while MuJoCo runs, closing either window stops cleanly, prediction colour
follows the assurance state and the deadline trace matches the configured
50 ms acceptance limit.

### Monte Carlo campaigns (Stage 8)

Run:

```bash
quadrotor-mpc-monte-carlo \
  --config config/native_monte_carlo.yaml \
  --workers 3
```

Verify:

- the raw table contains exactly
  `trials * modes * noise_levels` unique `(noise, mode, seed)` rows;
- deterministic, individual and joint controllers use identical seed sets at
  every uncertainty level;
- covariance scale `s` multiplies configured standard deviations by `sqrt(s)`;
- deterministic mode disables chance tightening while retaining the same
  estimator, plant, obstacles and safety-supervisor policy;
- every Bernoulli episode metric reports its event count, rate, a two-sided
  Wilson interval (descriptive) and a one-sided exact Clopper–Pearson upper
  bound; the collision gate uses the bound named by
  `statistics.collision_bound_method` (default `clopper_pearson_one_sided`);
- paired controller comparisons report exact McNemar tests for collision and
  success with Holm-adjusted p-values across all cells;
- risk-budget failures, slack and fallback remain separate metrics;
- guarantee eligibility, the claim rule and the timing gate follow
  [claims and statistics](claims-and-statistics.md);
- no per-horizon joint budget is reinterpreted as episode collision
  probability;
- interrupted runs resume only after a protocol-fingerprint match;
- manifest, protocol/config snapshots, checkpoint, CSV, aggregate JSON,
  Markdown and PNG are all present.


## Release checks

Version 2.0.1 added release gates around the research workbench (the
procedure is in the [release checklist](../guides/release.md)):

- the configured rejection deadline must not exceed the controller period;
- README commands and claims are checked against the repository;
- all runtime configs, tests and lint run in GitHub Actions;
- the Qt panel lifecycle and an installed wheel are smoke-tested;
- dependency resolution is frozen by `uv.lock`;
- native Monte Carlo release evidence refuses an uncommitted Git tree and
  records a deterministic validation-source SHA-256;
- an explicitly allowed dirty-source campaign is reproducible but marked
  non-release-eligible.

The 30-seed Stage 8 artifact stays unchanged for auditability. Its legacy
commit provenance is documented in `validation/README.md`; it must not be
presented as the clean 50-seed release campaign.
