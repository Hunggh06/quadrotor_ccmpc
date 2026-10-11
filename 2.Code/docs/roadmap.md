# Roadmap

> **In short:** what is still open, in priority order. Each item says why it
> matters, where the code lives and what "done" means. Nothing here is a hidden
> capability of the current release.
>
> **Related:** [design decisions](reference/design-decisions.md),
> [claims and statistics](validation/claims-and-statistics.md),
> [results history](validation/results-history.md)

## 1. A validation campaign for the current code

**Why.** Every published number (root README, [results history](validation/results-history.md),
notebook 09) comes from the Stage 8 campaign, which predates the October 2026
corrections: constraints on nodes `1..N`, post-solve re-projection and risk
certificate, enforced deadline fallback, the one-sided Clopper–Pearson gate
and the worst-trial timing gate. There are no results yet for the code as it
is.

**How.** On a clean commit, run the default protocol (50 paired seeds × 3
covariance levels × 3 controllers = 450 episodes) from the workbench
**Monte Carlo** page or:

```bash
uv run quadrotor-mpc-monte-carlo --config config/native_monte_carlo.yaml --workers 3 --batch-size 5
```

**Done when.** The run is committed under `2.Code/validation/`, and the
README validation table, the results history and notebook 09 quote it.
The gate rules in [claims and statistics](validation/claims-and-statistics.md)
are fixed and must not be changed after seeing the results.

## 2. Real-time solve within the 50 ms period

**Why.** Chance-controller p99 solve times were 87–95 ms in Stage 8 (81.6 ms
p99 in the Stage 6 single run) against a 50 ms control period. Until this is
fixed, only the `algorithmic_comparison` protocol is meaningful, and the
supplied CC-MPC scenario applies late solutions (`reject_on_deadline_miss:
false`).

**Where.** IPOPT options are set in `quadrotor_mpc/control/nmpc/core.py`
(`max_iter` 60, `tol` 1e-4, default linear solver); the horizon length is
`controller.horizon_steps` (20) in the configs.

**Candidates, none measured yet:** a faster sparse linear solver (HSL
MA27/MA57), a shorter horizon or coarser stages, compiled CasADi functions,
a real-time-iteration scheme (one SQP step per tick, e.g. acados).

**Done when.** A `realtime_qualification` campaign passes the timing gate (worst
trial p99 ≤ 50 ms on the end-to-end clock) without changing the safety
semantics.

## 3. A native scenario where CC-MPC and deterministic MPC differ

**Why.** In Stage 8 the deterministic controller had 0/90 collisions, the same
as the chance controllers. The scenario shows larger clearance for the chance
controllers but cannot show that they avoid collisions the deterministic
controller would cause. The paired McNemar tests therefore have nothing to
detect.

**How.** Design a native scenario (see [scenarios](guides/scenarios.md)) where
estimation and prediction error are large relative to the base safety margin:
for example tighter gaps, faster or less predictable obstacles, or a smaller
`safety_margin`. Fix it before running the comparison campaign.

**Done when.** A paired campaign on that scenario reports a non-trivial
deterministic collision rate, so the comparison has statistical power.

## 4. Joint risk at high uncertainty

**Why.** At `4 Sigma` the joint controller reached the goal in 0/30 episodes
and fell back in 30/30 (Stage 8): with a uniform split, each cell gets
`ε_total / (N·n_obs)`, which gives keep-out radii that block the path.

**Where.** `quadrotor_mpc/control/nmpc/risk_budget.py`. The
[architecture](reference/architecture.md#control-pipeline-boundary-controlnmpc-vs-controlccmpc)
reserves exactly one extension point for an adaptive allocator in
`control/nmpc/`. It must not be duplicated into `control/ccmpc/`.

**Done when.** A non-uniform allocator keeps `Σ ε ≤ ε_total` (checked by the
same `BUDGET_OK` accounting) and improves goal success at `4 Sigma` in a paired
campaign.

## 5. Smaller items

- **Covariance iteration.** One outer covariance iteration per tick is used;
  re-solving after updating the nominal trajectory is deferred
  ([chance constraints](concepts/chance-constraints.md#per-tick-order)).
- **Obstacle geometry.** The native controller supports spherical obstacles
  only; the 9-state track already has ellipsoids.
- **Correlated errors.** Vehicle and obstacle errors are assumed independent
  (zero cross-covariance).
- **Theory chapters.** `1.Docs/Theory/` is written in Vietnamese for Obsidian
  and still uses the old package name `quadrotor_ccmpc`. Translate it, or
  keep it as background material and update the code names.

## Out of scope

These are separate research or deployment tracks, not gaps in this release
(see the root README, *Scope boundaries*): a depth-camera detection pipeline,
ROS 2 integration, hardware-in-the-loop or real flight, and a formal
episode-wide collision-probability proof.
