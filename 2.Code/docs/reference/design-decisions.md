# Design decisions

> **In short:** the choices that shape the code and the validation, why they
> were made and what was rejected. Read this before changing one of them; most
> are guarded by tests or by the claim rules.
>
> **Related:** [architecture](architecture.md),
> [claims and statistics](../validation/claims-and-statistics.md),
> [roadmap](../roadmap.md)

Each entry follows the same pattern: **decision**, **why**, **rejected
alternative**, **consequence**.

## Modelling

### Two model tracks that stay separate

- **Decision.** Keep a 9-state ODE track (CVXPY QP, `control/ccmpc/`) and a
  13-state quaternion NMPC on a MuJoCo Crazyflie 2 (`control/nmpc/`). They do
  not share a plant model.
- **Why.** The 9-state track is small enough to teach and to sweep. The native
  track flies a model it does not contain exactly, which exposes model
  mismatch instead of hiding it.
- **Rejected.** One unified model, which would make the controller's model
  and the plant identical and the results optimistic.
- **Consequence.** Only `control/nmpc/` produces Monte Carlo evidence. New
  research features go there and are not duplicated into `control/ccmpc/`
  ([architecture](architecture.md#control-pipeline-boundary-controlnmpc-vs-controlccmpc)).

### The controller sees beliefs, never truth

- **Decision.** Every controller implements `reset(belief)` /
  `solve(belief, obstacle_beliefs, goal, t)`. Truth reaches it only through
  the seeded sensors, the 12D error-state EKF and the obstacle trackers. The
  deterministic baseline gets zero-covariance beliefs built from truth.
- **Why.** Chance constraints are only meaningful if the uncertainty they use
  comes from an estimator that can be wrong.
- **Consequence.** Deterministic and chance controllers run on the same
  plant, estimator and seed, so paired comparisons are fair
  ([controller interface](controller-interface.md)).

## Chance constraints

### Covariance propagation outside the NLP

- **Decision.** Propagate vehicle and obstacle covariance along the shifted
  nominal trajectory before the solve, and pass σ, β, ε and the tightened
  radius as do-mpc time-varying parameters.
- **Why.** The compiled NLP stays fixed (geometry and obstacle count define
  it), so beliefs can change every tick without rebuilding, and the solve
  stays smaller.
- **Not chosen.** Covariance as decision variables inside the NLP: larger and
  slower, for a controller that already misses 50 ms.
- **Consequence.** The constraint uses collision normals from the seed, not
  from the solution. The post-solve re-projection below measures that gap.

### Constraints on nodes 1..N, not 0..N−1

- **Decision.** Obstacle constraints act on the successor states `x_1..x_N`.
  Node 0 gets zero risk and the untightened radius.
- **Why.** `x_0` is the measured current state; no control can change it.
  Constraining it spent risk on an event that could only turn into slack,
  while the terminal node `x_N` was never constrained (the bug fixed in
  October 2026).
- **Consequence.** The joint budget is split over `N · n_obs` cells
  ([risk budget](../concepts/risk-budget.md)).

### Uniform joint allocation by Boole's inequality

- **Decision.** In joint mode each cell gets `ε_total / (N · n_obs)`. The sum
  is checked against `ε_total` on every solve (`BUDGET_OK`).
- **Why.** It is the simplest allocation with an exact, auditable union bound
  over one horizon, and unsupported allocators fail validation instead of
  silently falling back.
- **Rejected for now.** Adaptive or geometry-aware allocation. It is the main
  open research item ([roadmap](../roadmap.md#4-joint-risk-at-high-uncertainty)).
- **Consequence.** The bound covers one prediction horizon, never the
  receding-horizon episode.

### Soft constraints whose slack is always reported

- **Decision.** Each chance constraint has a nonnegative slack with a large
  penalty (`slack_penalty: 1e6`). The minimum slack needed by the optimized
  solution is recorded on every tick.
- **Why.** Hard constraints make IPOPT report infeasibility in exactly the
  situations where a usable command is most needed. Slack keeps the solver
  usable without hiding the relaxation.
- **Consequence.** Positive slack is never shown as guarantee-eligible. Its
  effect on the claim is decided by the risk certificate.

### Post-solve risk certificate instead of direction-robust tightening

- **Decision.** After each solve, re-project the covariances on the optimized
  trajectory's normals, compute the Gaussian crossing probability of the
  untightened sphere for every cell, and mark the tick `CERTIFIED` if the sum
  is within `ε_total` (joint) or every cell is within `ε_ind` (individual).
  The supervisor may accept slack only with a `CERTIFIED` tick
  (`accept_certified_slack`).
- **Why.** In October 2026 (joint, seeds 1000–1001) every remaining
  positive-slack cell turned out to be a re-projection shortfall: 98% at the
  terminal node, median 0.1–0.4 mm. The horizon risk sum stayed far below the
  budget (max 0.05 against 0.10). The slack rule was blocking claims the
  union bound actually supported.
- **Rejected.** Tightening with the largest σ over all directions (`σ_max`).
  It removes those cells but adds up to 74 mm of radius (p95 14 mm), which
  would make the controller much more conservative to fix sub-millimetre
  artefacts.
- **Consequence.** `chance_claim_rule: risk_certificate` is the default. The
  stricter legacy rule `zero_positive_slack` remains selectable
  ([chance constraints](../concepts/chance-constraints.md#post-solve-risk-certificate)).

## Safety

### A supervisor that only lets checked commands through

- **Decision.** `SafeFallbackController` wraps the controller. It checks solver
  success, residuals, `BUDGET_OK`, actuator bounds, slack and the deadline,
  and on any failure applies hold-last → position-hold PD → emergency hover.
- **Why.** The optimizer only proposes. Known-invalid or stale commands must
  not reach the plant silently, and solver exceptions must not stop the loop.
- **Consequence.** Every applied command, fallback included, uses the same
  `ControlSolution` contract and records why it was chosen
  ([safety supervisor](../concepts/safety-fallback.md)).

## Validation

### One-sided exact Clopper–Pearson bound for the collision gate

- **Decision.** The collision gate compares the one-sided 95% Clopper–Pearson
  upper bound with 0.10. Two-sided Wilson intervals are still reported for
  every rate, as description.
- **Why.** The safety question is one-sided ("is the rate below the limit?"),
  and Clopper–Pearson is conservative for every true rate. Wilson is
  approximate and can undercover at small counts near zero. With 0 collisions
  in 50 trials the bound is 0.0582.
- **Consequence.** The rule was fixed **before** the next campaign and must
  not be changed after seeing results. The protocol fingerprint records the
  method. Stage 8 is not re-graded
  ([claims and statistics](../validation/claims-and-statistics.md#statistical-rules)).

### Paired seeds with McNemar and Holm

- **Decision.** Every controller runs on the same seeds. Collision and success
  are compared with exact McNemar tests on discordant seeds, Holm-adjusted
  over all cells.
- **Why.** Pairing removes scenario-to-scenario variance, and Holm controls
  the family-wise error across the 3 × 2 comparisons.

### Worst-trial timing gate and two protocol types

- **Decision.** The timing gate requires the *worst* trial's p99 to fit the
  control period. Only the `realtime_qualification` protocol (late commands
  rejected to fallback) can pass a real-time claim. `algorithmic_comparison`
  applies late commands and records them.
- **Why.** An average hides the trials that would miss deadlines in flight.
  Separating the protocols lets allocators be compared on quality without
  deadline rejection distorting the result.

### Release evidence only from a known source

- **Decision.** Campaigns are release-eligible only from a clean Git commit or
  a fingerprinted, non-editable installed wheel. `--allow-dirty-source` runs
  are marked non-release-eligible.
- **Why.** A result must point to the exact code that produced it.

## Code structure

### Layered package guarded by a test

- **Decision.** `core` ← `control`/`estimation` ← `infrastructure` ←
  `application` ← `interfaces`. Inner layers never import outer ones, and
  `tests/test_architecture.py` enforces it.
- **Why.** Numerical code can be imported without starting do-mpc, MuJoCo, Qt
  or Streamlit, and UI changes cannot alter control behaviour.
- **Consequence.** The Qt panel and the workbench jobs run in separate
  processes and only exchange plain dictionaries
  ([architecture](architecture.md)).
