# Controller interface

> **In short:** the native runtime talks to every controller through two
> calls, `reset(belief)` and `solve(beliefs, goal, t)`, and gets back one
> `ControlSolution`. This page lists every input and output field. (Stage 1.)
>
> **Related:** [architecture](architecture.md), [overview](../concepts/overview.md)
> · **Code:** `quadrotor_mpc/core/contracts.py`

The contract has no MuJoCo, CasADi or do-mpc dependency, which keeps the
runtime independent of the solver backend.

## Lifecycle

```python
controller.reset(initial_vehicle_belief)

solution = controller.solve(
    vehicle_belief,
    obstacle_beliefs,
    goal,
    time_s,
)
```

`reset()` clears controller history and warm-start state. It is called at
startup, Reset and Run again. `solve()` is the only control action used by
`quadrotor_mpc/application/native/runtime.py`.

## Input contracts

| Type | Meaning | Shape |
|---|---|---:|
| `VehicleBelief.mean_state_13` | `[p, v, q_wxyz, omega]` nominal state | `(13,)` |
| `VehicleBelief.error_covariance_12` | covariance of `[δp, δv, δθ, δω]` | `(12, 12)` |
| `ObstacleBelief.mean_state_6` | obstacle `[position, velocity]` mean | `(6,)` |
| `ObstacleBelief.covariance_6` | obstacle position/velocity covariance | `(6, 6)` |
| `ObstacleBelief.predicted_positions` | optional nonlinear mean horizon | `(N+1, 3)` |
| `ControlGoal.position` | world-frame goal position | `(3,)` |
| `ControlGoal.quaternion_wxyz` | normalized goal attitude | `(4,)` |

Covariances must be finite, symmetric and positive semidefinite. Quaternion
inputs are normalized at construction.

## Output contract

`ControlSolution` horizon arrays (`N` = horizon steps, `n_obs` = obstacles):

| Field | Meaning | Shape |
|---|---|---:|
| `command` | `[thrust_deviation, τx, τy, τz]` actually to be applied | `(4,)` |
| `nominal_states` | optimized mean state horizon | `(N+1, 13)` |
| `predicted_covariances` | error-state covariance horizon | `(N+1, 12, 12)` |
| `predicted_obstacle_covariances` | 6D covariance horizon per obstacle | `(N+1, n_obs, 6, 6)` |
| `projected_uncertainties` | collision-normal relative standard deviation | `(N+1, n_obs)` |
| `tightened_safety_radii` | deterministic radius supplied to the NLP (seed normals) | `(N+1, n_obs)` |
| `chance_margins` | signed margins vs the radius re-projected on the solution | `(N+1, n_obs)` |
| `risk_allocations` | allocated risk per horizon constraint | `(N+1, n_obs)` |
| `slacks` | nonnegative constraint relaxation | `(N+1, n_obs)` |

Row 0 of every horizon array is the measured current state. The NLP
constrains nodes `1..N` only, so row 0 carries zero risk and the untightened
base radius.

Scalar fields:

| Field | Meaning | Type |
|---|---|---|
| `solver_status` | backend-independent status label | string |
| `achieved_risk_total`, `achieved_risk_max` | post-solve linearized horizon risk (sum, largest cell) | scalar |
| `risk_certificate_status` | `CERTIFIED`, `EXCEEDED`, `NOT_APPLICABLE`, `NOT_AVAILABLE` | string |
| `risk_semantics` | `disabled`, `individual` or `joint` | string |
| `risk_allocation_method` | `none` or `uniform` | string |
| `risk_budget_total` | configured joint budget, otherwise `None` | scalar |
| `risk_budget_allocated` | sum of risk over the returned horizon | scalar |
| `risk_budget_remaining` | unallocated joint budget, otherwise `None` | scalar |
| `risk_constraint_count` | number of allocated scalar constraints | integer |
| `risk_budget_status` | allocation audit status | string |
| `primary_solver_status`, `primary_solver_success` | raw backend outcome retained across fallback | scalar |
| `primary_solver_iterations`, `primary_solver_*_residual` | final backend convergence evidence | scalar |
| `command_source`, `solution_accepted` | applied command owner and gate decision | scalar |
| `fallback_active`, `fallback_level`, `fallback_reason` | fallback classification | scalar |
| `consecutive_rejections` | current rejection streak | integer |
| `solve_time_ms`, `deadline_missed` | end-to-end controller timing evidence | scalar |
| `safety_assurance_status` | guarantee-eligibility/degraded label | string |

When covariance propagation is disabled, the deterministic adapter returns zero predicted
covariance and zero risk allocation. When propagation is enabled, the same
adapter returns vehicle and obstacle covariance horizons but still uses the
unchanged deterministic obstacle constraint. Its geometric margin and implied
violation slack are still populated, so every controller exercises the same
telemetry schema.

## Deterministic adapter

`DeterministicNMPCController` owns all do-mpc-specific operations:

- model/controller construction;
- TVP updates;
- warm-start reset;
- `make_step`;
- optimized-horizon extraction;
- conversion to `ControlSolution`.

All obstacle centers are TVPs. The compiled NLP depends on obstacle count and
shape, but obstacle mean trajectories may change on every tick.

## Belief sources

`quadrotor_mpc/estimation/truth.py` converts native truth into zero-covariance beliefs only
when `estimation.enabled: false`. It remains the deterministic regression
baseline.

When `estimation.enabled: true`, `quadrotor_mpc/estimation/native.py` provides:

```text
MuJoCo truth → sensor simulation → vehicle/obstacle estimators → beliefs
```

The vehicle estimator is a 12D error-state EKF around the 13D quaternion
nominal state. Each obstacle has a position-only constant-velocity Kalman
filter. Tracker means—not configured truth motion—form the obstacle horizon.

## Layers behind `solve()`

| Layer | Class / module | Page |
|---|---|---|
| Covariance propagation | `control/nmpc/covariance.py` (open-loop default, `feedback_lqr` diagnostic) | [Covariance propagation](../concepts/covariance-propagation.md) |
| Chance constraints | `SphericalChanceConstrainedNMPCController` projects relative covariance, supplies tightened radii as TVPs, reports residual and slack | [Chance constraints](../concepts/chance-constraints.md) |
| Risk allocation | `control/nmpc/risk_budget.py` | [Risk budget](../concepts/risk-budget.md) |
| Supervisor | `SafeFallbackController` decorates the same interface, keeps the rejected primary horizon for diagnostics and replaces `command` with the selected fallback; solver exceptions become a finite, shape-valid diagnostic solution | [Safety supervisor and fallback](../concepts/safety-fallback.md) |
