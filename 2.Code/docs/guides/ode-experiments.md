# ODE experiments (9-state track)

> **In short:** compare deterministic MPC and CC-MPC on the fast 9-state model
> with paired seeds, run Monte Carlo and parameter sweeps, and get a complete
> report folder for every run.
>
> **Related:** [scenarios](scenarios.md#ode-scenarios-configscenarios),
> [metrics](../reference/metrics.md) · **Code:**
> `quadrotor_mpc/application/simulation/`, `quadrotor_mpc/application/experiments/`

For the 13-state MuJoCo pipeline use [Monte Carlo campaigns](monte-carlo.md)
instead.

## Run a paired comparison

```bash
quadrotor-mpc-run --config config/scenarios/static_obstacle.yaml --compare
```

Ten paired seeds:

```bash
quadrotor-mpc-run --config config/scenarios/moving_obstacle.yaml \
  --compare --trials 10 --seed 100
```

A single closed loop without the report bundle:

```bash
quadrotor-mpc-sim --config config/scenarios/point_to_point.yaml
```

Two solver backends are available: `scipy` (nonlinear rollout with soft chance
constraints; suitable for demos, CI and sweeps) and `cvxpy` (sequential-convex
QP, closer to the reference formulation).

## Parameter sweeps

Study the risk/performance trade-off:

```bash
quadrotor-mpc-sweep --config config/scenarios/static_obstacle.yaml \
  --parameter delta --values 0.01 0.03 0.05 0.10 0.20 \
  --mode both --trials 10
```

Supported parameters: `delta`, `measurement_pos`, `process_vel`, `drag_scale`,
`obstacle_speed_scale` and `horizon_steps`. A sweep writes raw CSV, aggregate
JSON and a four-panel PNG.

## Streamlit dashboard

```bash
quadrotor-mpc-dashboard
```

| Page | Purpose |
|---|---|
| Home | architecture and workflow |
| Scenario Builder | edit, validate and download scenario YAML |
| Live Simulation | 3-D path, predicted horizon, telemetry, uncertainty and solver |
| Compare Controllers | paired deterministic MPC versus CC-MPC |
| Monte Carlo | multi-seed distributions and aggregate metrics |
| Experiment Explorer | reopen manifests, metrics and interactive reports |
| Theory Mode | map equations to code, tests and plots |
| NMPC + MuJoCo | optional 13-state plant track |

## Experiment rules

Controller comparisons must use paired conditions:

1. Same scenario, plant model, start and goal.
2. Same seed and therefore the same disturbance realization.
3. Same stop condition, simulation duration and control frequency.
4. Same obstacle trajectories and measurement assumptions.
5. Solver failures, slack and deadline misses are recorded, never filtered.

Use at least 30 seeds for descriptive Monte Carlo work and more for estimating
rare collision probabilities. Report the number of trials together with
confidence intervals. A single trajectory is a diagnostic example, not
evidence of robustness.

The comparison groups are tracking, safety, efficiency, solver timing and
reliability. The paired delta in `metrics.json` is CC-MPC minus deterministic
MPC; interpret its sign per metric (see [metrics](../reference/metrics.md)).

## Run folder

Every `quadrotor-mpc-run` creates an immutable folder:

```text
outputs/runs/<run-id>/
├── manifest.yaml        timestamp, Python/platform, dependencies, commit, seeds
├── scenario.yaml        scenario snapshot
├── controller.yaml      controller snapshot
├── metrics.json         per-run and aggregate metrics, paired deltas
├── comparison.csv
├── report.png
├── report.html          self-contained interactive report
├── *-summary.json
├── *-timeseries.csv     actual/estimated/reference states, commands, sigmas,
│                        clearance, residual, slack, solve time, cost terms
├── *-predictions.npz    predicted horizons
└── *-events.jsonl
```

## Reproducibility

- Re-run from the snapshots in the run folder, not from a later edited global
  configuration.
- Archive the whole run folder with a manuscript or benchmark release.
- Never remove failed trials from aggregate results; classify and report them.
