# Monte Carlo campaigns

> **In short:** run the complete native pipeline many times with paired seeds
> for every controller and noise level, then let fixed statistical gates decide
> what may be claimed. (Stage 8.)
>
> **Before this:** [native simulation](native-simulation.md) · **Then read:**
> [claims and statistics](../validation/claims-and-statistics.md) · **Code:**
> `quadrotor_mpc/application/validation/`

## What is evaluated

```text
seeded sensor simulation
  -> vehicle ESEKF and obstacle Kalman trackers
  -> horizon covariance propagation
  -> deterministic / individual-risk / joint-risk NMPC
  -> safety acceptance gates and fallback
  -> 13-state MuJoCo plant and geometric collision truth
```

The controller receives beliefs only; MuJoCo truth is used by the sensor
simulator and the validation metrics, never as hidden controller input. This
is separate from the lightweight 9-state Monte Carlo in
[ODE experiments](ode-experiments.md).

## Quick start

From the workbench: **Monte Carlo** page (Ctrl+2), adjust the form, press
**Start campaign**. From the command line:

```bash
quadrotor-mpc-monte-carlo \
  --config config/native_monte_carlo.yaml \
  --workers 3 \
  --batch-size 5
```

The default protocol is 450 episodes (50 seeds × 3 noise levels × 3
controllers). For a quick smoke test use fewer trials in the workbench form
and tick **Allow uncommitted source**.

## Default protocol

`config/native_monte_carlo.yaml` builds on `mujoco_native_ccmpc.yaml`:

| Axis | Values |
|---|---|
| Controller | `deterministic`, `individual`, `joint_uniform` |
| Covariance level | `0.25 Σ`, `Σ`, `4 Σ` |
| Trials | 50 paired seeds per cell, starting at 1000 |
| Plant, obstacles, duration | inherited from the base scenario |
| Collision gate | one-sided exact Clopper–Pearson 95% upper bound ≤ 0.10 |
| Descriptive intervals | two-sided Wilson 95% for every rate |
| Paired tests | exact McNemar (collision, success), Holm-adjusted across cells |
| Timing gate | worst trial: every per-trial p99 solve time ≤ 50 ms |

Controllers within a cell share the seed, plant, obstacle motion, duration,
goal, actuator limits and supervisor thresholds:

- `deterministic`: estimator and supervisor active; chance constraints and
  covariance propagation disabled.
- `individual`: the same ε for every horizon cell.
- `joint_uniform`: one total budget split uniformly over all cells.

A covariance level `s` multiplies covariances by `s`, so every affected
standard deviation is multiplied by `√s`: measurement noise, filter initial and
process uncertainty and prediction-horizon process uncertainty. Dropout
probabilities and scenario geometry are not scaled.

### Protocol types

`monte_carlo.protocol` selects how deadline misses are treated:

| Protocol | Deadline handling | Can pass the real-time claim |
|---|---|---|
| `algorithmic_comparison` (default) | late but valid commands are applied and recorded | no |
| `realtime_qualification` | late commands are rejected to fallback | yes, with the end-to-end deadline clock |

The algorithmic protocol compares allocators without letting deadline
rejection hide their quality; only the real-time protocol can support a
20 Hz claim.

## Running, stopping and resuming

`--workers 1` runs serially for maximum reproducibility and reuses one
compiled controller per cell. Parallel runs use spawned processes; each batch
still reuses one compiled controller for all its seeds.

Resume an interrupted campaign with the same effective overrides:

```bash
quadrotor-mpc-monte-carlo \
  --config config/native_monte_carlo.yaml \
  --workers 3 \
  --resume outputs/native_monte_carlo/<run-id>
```

Resume is refused when the protocol fingerprint or the validation-source
snapshot differs. The workbench offers interrupted campaigns in its
**Resume** list.

## Output folder

```text
outputs/native_monte_carlo/<run-id>/
├── manifest.yaml     state, Git commit/branch, clean flag, source SHA-256
├── protocol.yaml     effective protocol
├── base_config.yaml  effective base scenario
├── trials.jsonl      append-only checkpoint, one record per episode
├── trials.csv        raw table
├── aggregate.json    rates, intervals, paired tests, gates, verdict
├── report.md
└── report.png
```

`trials.jsonl` is appended after every episode (serial) or every completed
batch (parallel). The manifest starts as `RUNNING` and becomes `COMPLETED`
only after all aggregate files are written.

## Source provenance

Release evidence is accepted only from a clean Git worktree or a
fingerprinted, non-editable installed distribution. The manifest records the
commit, branch, clean state and a deterministic SHA-256 over
validation-relevant source, configuration and the MuJoCo MJCF/OBJ model.
Dirty worktrees, editable installs and unverified loose source trees are
refused.

`--allow-dirty-source` (workbench: **Allow uncommitted source**) is for smoke
tests only. Such a run is reproducible through its snapshot hash but is
labelled `FAIL_DIRTY_SOURCE` and is never release-eligible.

## Reading the result

The verdict and its gates are explained in
[claims and statistics](../validation/claims-and-statistics.md). Measured
values of earlier campaigns, with the code revision they belong to, are in
[results history](../validation/results-history.md).
