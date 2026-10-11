# Scenarios

> **In short:** a scenario is a YAML file with start, goal, obstacles and
> controller settings. Copy a shipped file and change it; always fix the seed
> for published results.
>
> **Related:** [first run](../getting-started/first-run.md#3-which-native-configuration-to-use),
> [native simulation](native-simulation.md), [ODE experiments](ode-experiments.md)

## Native scenarios (`config/`)

| Config | Obstacles | Estimation | Chance constraints | Purpose |
|---|---|---|---|---|
| `mujoco_native.yaml` | static | off (truth) | off | plant and viewer check |
| `mujoco_native_dynamic.yaml` | static + moving | off (truth) | off | deterministic regression baseline |
| `mujoco_native_estimation.yaml` | static + moving | on | off (covariance only) | estimation and propagated uncertainty |
| `mujoco_native_ccmpc.yaml` | static + moving | on | joint risk + supervisor | full pipeline; Monte Carlo base |

A native YAML defines the start and goal pose, actuator bounds and safety
margin, MPC and MuJoCo timesteps, horizon and stop conditions, estimation,
covariance propagation, chance constraints, safety supervisor, viewer, panel,
recording and obstacles. Check a file without running it:

```bash
quadrotor-mpc-native --config my_scenario.yaml --validate-config
```

The workbench **Simulate** page lists every native YAML in `config/`.

### Native obstacles

The native controller supports spheres, either static or moving.

All motion is evaluated by `quadrotor_mpc/core/obstacle_motion.py`, which is shared by NMPC TVPs,
MuJoCo mocap bodies, clearance metrics and viewer overlays. This prevents the
controller from predicting a different obstacle path than the plant uses.

Static sphere:

```yaml
- name: static_gate
  type: static
  position: [1.45, 1.05, 2.00]
  radius: 0.40
```

Constant velocity:

```yaml
- name: crossing
  type: dynamic
  radius: 0.24
  motion:
    type: constant_velocity
    initial_position: [1.85, -0.80, 1.80]
    velocity: [0.0, 0.38, 0.0]
```

Three-axis sinusoid:

```yaml
- name: oscillating
  type: dynamic
  radius: 0.20
  motion:
    type: sinusoidal
    center: [2.35, 1.55, 2.10]
    amplitude: [0.0, 0.0, 0.45]
    period_s: 4.5
    phase_rad: 0.0
```

Piecewise-linear waypoints:

```yaml
- name: patrol
  type: dynamic
  radius: 0.20
  motion:
    type: waypoints
    repeat: true
    points:
      - {time_s: 0.0, position: [1.0, -0.5, 1.5]}
      - {time_s: 2.0, position: [1.0,  0.5, 1.5]}
      - {time_s: 4.0, position: [2.0,  0.5, 2.0]}
```

The older `amp`/`period` y-axis sinusoid remains accepted for backward
compatibility and is normalized at configuration load time.

## ODE scenarios (`config/scenarios/`)

| Scenario | Purpose |
|---|---|
| `hover` | equilibrium, serialization and regression smoke test |
| `point_to_point` | tracking with noise and model mismatch |
| `static_obstacle` | paired deterministic versus CC-MPC example |
| `moving_obstacle` | constant-velocity prediction and growing obstacle covariance |
| `rotated_ellipsoid` | rotated geometry and non-axis-aligned clearance |
| `high_noise` | stress covariance tightening and estimator robustness |

Create new ODE scenarios in the Streamlit dashboard (**Scenario Builder**) or
copy a YAML file. Every ODE scenario must specify a 9-element start state, a
3-element goal and a positive plant timestep. Use explicit seeds in published
results.
