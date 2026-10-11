# First run

> **In short:** fly one episode in the desktop workbench, then try the same
> thing from the command line. No theory needed yet.
>
> **Before this:** [installation](installation.md) · **Next:**
> [overview](../concepts/overview.md).

## 1. Fly an episode in the workbench

```bash
uv run quadrotor-mpc-app
```

On Windows you can double-click `Quadrotor MPC Workbench.bat` at the
repository root instead.

1. On **Simulate**, keep the default scenario (*crossing obstacle, estimation
   + chance constraints*), choose **Joint risk** and press **Run simulation**
   (Ctrl+R).
2. Watch the top-down trajectory and the tiles: minimum clearance, solver time
   and horizon risk update live.
3. When it finishes, press **Open in Results**. Switch the controller to
   **Deterministic**, run again with the same seed, then select both runs on
   **Results** to overlay them.

The deterministic run passes closer to the obstacles; joint risk keeps a
larger margin because it accounts for estimation uncertainty. The
[workbench guide](workbench.md) explains every page.

## 2. The same run from the command line

Headless, with a recording under `outputs/native/`:

```bash
uv run quadrotor-mpc-native --config config/mujoco_native_ccmpc.yaml \
  --headless --mode joint_uniform --seed 1001
```

With the MuJoCo 3-D window and the Qt safety panel (Linux desktop):

```bash
MUJOCO_GL=glfw uv run quadrotor-mpc-native --config config/mujoco_native_ccmpc.yaml
```

Replay a recording without solving again:

```bash
MUJOCO_GL=glfw uv run quadrotor-mpc-native --replay outputs/native/<run-folder>
```

## 3. Which native configuration to use

Each file adds one layer on top of the previous one:

| Config | Obstacles | Estimation | Chance constraints | Use it to |
|---|---|---|---|---|
| `mujoco_native.yaml` | static | off (truth) | off | check the plant and viewer |
| `mujoco_native_dynamic.yaml` | static + moving | off (truth) | off | deterministic regression baseline |
| `mujoco_native_estimation.yaml` | static + moving | on | off (covariance only) | study estimation and propagated uncertainty |
| `mujoco_native_ccmpc.yaml` | static + moving | on | on (joint risk + supervisor) | the full pipeline; base of Monte Carlo |

## 4. The ODE track in one command

```bash
uv run quadrotor-mpc-run --config config/scenarios/static_obstacle.yaml --compare
```

This runs deterministic MPC and CC-MPC on the 9-state model with the same seed
and writes a report under `outputs/runs/`. See
[ODE experiments](../guides/ode-experiments.md).

## Where results go

| Command | Output folder |
|---|---|
| `quadrotor-mpc-native` (recording on) | `outputs/native/<timestamp>_<scenario>/` |
| `quadrotor-mpc-monte-carlo` | `outputs/native_monte_carlo/<run-id>/` |
| `quadrotor-mpc-run` | `outputs/runs/<run-id>/` |
| `quadrotor-mpc-sweep` | `outputs/sweeps/` |

`outputs/` is ignored by Git. The workbench **Results** page lists native runs
and campaigns from these folders.
