# Desktop workbench

`quadrotor-mpc-app` is a single dark Qt window that replaces the common
terminal workflow for the native NMPC track.

```bash
cd 2.Code
uv sync --all-extras --locked   # once
uv run quadrotor-mpc-app        # or double-click "Quadrotor MPC Workbench.bat"
```

`--workspace <dir>` points the app at another `2.Code` checkout.

## Pages

| Page | Shortcut | What it does |
|---|---|---|
| Simulate | Ctrl+1 | Pick a scenario YAML, controller (deterministic / individual / joint risk), noise scale, seed and duration, then **Run** (Ctrl+R). The episode runs headless in a separate process; trajectory, moving obstacles, clearance, solve time, slack and achieved risk stream live. **Open in Results** jumps to the recorded run. |
| Monte Carlo | Ctrl+2 | Edit the main protocol fields (trials, seeds, controllers, noise scales, gates), **Start**, **Stop** or **Resume** an interrupted campaign. A live table shows each trial's outcome. |
| Results | Ctrl+3 | Browse recorded simulations and campaigns. Select up to three simulations to overlay trajectories and signals; select a campaign to see its gates, collision and timing charts and the generated report. |

Jobs run as `python -m quadrotor_mpc.interfaces.cli.native --headless
--progress-json` and `python -m quadrotor_mpc.interfaces.cli.monte_carlo`, so a
crash or a long solve never freezes the window. Closing the window stops any
running job. The status bar shows the git state; campaigns started from a dirty
tree require the **Allow uncommitted source** box (`--allow-dirty-source`).

## Headless CLI

The same mode is available without the GUI:

```bash
uv run quadrotor-mpc-native --config config/mujoco_native_ccmpc.yaml \
  --headless --mode joint_uniform --noise-scale 1.0 --seed 7 --duration 8
```

`--progress-json` adds machine-readable `@@progress {...}` lines (one per
control step) and a final `@@result {...}` line with the outcome and recording
directory.

## Visual conventions

Controllers keep fixed colours everywhere (blue deterministic, orange
individual, aqua joint), validated for colour-vision deficiency on the dark
surface. Status (pass, warning, fail) always pairs colour with an icon and a
text label.
