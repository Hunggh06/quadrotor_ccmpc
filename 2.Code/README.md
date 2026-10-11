# Quadrotor MPC Research Workbench

A reproducible simulation, comparison and reporting environment for
deterministic MPC, chance-constrained MPC and a 13-state quaternion
NMPC/MuJoCo track.

Version `2.0.1` is a release-stabilized research workbench. The verified native
campaign is `VALIDATED_WITH_LIMITATIONS`: the joint risk budget is a per-solve
bound rather than an episode-wide guarantee, and solver timing currently blocks
a real-time claim.

**Documentation starts at [`docs/README.md`](docs/README.md).**

## Install

```bash
uv sync --all-extras --locked
```

Details, pip installation and OS packages:
[installation](docs/getting-started/installation.md).

## Commands

| Command | What it does | Guide |
|---|---|---|
| `quadrotor-mpc-app` | desktop workbench: simulate, Monte Carlo, results | [workbench](docs/getting-started/workbench.md) |
| `quadrotor-mpc-native` | one native NMPC/MuJoCo episode (3-D viewer, headless or replay) | [native simulation](docs/guides/native-simulation.md) |
| `quadrotor-mpc-monte-carlo` | paired native Monte Carlo campaign | [Monte Carlo campaigns](docs/guides/monte-carlo.md) |
| `quadrotor-mpc-run` | paired 9-state ODE experiment with report bundle | [ODE experiments](docs/guides/ode-experiments.md) |
| `quadrotor-mpc-sim` | single 9-state ODE closed loop | [ODE experiments](docs/guides/ode-experiments.md) |
| `quadrotor-mpc-sweep` | 9-state parameter sweep | [ODE experiments](docs/guides/ode-experiments.md#parameter-sweeps) |
| `quadrotor-mpc-dashboard` | Streamlit dashboard for the ODE track | [ODE experiments](docs/guides/ode-experiments.md#streamlit-dashboard) |

On Windows, double-click `Quadrotor MPC Workbench.bat` at the repository root
to open the workbench.

![Workbench comparing three controllers](../assets/readme/workbench-results-compare.png)

## First commands

```bash
uv run quadrotor-mpc-app
uv run quadrotor-mpc-native --config config/mujoco_native_ccmpc.yaml --headless --seed 1001
uv run quadrotor-mpc-run --config config/scenarios/static_obstacle.yaml --compare
uv run pytest -q
```

A guided tour is in [first run](docs/getting-started/first-run.md).

## Directory map

```text
2.Code/
├── quadrotor_mpc/      installable Python package
│   ├── core/           stable contracts and shared domain values
│   ├── control/        9-state CC-MPC and native quaternion NMPC
│   ├── estimation/     sensor simulation, ESEKF and obstacle tracking
│   ├── infrastructure/ MuJoCo plant and resource adapters
│   ├── application/    simulation, native runtime, experiments and validation
│   ├── reporting/      Plotly, Matplotlib and HTML reports
│   └── interfaces/     CLI, Streamlit, MuJoCo viewer, Qt panel and workbench
├── config/             native scenarios (top level) and ODE scenarios (scenarios/)
├── docs/               documentation, start at docs/README.md
├── models/             sourced MuJoCo MJCF and mesh assets
├── scripts/            release check and Qt smoke test
├── tests/              unit, integration and architecture tests
├── validation/         committed evidence
└── pyproject.toml      build metadata and console entry points
```
