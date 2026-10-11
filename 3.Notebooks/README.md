# Notebooks

A guided, runnable tour of the controller, from the 9-state model to Monte Carlo
statistics. Every notebook imports the `quadrotor_mpc` package from `2.Code`,
so what you plot is exactly what the simulator runs; nothing is re-implemented.

## Run them

```bash
cd 2.Code
uv sync --all-extras --locked        # includes the "notebooks" extra (JupyterLab)
uv run jupyter lab ../3.Notebooks
```

Each notebook starts with `from nbsetup import *`, which checks the package,
defines `CODE`, `CONFIG` and `OUTPUT` (recordings go to `3.Notebooks/outputs/`,
ignored by Git) and applies the shared plot style. Set
`QMPC_NOTEBOOK_FAST=1` to shorten the long runs.

## Learning track

| # | Notebook | What you learn | Runtime |
|---|---|---|---|
| 01 | [ODE dynamics](01_ode_dynamics.ipynb) | the 9-state model, hover equilibrium, drag-limited forward speed | seconds |
| 02 | [Linearization](02_ode_linearization.ipynb) | $A_k, B_k, C_k$, the second-order input term, linearization vs discretization error | seconds |
| 03 | [MPC and iterative MPC](03_ode_mpc_and_impc.ipynb) | one CC-MPC plan around an obstacle, iMPC convergence | ~15 s |
| 04 | [Chance constraints](04_chance_constraints.ipynb) | the Gaussian lemma, $\Phi^{-1}$ margins, sampled check of the residual, $\Omega^{1/2}$ | seconds |
| 05 | [ODE controller comparison](05_ode_controller_comparison.ipynb) | deterministic MPC collides where CC-MPC keeps a margin; effect of δ | ~1 min |
| 06 | [Native estimation](06_native_estimation.ipynb) | ESEKF estimate vs truth with 3σ bands, obstacle tracking | ~30 s |
| 07 | [Native uncertainty and risk](07_native_uncertainty_and_risk.ipynb) | covariance horizon, tightened radii, joint budget, risk certificate | ~40 s |
| 08 | [Native controller comparison](08_native_controller_comparison.ipynb) | deterministic vs individual vs joint risk: paths, clearance, timing | ~1–2 min |
| 09 | [Monte Carlo statistics](09_monte_carlo_statistics.ipynb) | Clopper–Pearson vs Wilson, paired McNemar, reading a campaign | seconds |

Notebooks 01–05 use the 9-state ODE track, 06–09 the 13-state native MuJoCo
track. Concepts are explained in the [documentation](../2.Code/docs/README.md)
and derived in the [theory chapters](../1.Docs/Theory/README.md); each notebook
links to the relevant pages.

Numbers in the saved outputs come from one machine and one seed. Solve times in
particular depend on the host. Notebook 09 analyses the committed **historical**
Stage 8 campaign, which predates the October 2026 corrections.

## Paper concepts (not implemented)

[`extras/`](extras/) keeps standalone illustrations of parts of the source
papers that the package does **not** implement: depth-camera obstacle
detection, multi-robot coordination, field-of-view constraints and the
detection formula checks. Each one says so at the top.
