# Documentation

Start here. The pages are grouped by what you want to do, and the concept pages
are numbered in the order the data flows through the controller.

```text
docs/
├── getting-started/   install, first run, desktop workbench
├── concepts/          how the native CC-NMPC works, one stage per page
├── guides/            task-oriented how-tos (simulate, scenarios, experiments, Monte Carlo, release)
├── reference/         exact contracts, architecture, metrics, theory-to-code map
├── validation/        test strategy, statistical claim rules, historical results
└── archive/           dated audit reports kept for traceability
```

## Reading paths

**New to the project (about one hour).**

1. [Installation](getting-started/installation.md) and [first run](getting-started/first-run.md)
2. [Overview](concepts/overview.md): the two model tracks, the closed loop and a glossary
3. [Desktop workbench](getting-started/workbench.md): run and compare without the terminal

**I want to understand the controller.** Read the concept pages in order:

| # | Page | Question it answers |
|---|---|---|
| 0 | [Overview](concepts/overview.md) | What runs, in which order, and what the key terms mean |
| 1 | [Model conventions](concepts/conventions.md) | Which state, command and frame conventions are used |
| 2 | [Sensing and estimation](concepts/estimation.md) | How noisy measurements become beliefs with covariance |
| 3 | [Covariance propagation](concepts/covariance-propagation.md) | How that uncertainty grows along the prediction horizon |
| 4 | [Chance constraints](concepts/chance-constraints.md) | How uncertainty becomes a larger keep-out radius |
| 5 | [Risk budget](concepts/risk-budget.md) | How a collision probability is split across the horizon |
| 6 | [Safety supervisor and fallback](concepts/safety-fallback.md) | When a solution is rejected and what is applied instead |

The mathematical background (dynamics, quaternions, MPC, chance constraints)
is in the [theory chapters](../../1.Docs/Theory/README.md). To see each step
run on real data, use the [notebooks](../../3.Notebooks/README.md): 01–05 cover
the 9-state track, 06–09 the native track.

**I want to run experiments.**

| Task | Guide |
|---|---|
| Fly one native episode, watch it, record and replay it | [Native simulation](guides/native-simulation.md) |
| Pick or write a scenario, including moving obstacles | [Scenarios](guides/scenarios.md) |
| Compare controllers on the 9-state ODE track, run sweeps | [ODE experiments](guides/ode-experiments.md) |
| Run a paired native Monte Carlo campaign | [Monte Carlo campaigns](guides/monte-carlo.md) |
| Interpret what a campaign may and may not claim | [Claims and statistics](validation/claims-and-statistics.md) |
| Cut a release | [Release checklist](guides/release.md) |

**I want to change the code.**

| Topic | Reference |
|---|---|
| Package layers and dependency rules | [Architecture](reference/architecture.md) |
| `reset()` / `solve()` inputs and outputs | [Controller interface](reference/controller-interface.md) |
| Qt safety panel states and colours | [Desktop safety panel](reference/desktop-safety-panel.md) |
| Metric definitions | [Metrics](reference/metrics.md) |
| Where each equation lives and which test checks it | [Theory-to-code map](reference/theory-code-map.md) |
| Old import paths before the layered package | [Package migration](reference/package-migration.md) |
| What each feature must pass before it is trusted | [Test strategy](validation/test-strategy.md) |
| Why the main modelling, safety and statistics choices were made | [Design decisions](reference/design-decisions.md) |
| What is still open, in priority order | [Roadmap](roadmap.md) |
| Setup, CI checks and rules for results | [CONTRIBUTING.md](../../CONTRIBUTING.md) |

## Status at a glance

The current release is `VALIDATED_WITH_LIMITATIONS`. The joint risk budget is
a per-solve bound, not an episode-wide guarantee, and the chance controllers do
not yet meet the 50 ms control period. See
[claims and statistics](validation/claims-and-statistics.md) before quoting
any number, and [results history](validation/results-history.md) for the
measured values and the code revision they belong to.

## Where did the old pages go?

| Old file | New location |
|---|---|
| `WORKBENCH.md` | [getting-started/workbench.md](getting-started/workbench.md) |
| `ARCHITECTURE.md` | [reference/architecture.md](reference/architecture.md) |
| `CLOSED_LOOP_FLOW.md` | merged into [concepts/overview.md](concepts/overview.md) |
| `MODEL_CONVENTIONS.md` | [concepts/conventions.md](concepts/conventions.md) |
| `NATIVE_SENSOR_ESTIMATION.md` | [concepts/estimation.md](concepts/estimation.md) |
| `HORIZON_COVARIANCE_PROPAGATION.md` | [concepts/covariance-propagation.md](concepts/covariance-propagation.md) |
| `SPHERICAL_CHANCE_CONSTRAINTS.md` | [concepts/chance-constraints.md](concepts/chance-constraints.md) |
| `RISK_BUDGET_MANAGEMENT.md` | [concepts/risk-budget.md](concepts/risk-budget.md) |
| `SAFE_SLACK_FALLBACK.md` | [concepts/safety-fallback.md](concepts/safety-fallback.md) |
| `NATIVE_MUJOCO_VIEWER.md` | [guides/native-simulation.md](guides/native-simulation.md) (obstacle schema moved to [guides/scenarios.md](guides/scenarios.md)) |
| `SCENARIOS.md` | [guides/scenarios.md](guides/scenarios.md) |
| `EXPERIMENT_PROTOCOL.md` | [guides/ode-experiments.md](guides/ode-experiments.md) and [guides/monte-carlo.md](guides/monte-carlo.md) |
| `REPRODUCIBILITY.md` | merged into [guides/ode-experiments.md](guides/ode-experiments.md) and [guides/monte-carlo.md](guides/monte-carlo.md) |
| `NATIVE_MONTE_CARLO_VALIDATION.md` | [guides/monte-carlo.md](guides/monte-carlo.md), [validation/claims-and-statistics.md](validation/claims-and-statistics.md) |
| `RELEASE_2_0_1.md` | [guides/release.md](guides/release.md) |
| `CONTROLLER_INTERFACE.md` | [reference/controller-interface.md](reference/controller-interface.md) |
| `DESKTOP_SAFETY_INTERFACE.md` | [reference/desktop-safety-panel.md](reference/desktop-safety-panel.md) |
| `METRICS.md` | [reference/metrics.md](reference/metrics.md) |
| `THEORY_CODE_MAPPING.md` | [reference/theory-code-map.md](reference/theory-code-map.md) |
| `LAYERED_PACKAGE_MIGRATION.md` | [reference/package-migration.md](reference/package-migration.md) |
| `VALIDATION.md` | [validation/test-strategy.md](validation/test-strategy.md) and [validation/results-history.md](validation/results-history.md) |
| `QT_PANEL_A11Y_AUDIT.md` | [archive/2026-08-qt-panel-a11y-audit.md](archive/2026-08-qt-panel-a11y-audit.md) |
| `UI_CSS_PROBE.md` | [archive/streamlit-css-probe.md](archive/streamlit-css-probe.md) |
