# Results history

> **In short:** measured values from earlier development stages and the
> committed 30-seed campaign, kept for traceability. Use them to understand
> trends, not as numbers for the current code.
>
> **Related:** [claims and statistics](claims-and-statistics.md),
> [test strategy](test-strategy.md), raw evidence in
> [`validation/stage8_30_seed/`](../../validation/stage8_30_seed/)

> [!IMPORTANT]
> Every number on this page predates the October 2026 corrections:
> constraints on the controllable nodes `1..N`, post-solve re-projection and
> risk certificate, enforced deadline fallback, the one-sided Clopper–Pearson
> collision gate and the worst-trial timing gate. A new 50-seed campaign on a
> clean commit is required before quoting results for the current code.

## Stage 1 · controller-interface regression (dynamic scenario)

Reference for the deterministic baseline after the belief-interface
refactor:

| Check | Result |
|---|---:|
| Completed steps | 200 / 200 |
| Final goal error | 0.00013550 m |
| Minimum clearance | 0.29991421 m |
| Collision | false |
| NaN state | false |

## Stage 4–5 · paired 10-second run, estimator seed 7

Deterministic, individual (ε = 0.05) and joint (ε_total = 0.10) controllers
on the same 10-second scenario:

| Controller | Final error | Minimum clearance | Maximum slack | Status counts |
|---|---:|---:|---:|---:|
| Deterministic dynamic baseline | 0.000135502 m | 0.299914211 m | 0 | `DISABLED` |
| Stage 4 individual, epsilon 0.05 | 0.076558310 m | 0.317015107 m | 0.064665434 m | 161 safe / 39 slack |
| Stage 5 joint uniform, total 0.10 | 0.104444105 m | 0.410519817 m | 0.075120639 m | 114 safe / 86 slack |

For the joint run, every one of 200 solves allocated 63 constraints, the
maximum absolute budget-sum error was \(5.55\times10^{-17}\), and no collision,
NaN state or NaN control occurred. The positive slack prevents a probabilistic
safety guarantee despite correct risk accounting.

## Stage 6 · supervisor run

Verified 10-second Stage 6 run, estimator seed 7, joint uniform budget 0.10 and
100 ms configured solve deadline:

| Metric | Result |
|---|---:|
| Completed ticks | 200 / 200 |
| Primary solver success | 200 / 200 |
| Primary commands applied | 200 |
| Nominal deadline rejects / fallback | 0 / 0 |
| Minimum clearance | 0.410519826 m |
| Maximum slack | 0.075120640 m |
| Guarantee-eligible ticks | 114 |
| Positive-slack, not-guaranteed ticks | 86 |
| Maximum primal / dual solver residual | 9.12e-7 / 6.92e-7 |
| Mean / p95 / p99 solve time | 60.63 / 73.62 / 81.60 ms |
| Maximum solve time | 88.42 ms |
| Collision / NaN | no / no |

The supervisor-disabled Stage 5 regression remains exactly:
final error 0.104444105 m, minimum clearance 0.410519817 m, 114
`SOLVED_SAFE` and 86 `SOLVED_WITH_SLACK`.

The current native NMPC does **not** pass a strict 50 ms real-time gate on the
validation host because p99 solve time is 81.60 ms. Stage 6 validates the
deadline-miss response; it does not claim that the solver is fast enough for a
20 Hz deployment.

## Stage 7 · paired regression against Stage 6

These numbers were measured under the former 100 ms acceptance deadline;
v2.0.1 set the deadline equal to the 50 ms control period.

Paired control-path regression against Stage 6 commit `5256c82` in the same
Python/dependency environment:

| Configuration | Stage 6 | Stage 7 | Difference |
|---|---:|---:|---:|
| Deterministic final error | 0.000101204527 m | 0.000101204527 m | 0 |
| Deterministic clearance | 0.299927809264 m | 0.299927809264 m | 0 |
| CC-MPC final error | 0.104444105413 m | 0.104444105413 m | 0 |
| CC-MPC clearance | 0.410519825565 m | 0.410519825565 m | 0 |
| CC-MPC maximum slack | 0.075120639653 m | 0.075120639653 m | 0 |

Both CC-MPC runs produced 114 guarantee-eligible ticks, 86 positive-slack
ticks, zero fallback activations, no collision and no NaN. This paired check
isolates Stage 7 from solver-library version drift and confirms the presentation
integration does not change an applied command.

## Stage 8 · 30-seed native campaign

Paired seeds 1000–1029, 270 episodes. Report figure:
[`report.png`](../../validation/stage8_30_seed/report.png).

| Covariance | Controller | Success | Collision | Mean min clearance | Mean max slack | Fallback episodes |
|---|---|---:|---:|---:|---:|---:|
| `0.25 Sigma` | deterministic | 30/30 | 0/30 | 0.305261 m | 0 | 0/30 |
| `0.25 Sigma` | individual | 30/30 | 0/30 | 0.326352 m | 0.062461 m | 16/30 |
| `0.25 Sigma` | joint | 30/30 | 0/30 | 0.343861 m | 0.056917 m | 14/30 |
| `Sigma` | deterministic | 30/30 | 0/30 | 0.313791 m | 0 | 2/30 |
| `Sigma` | individual | 30/30 | 0/30 | 0.356432 m | 0.061792 m | 19/30 |
| `Sigma` | joint | 30/30 | 0/30 | 0.414314 m | 0.070586 m | 17/30 |
| `4 Sigma` | deterministic | 30/30 | 0/30 | 0.323614 m | 0 | 4/30 |
| `4 Sigma` | individual | 30/30 | 0/30 | 0.467242 m | 0.079857 m | 22/30 |
| `4 Sigma` | joint | 0/30 | 0/30 | 0.660826 m | 0.139326 m | 30/30 |

All 270 episodes completed without collision, NaN or risk-budget failure. The
joint-minus-deterministic mean clearance deltas were `+0.038600 m`,
`+0.100523 m` and `+0.337212 m` as covariance increased. At `4 Sigma`, joint
control traded that clearance for a mean final-error increase of `0.338990 m`;
every episode missed the configured goal threshold.

The campaign status is `VALIDATED_WITH_LIMITATIONS`. Under the gate rules of
that release, zero collisions in 30 trials gave a two-sided Wilson 95% upper
bound of `0.113513`, above the empirical gate `0.10`. (The current default,
fixed before any new campaign, is the one-sided exact Clopper–Pearson bound,
`1 - 0.05^(1/30) = 0.0950` for the same counts; the historical campaign is not
re-graded.) Every chance-controller cell also contained positive slack and
fallback. Median per-trial solver p99 ranged from `87.523 ms` to `95.470 ms`
for the chance controllers, so the 50 ms real-time gate failed.
