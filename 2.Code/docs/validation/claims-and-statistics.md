# Claims and statistics

> **In short:** what a Monte Carlo campaign is allowed to claim, which gates
> decide it, and why a per-horizon risk budget is never an episode-wide
> guarantee. Read this before quoting any number.
>
> **Related:** [Monte Carlo campaigns](../guides/monte-carlo.md),
> [risk budget](../concepts/risk-budget.md),
> [results history](results-history.md)

## Five things that must not be confused

1. **Geometric collision**: observed with MuJoCo truth.
2. **Positive slack**: a horizon cell exceeded its allocated share at the
   optimized trajectory. Whether that blocks the claim is decided by the
   post-solve [risk certificate](../concepts/chance-constraints.md#post-solve-risk-certificate).
3. **Fallback**: the supervisor rejected the primary NMPC command.
4. **`BUDGET_OK`**: the risk allocation of the current prediction horizon sums
   correctly, nothing more.
5. **A finite Monte Carlo confidence interval**: empirical evidence, not
   proof.

The joint budget is a union bound over one prediction horizon. Receding-horizon
replanning creates overlapping, state-dependent events, so it is never reported
as an episode-wide collision probability.

## Verdicts

Every controller/noise cell gets two independent labels in `aggregate.json`.

**Claim status** (chance controllers): may the cell support a finite-sample
safety claim?

| Claim status | Meaning |
|---|---|
| `EMPIRICALLY_SUPPORTED_NOT_PROVEN` | no blocker below applies |
| `BLOCKED_<reason>` | the first blocker found, e.g. `BLOCKED_INSUFFICIENT_TRIALS` |
| `NOT_APPLICABLE_DETERMINISTIC` | deterministic baseline; no claim is made |

Blockers, in the order they are checked:

| Blocker | Trigger |
|---|---|
| `INSUFFICIENT_TRIALS` | fewer trials than `statistics.minimum_trials_for_claim` (30) |
| `NUMERICAL_OR_INCOMPLETE` | any incomplete episode or NaN |
| `COLLISION_CONFIDENCE_BOUND` | collision upper bound above `validation.empirical_collision_rate_limit` (0.10) |
| `RISK_BUDGET_FAILURE` | any risk-budget accounting failure |
| `RISK_CERTIFICATE_UNAVAILABLE` / `_EXCEEDED` | a tick without a certificate, or with `EXCEEDED` (rule `risk_certificate`) |
| `POSITIVE_SLACK` | any positive slack (legacy rule `zero_positive_slack` only) |
| `FALLBACK` | any fallback episode |

**Validation status**: how the cell as a whole came out.

| Validation status | Meaning |
|---|---|
| `PASS` | every gate passed, including a real-time pass under the `realtime_qualification` protocol |
| `VALIDATED_WITH_LIMITATIONS` | complete and numerically sound, but a claim is blocked, the collision bound or timing gate failed, or the protocol cannot qualify real time |
| `FAIL` | incomplete or numerical failures, or a risk-budget failure |

Under the default `algorithmic_comparison` protocol the best reachable
validation status is therefore `VALIDATED_WITH_LIMITATIONS`, even when the
claim status is `EMPIRICALLY_SUPPORTED_NOT_PROVEN`.

Even `EMPIRICALLY_SUPPORTED_NOT_PROVEN` is not a proof of Gaussian calibration
or of an episode-wide collision guarantee. Before claiming a chance-constraint
probability, also check the Gaussian assumptions, covariance calibration and
obstacle prediction error.

## Statistical rules

These rules were fixed before the next campaign and must not be changed after
seeing its results.

**Collision bound.** The gate uses the one-sided exact Clopper–Pearson upper
bound (`statistics.collision_bound_method: clopper_pearson_one_sided`): it is
conservative for every true rate, and the safety question is one-sided. With
zero collisions the bound is `1 - (1 - c)^(1/n)`:

| Trials | Upper bound with 0 collisions (95%) |
|---:|---:|
| 30 | 0.0950 |
| 50 | 0.0582 |

`wilson_two_sided` stays selectable to re-grade legacy artifacts, and the
protocol fingerprint records the choice. Two-sided Wilson 95% intervals are
reported for every rate (success, collision, incomplete, numerical failure,
positive slack, fallback, guarantee eligibility, budget failure) as
description only.

**Paired comparisons.** Each chance controller is compared with the
deterministic controller on the same seeds: exact McNemar tests on discordant
seeds for collision and success, Holm step-down adjusted over all
(noise level, controller) cells. Paired seed deltas are also kept for
tracking, clearance, effort, timing, slack and fallback.

**Timing.** `validation.timing_percentile` selects the per-trial statistic
(p99 by default). The gate requires the *worst* trial to fit within the
control period and fails if any trial lacks timing samples. Aggregate timing
summaries are named `per_trial_<stat>` with mean, median and max across trials.
A real-time pass additionally requires the `realtime_qualification` protocol.

**Chance claim rule.** `validation.chance_claim_rule: risk_certificate`
(default) requires every tick of every chance episode to carry a `CERTIFIED`
risk certificate. `zero_positive_slack` restores the stricter legacy rule,
under which any positive slack blocks the claim.

**Guarantee eligibility.** `HORIZON_GUARANTEE_ELIGIBLE` requires every
completed tick to pass the full gate set: joint semantics, valid budget,
primary solver success, available in-tolerance residuals, slack within
tolerance, no deadline miss and no fallback.
