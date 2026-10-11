# Validation Evidence

`stage8_30_seed/` is the original Phase 8 campaign committed with version
2.0.0. Its manifest reports source commit `9f4a1b3`, which predates the Phase 8
implementation commit. The numerical evidence is retained unchanged for
auditability, but it is not treated as release-provenance-complete evidence.

Version 2.0.1 fixes this class of error:

- a release campaign refuses a dirty Git tree by default;
- manifests record the commit, branch, clean state and a deterministic
  validation-source SHA-256;
- resume is rejected when the source snapshot changes;
- an explicitly allowed dirty-source smoke campaign is labeled
  `DIRTY_GIT_SNAPSHOT` and `FAIL_DIRTY_SOURCE`.

The 50-seed/cell release campaign must be generated after the v2.0.1 source is
committed. This sequencing prevents a report from naming a commit that does
not contain the code that generated it.

## Reading the historical artifact

- `stage8_30_seed/report.md` is kept byte-for-byte. Its "Reproduction" line
  names the pre-2.0.1 script `run_native_monte_carlo.py`; the equivalent
  command today is `quadrotor-mpc-monte-carlo --config config/native_monte_carlo.yaml`.
- Its gates were computed with the rules of that release (two-sided Wilson
  collision bound, `real_time_p99` = p95 across trials of per-trial p99,
  chance constraints applied to horizon nodes 0..N-1). The current code uses a
  one-sided Clopper–Pearson bound, a worst-trial timing gate and constraints on
  the controllable nodes 1..N (see `docs/NATIVE_MONTE_CARLO_VALIDATION.md`), so
  the historical aggregate must not be compared gate-for-gate with new runs.
