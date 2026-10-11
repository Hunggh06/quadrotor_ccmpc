# Release checklist

> **In short:** the steps to cut a release: automated gates, installed-wheel
> check, the 50-seed campaign on a clean commit, one manual desktop session,
> then the tag.
>
> **Related:** [test strategy](../validation/test-strategy.md),
> [Monte Carlo campaigns](monte-carlo.md) · history:
> [2.0.1 notes](../archive/release-2.0.1-notes.md)

The release claim stays: *empirically validated with documented limitations;
not production-ready, not proven episode-wide probabilistically safe, and not
20 Hz real-time*, until a campaign shows otherwise.

## 1. Automated gates

Run from `2.Code/`:

```bash
uv sync --all-extras --locked
uv run ruff check .
uv run pytest
uv run python scripts/release_check.py
uv run python scripts/smoke_native_panel.py
uv run python -m build
```

Before a new version, bump `EXPECTED_VERSION` in `scripts/release_check.py`
and the version in `pyproject.toml`, add a `## <version>` heading to
`CHANGELOG.md` (the release check requires it) and update the validation table
in the root README.

## 2. Installed wheel

Validate the installed wheel outside the source checkout:

```bash
python -m venv /tmp/quadrotor-wheel-test
/tmp/quadrotor-wheel-test/bin/python -m pip install dist/*.whl
cd /tmp
/tmp/quadrotor-wheel-test/bin/quadrotor-mpc-native --validate-config
/tmp/quadrotor-wheel-test/bin/quadrotor-mpc-monte-carlo --validate-config
```

## 3. Release campaign

Run the 50-seed campaign only after committing the intended source (see
[Monte Carlo campaigns](monte-carlo.md#source-provenance)):

```bash
uv run quadrotor-mpc-monte-carlo \
  --config config/native_monte_carlo.yaml \
  --workers 3 \
  --batch-size 5
```

The campaign has 450 episodes:

```text
50 paired seeds × 3 covariance levels × 3 controller modes
```

`CLEAN_GIT_COMMIT` or a fingerprinted, non-editable installed distribution is
release-eligible. Installed payloads receive the same deterministic snapshot
hash treatment as source checkouts. `DIRTY_GIT_SNAPSHOT` remains reproducible
through its snapshot hash, while editable distributions and unverified loose
source trees are also explicitly non-release-eligible. Native MJCF/OBJ model
files are included in the validation-source fingerprint.

## 4. Manual desktop acceptance

Automated CI verifies that the Qt safety panel can be constructed, updated,
reset and closed. Before creating the GitHub release, perform one native Linux
desktop session and check:

1. MuJoCo and the Qt panel open together.
2. Pause, single-step, Reset and Run again work.
3. Positive slack never appears as guarantee-eligible.
4. A deadline miss or fallback is visible in the cards, plots and transition
   log.
5. Closing either configured window terminates the session cleanly.
6. Replay opens without invoking the solver.

This manual check is a release acceptance test, not a new scientific
validation claim.

## 5. Tagging

After CI and the manual desktop acceptance pass on the committed release
revision:

```bash
git tag -a vX.Y.Z -m "quadrotor MPC research workbench vX.Y.Z"   # version from pyproject.toml
git push origin vX.Y.Z
```

Attach the source archive, validation report and checksum to the GitHub
Release. Tag only after the release changes have been reviewed and committed.
