# Contributing

Thanks for working on the quadrotor MPC workbench. This page covers the setup,
the checks CI runs, and the rules that keep results trustworthy. All commands
run from `2.Code/` unless stated otherwise.

## Setup

```bash
git clone https://github.com/whoisLePhuc/quadrotor_mpc.git
cd quadrotor_mpc/2.Code
uv sync --all-extras --locked
```

Details, pip installation and OS packages are in
[installation](2.Code/docs/getting-started/installation.md). If you have never
run the project, do the [first run](2.Code/docs/getting-started/first-run.md)
before changing code.

## Before you push

CI (`.github/workflows/ci.yml`) runs these on Python 3.10 and 3.12. A failing
step skips every step after it, so run them locally first:

```bash
uv run ruff check .
uv run ruff format --check .          # fix with: uv run ruff format .
uv run pytest
uv run python scripts/release_check.py
```

If you changed notebooks or code they import, also run them (about five
minutes; CI does this on Python 3.12):

```bash
QMPC_RUN_NOTEBOOKS=1 uv run pytest tests/test_notebooks.py
```

If you change dependencies in `pyproject.toml`, run `uv lock` and commit
`uv.lock`. CI installs with `--locked` and fails on a stale lock.

## Where code goes

Read the [architecture](2.Code/docs/reference/architecture.md) first. The
short version:

- Inner layers never import outer ones (`core` → `control`/`estimation` →
  `infrastructure` → `application` → `interfaces`).
  `tests/test_architecture.py` enforces this.
- New research features for the native track go into `control/nmpc/`, which
  is the only pipeline that produces Monte Carlo evidence. Do not duplicate
  them into `control/ccmpc/`.
- Controllers talk to the runtime only through `reset()` / `solve()`
  ([controller interface](2.Code/docs/reference/controller-interface.md)).
  A controller must never read MuJoCo truth.
- Every new stage must be switchable in YAML. With chance constraints and the
  supervisor disabled, the deterministic baseline must stay unchanged.

Before changing a modelling, safety or statistics choice, read
[design decisions](2.Code/docs/reference/design-decisions.md): it explains
why things are the way they are and what was already rejected.

## Rules for results

These matter more than code style:

- **Never hide slack, fallback or deadline misses.** Positive slack is always
  recorded and must never be shown as guarantee-eligible.
- **Do not change the statistical gates after seeing a campaign's results.**
  The rules in [claims and statistics](2.Code/docs/validation/claims-and-statistics.md)
  are fixed before a campaign is run.
- **Release evidence comes from a clean commit.** `quadrotor-mpc-monte-carlo`
  refuses release campaigns on a dirty tree; `--allow-dirty-source` is for
  smoke runs only.
- **Say which code a number belongs to.** When you quote a result, link the
  campaign folder or commit. Numbers from older code go in
  [results history](2.Code/docs/validation/results-history.md), marked as
  historical.
- **Zero observed collisions is not a guarantee.** Use the claim wording in
  the claims page.

## Tests

Add a test with every behaviour change. The
[test strategy](2.Code/docs/validation/test-strategy.md) lists what each stage
must pass. Bug fixes should come with a regression test that fails before the
fix. For example, `tests/test_nmpc_constraint_nodes.py` was added with the
fix for the node 1..N constraint bug.

## Documentation

- User-facing docs live in `2.Code/docs/` (start at
  [docs/README.md](2.Code/docs/README.md)) and are written in English.
- Update the relevant concept or guide page in the same commit as the code.
- Add an entry under the top *Unreleased* heading of
  [2.Code/CHANGELOG.md](2.Code/CHANGELOG.md).
- `1.Docs/Theory/` is background material in Vietnamese (Obsidian format).
  It is not kept in sync with the code.

## Commits and pull requests

- Work on a branch and open a pull request to `main`. CI must be green.
- Write commit messages in the imperative ("Fix …", "Add …"), with a body
  that explains why when the change is not obvious.
- Keep generated files out of Git. `outputs/` (runs, campaigns, notebook
  figures) is ignored. Commit validation evidence only under
  `2.Code/validation/`.

For releases, follow the [release checklist](2.Code/docs/guides/release.md).
