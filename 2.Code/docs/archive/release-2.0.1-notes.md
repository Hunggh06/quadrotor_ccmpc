# Release 2.0.1 stabilization notes

> Archived release notes. The current procedure is the
> [release checklist](../guides/release.md); later changes are in
> [`CHANGELOG.md`](../../CHANGELOG.md).

## Release statement

Version 2.0.1 closes the planned Phase 1–8 implementation as a
feature-complete research workbench. It does not extend the controller
algorithm or upgrade the scientific claim. The current claim remains:

> Empirically validated with documented limitations; not production-ready,
> not proven episode-wide probabilistically safe, and not 20 Hz real-time.

## Stabilization changes

- Root documentation now describes measured repository behavior rather than
  copying performance claims from the source papers.
- Broken scenario/notebook commands were replaced with files that exist.
- The CC-MPC command-acceptance deadline is `50 ms`, equal to the controller
  period. Configuration validation rejects a longer deadline when late
  solutions are configured for rejection.
- GitHub Actions runs lint, the full Python test suite, all native config
  checks, a real Qt panel lifecycle smoke test, distribution build and an
  installed-wheel smoke test.
- `uv.lock` freezes the complete cross-platform dependency resolution.
- The wheel installs native top-level modules, six console scripts, native YAML
  configurations and the licensed Crazyflie MuJoCo assets.
- Root and wheel distributions include the MIT license.
- Native Monte Carlo manifests record clean/dirty source state and an exact
  validation-source SHA-256. Release campaigns refuse a dirty Git tree by
  default and verify the snapshot again on resume.
