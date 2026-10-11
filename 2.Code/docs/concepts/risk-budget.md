# 5 · Risk budget

> **In short:** decides the ε of each horizon-node/obstacle cell. *Individual*
> gives every cell the same ε; *joint* splits one total budget evenly across
> all `N × n_obs` cells, so the whole horizon is covered by a union bound.
> (Stage 5.)
>
> **Before this:** [chance constraints](chance-constraints.md) · **Next:**
> [safety supervisor and fallback](safety-fallback.md) · **Code:**
> `quadrotor_mpc/control/nmpc/risk_budget.py`

Only uniform joint allocation is implemented. Adaptive or geometry-aware
allocation, iterative risk allocation and an episode-wide probability
guarantee are out of scope.

## Semantics

### Individual mode

Each scalar constraint receives the configured value:

\[
\epsilon_{k,o}=\epsilon_{\mathrm{individual}}.
\]

This is a per-constraint statement. The sum may be larger than one, and the
mode must not be described as a joint-horizon guarantee.

### Joint uniform mode

For \(N_c\) active scalar chance constraints and configured total budget
\(\epsilon_{\mathrm{total}}\):

\[
\epsilon_{k,o}
=
\frac{\epsilon_{\mathrm{total}}}{N_c},
\qquad
\sum_{k,o}\epsilon_{k,o}
=
\epsilon_{\mathrm{total}}.
\]

The native NLP has \(N+1\) state nodes, but node 0 is the measured current
state: no control can change it, so it is not a decision constraint. The
obstacle constraints are imposed on the successor states \(x_1..x_N\) (each
do-mpc stage \(k\) constrains \(x_{k+1}=f(x_k,u_k)\)), hence

\[
N_c=N\,n_{\mathrm{obs}},\qquad \epsilon_{0,o}=0 .
\]

Earlier releases constrained \(x_0..x_{N-1}\) and split the budget over
\((N+1)n_{\mathrm{obs}}\) cells: risk was spent on the uncontrollable node 0
(whose violation could only become slack) while the terminal node \(x_N\)
was never constrained. The numerical model and the union-bound accounting
remain identical: both cover exactly nodes \(1..N\).

By Boole's inequality:

\[
\mathbb{P}\left(\bigcup_{k,o}\mathcal C_{k,o}\right)
\le
\sum_{k,o}\epsilon_{k,o}
\le
\epsilon_{\mathrm{total}}.
\]

This bound applies to the prediction horizon of one solve. Receding-horizon
replanning introduces overlapping, state-dependent events; therefore the same
number is not automatically an upper bound for collision over the complete
closed-loop episode.

## Tightening

Each allocated cell has its own one-sided Gaussian quantile:

\[
\beta_{k,o}=\Phi^{-1}(1-\epsilon_{k,o}),
\]

and its tightened radius is:

\[
r_{\mathrm{tight},k,o}
=
r_{\mathrm{base},o}
+
\beta_{k,o}\sigma_{k,o}.
\]

The risk allocator and covariance projection remain outside the nonlinear
program. The resulting epsilon, beta and radius arrays are supplied as TVPs.

## Configuration

```yaml
controller:
  chance_constraints:
    enabled: true
    type: spherical
    individual_epsilon: 0.05
    risk_budget:
      semantics: joint
      allocation: uniform
      total_epsilon: 0.10
      tolerance: 1.0e-12
```

Set `semantics: individual` for the per-cell mode. `allocation: uniform` is
the only joint allocator accepted; unsupported names fail configuration
validation instead of silently falling back.

## Runtime audit

Every `ControlSolution`, native result, telemetry sample and recording reports:

- risk semantics and allocation method;
- configured total epsilon;
- allocated and remaining epsilon;
- number of active scalar constraints;
- `BUDGET_OK`, `INDIVIDUAL_ONLY` or `DISABLED`.

Uniform joint allocation raises an error if the allocated sum exceeds the
configured total by more than `tolerance`.

`BUDGET_OK` only validates the allocation arithmetic. A probabilistic safety
claim additionally requires:

- nonpositive solver relaxation within tolerance;
- calibrated Gaussian covariance;
- correct obstacle prediction and independence assumptions;
- Monte Carlo validation with confidence intervals.

Any positive safety slack still invalidates the chance guarantee.
