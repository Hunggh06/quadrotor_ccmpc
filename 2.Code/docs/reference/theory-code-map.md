# Theory-to-code map

> **In short:** where each equation of the 9-state ODE track is implemented,
> which test checks it and which output would expose a wrong implementation.
> For the native track, each [concept page](../README.md#reading-paths) names
> its code module.
>
> **Background:** [theory chapters](../../../1.Docs/Theory/README.md)

| Theory | Implementation | Test | Observable evidence |
|---|---|---|---|
| Nonlinear 9-state dynamics and RK4 | `quadrotor_mpc/control/ccmpc/dynamics.py` | `test_dynamics.py` | state telemetry |
| Linearization | `quadrotor_mpc/control/ccmpc/dynamics.py` | `test_dynamics.py` | predicted horizon |
| Covariance propagation | `quadrotor_mpc/control/ccmpc/uncertainty.py` | `test_uncertainty.py` | sigma plot |
| EKF predict/update | `quadrotor_mpc/application/simulation/estimators.py` | `test_estimators.py` | actual versus estimate |
| Rotated ellipsoid matrix root | `quadrotor_mpc/control/ccmpc/risk.py` | `test_risk.py` | clearance plot |
| Chance residual | `quadrotor_mpc/control/ccmpc/risk.py` | `test_risk.py` | residual and slack |
| Receding-horizon controller | `quadrotor_mpc/application/simulation/controllers.py` | `test_runner.py` | prediction replay |
| Paired controller comparison | `quadrotor_mpc/application/experiments/manager.py` | `test_experiments.py` | comparison CSV/report |
| Parameter sweep | `quadrotor_mpc/application/experiments/sweep.py` | `test_sweep.py` | sweep CSV/PNG |

For every new theoretical equation, add a code location, a focused test and a plot or metric that
can falsify an incorrect implementation.
