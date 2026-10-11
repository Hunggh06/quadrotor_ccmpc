---
title: 'Chương 14: Covariance Propagation'
chapter: 14
tags:
- ccmpc
- uncertainty
- ekf
- covariance
phase: control-theory
cssclass: theory-note
created: '2026-06-27'
aliases:
- Chapter 14
- Covariance Propagation
- Ch.14
---

## 14.1 Introduction

To evaluate chance constraints at each step of the prediction horizon, we need the robot's **position uncertainty covariance** $\boldsymbol{\Sigma}^k$ at each future step $k$.

**Covariance propagation** is the process of computing how uncertainty evolves over time under the nonlinear dynamics:

$$\mathbf{x}_{k+1} = \mathbf{f}(\mathbf{x}_k, \mathbf{u}_k) + \boldsymbol{\omega}_k$$

where $\boldsymbol{\omega}_k \sim \mathcal{N}(\mathbf{0}, \mathbf{W}_d^k)$ is the discrete per-step process noise.

## 14.2 Approaches to Uncertainty Propagation

There are three main approaches (from least to most expensive):

| Method | Local accuracy profile | Typical computation | Used Here? |
|--------|------------------------|---------------------|------------|
| **EKF linearization** | First-order; suitable when local linearization is adequate | Low | ✅ Yes |
| Unscented Transform (UKF) | Captures some nonlinear moment effects without Jacobians | Moderate | No |
| Polynomial Chaos Expansion | Higher-order approximation with basis/truncation choices | High | No |

The EKF approach is chosen because:
1. The prediction horizon is short (1–2 seconds)
2. Real-time performance is critical (~16 Hz control loop)
3. The short horizon makes a local linearization plausible, but its error still requires validation for the chosen maneuvers and timestep

## 14.3 EKF-Type Propagation (Eq. 19)

The state covariance evolves according to the linearized dynamics:

$$\boxed{\boldsymbol{\Gamma}^{k+1} = \mathbf{F}^k \boldsymbol{\Gamma}^k \mathbf{F}^{k\;T} + \mathbf{W}_d^k}$$

where:

- $\boldsymbol{\Gamma}^k \in \mathbb{R}^{9 \times 9}$: Full state covariance at step $k$
- $\mathbf{F}^k = \frac{\partial \mathbf{f}_d}{\partial \mathbf{x}}\big|_{\hat{\mathbf{x}}^k, \mathbf{u}^k}$: discrete state-transition Jacobian (9×9)
- $\mathbf{W}_d^k$: discrete process-noise covariance (9×9)

### Initial Covariance

$$\boldsymbol{\Gamma}^0 = \text{diag}\left([\sigma_{p0}^2]_{1\times 3}, [\sigma_{v0}^2]_{1\times 3}, [\sigma_{a0}^2]_{1\times 3}\right)$$

Illustrative project values, to be replaced by estimator output or identified noise statistics:
- $\sigma_{p0} = 0.05$ m (position uncertainty)
- $\sigma_{v0} = 0.1$ m/s (velocity uncertainty)
- $\sigma_{a0} = 0.03$ rad (attitude uncertainty)

### Process Noise

**Convention used by this implementation:** the configured process noise is a
*continuous-time* white-noise intensity (power spectral density)
$\mathbf{W}_c$, discretized per step with the first-order (Euler–Maruyama)
approximation

$$\mathbf{W}_d = \mathbf{W}_c\,\Delta t,\qquad
\mathbf{W}_c = \text{diag}\left([q_{p}^2]_{1\times 3}, [q_{v}^2]_{1\times 3}, [q_{a}^2]_{1\times 3}\right)$$

so the accumulated noise over a fixed time span does not depend on the chosen
timestep. The intensities have units m/√s, (m/s)/√s and rad/√s; the per-step
standard deviation is $q\sqrt{\Delta t}$.

Illustrative tuning values (`controller.uncertainty.process_noise_*` in
`config/controller.yaml`), not universal values from the two papers:
- $q_{p} = 0.01$ m/√s → per-step σ ≈ 2.4 mm at $\Delta t = 0.06$ s
- $q_{v} = 0.1$ (m/s)/√s → per-step σ ≈ 0.024 m/s
- $q_{a} = 0.02$ rad/√s → per-step σ ≈ 4.9 mrad

Do not multiply an already-discrete per-step covariance by $\Delta t$ again
(see Ch. 18). Code: `control/ccmpc/uncertainty.py`, `UncertaintyPropagator.propagate`.

### Position Covariance Extraction

The position covariance (3×3 block) used in chance constraints is:

$$\boldsymbol{\Sigma}^k = \boldsymbol{\Gamma}^k_{[0:3, 0:3]}$$

(the top-left 3×3 submatrix).

## 14.4 State Transition Jacobian

$\mathbf{F}^k=\partial\mathbf{f}_d/\partial\mathbf{x}$ is the Jacobian of the discrete transition. If a continuous Jacobian $\mathbf{A}_c=\partial\mathbf{f}_c/\partial\mathbf{x}$ is used, $\mathbf{F}^k\approx\mathbf{I}+\Delta t\mathbf{A}_c$ is only a first-order discretization.

For the quadrotor dynamics, this is a 9×9 matrix computed via finite differences:

```python
def jacobian_state(f, x, u, eps=1e-6):
 """Central finite difference Jacobian."""
 n = len(x)
 J = np.zeros((n, n))
 for i in range(n):
 dx = np.zeros(n)
 dx[i] = eps
 J[:, i] = (f(x + dx, u) - f(x - dx, u)) / (2 * eps)
 return J
```

The analytical form has significant structure:

$$\mathbf{A}_c = \begin{bmatrix}
\mathbf{0}_{3\times 3} & \mathbf{I}_{3\times 3} & \mathbf{0}_{3\times 3} \\
\mathbf{0}_{3\times 3} & \operatorname{diag}(-k_D,-k_D,-1/\tau_{vz}) & \frac{\partial \dot{\mathbf{v}}}{\partial \boldsymbol{\eta}} \\
\mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \frac{\partial \dot{\boldsymbol{\eta}}}{\partial \boldsymbol{\eta}}
\end{bmatrix}$$

where:
- Position row: $\dot{\mathbf{p}} = \mathbf{v}$ → Jacobian is $[\mathbf{0}, \mathbf{I}, \mathbf{0}]$
- Velocity row: $\dot{\mathbf{v}}$ depends on attitude through $\tan\phi$, $\tan\theta$
- Attitude row: $\dot{\phi} = -\phi/\tau_\phi$, etc. → diagonal Jacobian

### Key Coupling: $\frac{\partial \dot{\mathbf{v}}}{\partial \boldsymbol{\eta}}$

This is the critical coupling that makes uncertainty grow. The implemented
velocity model (`control/ccmpc/dynamics.py`) is

$$\dot{v}_x = gF_\theta\cos\psi - gF_\phi\sin\psi - k_Dv_x,\qquad
\dot{v}_y = gF_\theta\sin\psi + gF_\phi\cos\psi - k_Dv_y,$$

with tilt factors $F_\theta=\tan\theta/(\cos\theta\,A)$, $F_\phi=\tan\phi/(\cos\phi\,A)$
and $A=\sqrt{1+\tan^2\theta+\tan^2\phi}$. At level attitude
($\phi=\theta=0$, so $\partial F_\theta/\partial\theta=\partial F_\phi/\partial\phi=1$):

$$\frac{\partial \dot{v}_x}{\partial \theta} = g\cos\psi, \quad \frac{\partial \dot{v}_x}{\partial \phi} = -g\sin\psi$$
$$\frac{\partial \dot{v}_y}{\partial \theta} = g\sin\psi, \quad \frac{\partial \dot{v}_y}{\partial \phi} = +g\cos\psi$$

For $\psi = 0$ this gives $\partial\dot v_x/\partial\theta = g$, $\partial\dot v_y/\partial\phi = +g$ and
zero cross terms. (Earlier revisions printed $-g$ for $\partial\dot v_y/\partial\phi$ and
$+g\sec^2\phi\sin\psi$ for $\partial\dot v_x/\partial\phi$; both signs were inconsistent with the
code, which is verified against finite differences.)

Away from level attitude the derivatives of $F_\theta, F_\phi$ also couple roll and pitch
through $A$, so simple $\sec^2$ expressions are only approximations; the code
differentiates the full expressions.

## 14.5 Why Last-Loop Trajectory?

**Key design choice**: The covariance propagation uses the **last-loop trajectory** as the linearization point, NOT the current optimization variables.

**Reason**: If $\mathbf{F}^k$ depends on the current optimization variables $\hat{\mathbf{x}}^k$ and $\mathbf{u}^k$, then:
- $\mathbf{F}^k = \mathbf{F}^k(\hat{\mathbf{x}}^k, \mathbf{u}^k)$
- $\boldsymbol{\Gamma}^{k+1}$ depends on $\mathbf{F}^k$
- The chance constraint depends on $\boldsymbol{\Gamma}^{k+1}$
- This introduces nonlinear coupling between state decisions, covariance propagation, and chance constraints, greatly increasing optimization complexity

With last-loop propagation, $\mathbf{F}^k$ is fixed before solving the QP, keeping the problem size manageable.

**Justification and limitation**: Near a converged receding-horizon solution, consecutive trajectories may be close enough for this approximation to work well. Large disturbances, a new obstacle, or a poor warm start can invalidate that assumption, so covariance and constraint residuals must be checked after the solve.

## 14.6 Implementation

```python
class UncertaintyPropagator:
 def propagate(self, Gamma_0, x_guess, u_guess, dynamics, dt):
 """Propagate covariance over prediction horizon.
 
 Args:
 Gamma_0: Initial state covariance (9×9)
 x_guess: Last-loop state trajectory (9, N+1)
 u_guess: Last-loop control sequence (4, N)
 dynamics: Quadrotor dynamics model
 dt: Time step
 
 Returns:
 List of covariances [Gamma_0, ..., Gamma_N]
 """
 N = x_guess.shape[1] - 1
 Gamma_list = [Gamma_0.copy()]
 
 for k in range(N):
 Gamma_k = Gamma_list[-1]
 A_cont = dynamics.jacobian_state(x_guess[:, k], u_guess[:, k])
 F_k = np.eye(9) + dt * A_cont # Discrete-time Jacobian
 W_d = self.W * dt # W is the continuous intensity W_c
 Gamma_k1 = F_k @ Gamma_k @ F_k.T + W_d
 Gamma_list.append(Gamma_k1)
 
 return Gamma_list
```

## 14.7 Verification

The following properties are verified numerically:

1. **Symmetry**: $\boldsymbol{\Gamma}^k\approx\boldsymbol{\Gamma}^{kT}$ within numerical tolerance.
2. **Positive semidefiniteness**: All eigenvalues satisfy $\lambda_{\min}\geq-\epsilon$; strict positive definiteness is not required.
3. **Position submatrix validity**: $\boldsymbol{\Sigma}^k=\boldsymbol{\Gamma}^k_{[0:3,0:3]}$ is positive semidefinite.
4. **Monte Carlo consistency**: Over a locally valid horizon, sampled covariance should agree with predicted covariance within statistical tolerance.

The trace need not grow monotonically: stable dynamics can contract existing uncertainty more than process noise increases it.

```python
# Verification: symmetry and positive semidefiniteness
Gamma = 0.5 * (Gamma + Gamma.T)
assert np.allclose(Gamma, Gamma.T, atol=1e-10)
assert np.linalg.eigvalsh(Gamma).min() >= -1e-10
```

## 14.8 Obstacle Uncertainty Propagation

Obstacles use a simpler constant-velocity model (Eq. 6):

$$\begin{aligned}
\hat{\mathbf{p}}_o^{k+1} &= \hat{\mathbf{p}}_o^k + \hat{\mathbf{v}}_o^k \Delta t \\
\hat{\mathbf{v}}_o^{k+1} &= \hat{\mathbf{v}}_o^k \\
\boldsymbol{\Sigma}_o^{k+1} &= \boldsymbol{\Sigma}_o^k + \boldsymbol{\Sigma}_{o,v} \Delta t^2
\end{aligned}$$

This is the simplified position-only expression printed in the 2020 paper. Recursively applying it gives growth linear in step index $k$, despite the factor $\Delta t^2$ in each increment; it should not be described as quadratic growth in time.

For a consistent constant-velocity Gaussian state $\mathbf{z}_o=[\mathbf{p}_o^T,\mathbf{v}_o^T]^T$, prefer:

$$\mathbf{F}_{pv}=\begin{bmatrix}\mathbf{I}&\Delta t\mathbf{I}\\\mathbf{0}&\mathbf{I}\end{bmatrix},\qquad \boldsymbol{\Sigma}_{pv}^{k+1}=\mathbf{F}_{pv}\boldsymbol{\Sigma}_{pv}^{k}\mathbf{F}_{pv}^T+\mathbf{Q}_{pv}^k.$$

This retains position–velocity cross-covariance and produces the expected $(k\Delta t)^2\boldsymbol{\Sigma}_{v,0}$ term when velocity uncertainty is constant.

**Note on velocity uncertainty**: If obstacle velocity estimation is very noisy (large $\boldsymbol{\Sigma}_{o,v}$), the predicted covariance grows rapidly. In practice, $\boldsymbol{\Sigma}_{o,v}$ is bounded to prevent excessive growth.

## 14.9 VIO Drift Model

For realistic simulation, a time-correlated drift model is used:

$$\mathbf{b}_{t+1} = \mathbf{b}_t + \mathbf{w}_t, \quad \mathbf{w}_t \sim \mathcal{N}(0, \mathbf{Q}_{\text{drift}} \cdot \Delta t)$$

where:
- $\mathbf{b}_t \in \mathbb{R}^9$: Bias on state estimate
- $\mathbf{Q}_{\text{drift}}$: Per-second drift noise variances

The VIO estimate is then: $\hat{\mathbf{x}}_{\text{VIO}} = \mathbf{x}_{\text{true}} + \mathbf{b} + \boldsymbol{\nu}$

where $\boldsymbol{\nu} \sim \mathcal{N}(0, \mathbf{R}_{\text{VIO}})$ is instantaneous measurement noise.

## 14.10 Prerequisites and Related Chapters

> [!info] Prerequisites
- [[02_Quadrotor_Dynamics|Ch.2: Quadrotor Dynamics]] (Quadrotor Dynamics)
- [[08_Linearization|Ch.8: Linearization]] — For computing $\mathbf{F}^k$
- Probability theory (covariance matrices)

> [!info] Used In
- [[12_CCMPC|Ch.12: Chance-Constrained MPC]] — $\boldsymbol{\Sigma}^k$ in chance constraints
- [[13_Chance_Constraints|Ch.13: Chance Constraints]] — $\boldsymbol{\Sigma}^k$ determines constraint tightness

> [!info] See Also
- [[10_State_Space_Model|Ch.10: State-Space Model]] — Stochastic system formulation
