# 1 · Model conventions

> **In short:** the state vectors, commands, frames and units used by both
> tracks. Check here before comparing numbers between them.
>
> **Before this:** [overview](overview.md) · **Next:**
> [sensing and estimation](estimation.md).

## Frames and units

- World frame: right-handed Cartesian, altitude positive upward.
- Angles are radians in code and configuration unless a key ends in `_deg`.
- Covariances use the same component order as the state they describe.

## 9-state ODE track

State and command:

$$x=[p_x,p_y,p_z,v_x,v_y,v_z,\phi,\theta,\psi]$$

$$u=[\phi_c,\theta_c,v_{z,c},\dot\psi_c]$$

- Ellipsoid `size` is the full axis length used by the source formulation; the
  MAV collision radius is added when the collision matrix is built.
- A chance residual greater than or equal to zero is locally safe; a negative
  value means the softened probabilistic constraint is violated.

## 13-state native track

| Quantity | Components | Size |
|---|---|---:|
| Nominal state | position, velocity (world), quaternion `q_wxyz`, body rates `ω` | 13 |
| Error state (covariance) | `δp, δv, δθ, δω` | 12 |
| Obstacle state | position, velocity | 6 |
| Command | thrust deviation from hover, `τx, τy, τz` | 4 |

The attitude error is right-multiplicative, `q_true = q_nominal ⊗ Exp(δθ)`,
so the covariance is 12×12 and never carries a redundant quaternion component.

The commands of the two tracks are different physical quantities. An adapter
is required before comparing controls between them.
