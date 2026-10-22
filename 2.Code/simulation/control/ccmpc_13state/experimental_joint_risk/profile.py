"""Read the solved allocation without repairing or renormalizing failures."""

from dataclasses import replace

import numpy as np
from scipy.special import ndtri


def extract_allocation(mpc, horizon_steps, obstacle_count):
    result = np.zeros((horizon_steps + 1, obstacle_count))
    for k in range(horizon_steps):
        for j in range(obstacle_count):
            result[k + 1, j] = float(
                mpc.opt_x_num_unscaled["_u", k, 0, f"delta_{j}"]
            )
    return result


def allocation_profile(profile, allocation, options):
    delta = np.asarray(allocation, dtype=float)
    if delta.shape != profile.risk_allocations.shape or not np.all(np.isfinite(delta)):
        raise RuntimeError("joint optimizer returned an unreadable allocation")
    if np.any(delta[1:] <= 0) or np.any(delta[1:] >= 0.5):
        raise RuntimeError("joint optimizer returned risk outside the quantile domain")
    total = float(np.sum(delta[1:]))
    valid = bool(
        np.all(delta[1:] >= options.delta_min - options.feasibility_tolerance)
        and np.all(delta[1:] <= options.delta_max + options.feasibility_tolerance)
        and total <= options.total_epsilon + options.feasibility_tolerance
        and np.all(delta[0] == 0)
    )
    beta = np.zeros_like(delta)
    beta[1:] = -ndtri(delta[1:])
    tightening = beta * profile.projected_sigmas_m
    base = profile.safety_radii_m - profile.tightenings_m
    return replace(
        profile, risk_allocations=delta, gaussian_quantiles=beta,
        tightenings_m=tightening, safety_radii_m=base + tightening,
        risk_allocation_method="joint_optimized", allocated_epsilon=total,
        remaining_epsilon=max(0.0, options.total_epsilon - total),
        budget_status="BUDGET_OK" if valid else "JOINT_ALLOCATION_INVALID",
    ), valid
