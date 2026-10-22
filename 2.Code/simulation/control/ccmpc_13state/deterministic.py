"""do-mpc adapter implementing the native belief-based controller contract."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from typing import Any

import casadi as ca
import numpy as np

from simulation.control.ccmpc_13state.chance_constraints import (
    ChanceConstraintOptions,
    build_spherical_chance_profile,
    certify_achieved_risk,
    evaluate_spherical_constraints,
)
from simulation.control.ccmpc_13state.core import (
    DRONE_RADIUS,
    FIRST_CONSTRAINED_NODE,
    INPUT_NAMES,
    STATE_NAMES,
    build_controller,
    build_model,
    make_mpc_tvp_fun,
)
from simulation.control.ccmpc_13state.covariance import (
    CovariancePropagationOptions,
    HorizonCovariancePropagator,
)
from simulation.control.ccmpc_13state.residuals import (
    ResidualStatus,
    SolverResidual,
    extract_residual,
)
from simulation.core.chance_profile import (
    ENFORCED_PHASE,
    POST_SOLVE_DIAGNOSTIC_PHASE,
    ChanceProfileShapeError,
    build_chance_profile_snapshot,
)
from simulation.core.contracts import (
    ControlGoal,
    ControlSolution,
    ObstacleBelief,
    VehicleBelief,
)
from simulation.core.timing import TimingRecorder


def _constrained_mask(
    steps: int, obstacles: int, first: int = FIRST_CONSTRAINED_NODE
) -> np.ndarray:
    """Nodes the NLP actually constrains (node 0 is the fixed measured state)."""
    mask = np.ones((int(steps), int(obstacles)), dtype=bool)
    mask[: max(0, min(int(first), int(steps))), :] = False
    return mask


def _extract_nominal_states(mpc, fallback_state: np.ndarray) -> np.ndarray:
    axes: list[np.ndarray] = []
    try:
        for name in STATE_NAMES:
            values = np.asarray(mpc.data.prediction(("_x", name)), dtype=float)
            axes.append(values.reshape(-1))
    except (AttributeError, KeyError, TypeError, ValueError):
        return fallback_state.reshape(1, 13).copy()
    length = min((len(axis) for axis in axes), default=0)
    if length < 1:
        return fallback_state.reshape(1, 13).copy()
    return np.column_stack([axis[-length:] for axis in axes])


def _extract_nominal_controls(
    mpc,
    fallback_command: np.ndarray,
    horizon_length: int,
) -> np.ndarray:
    control_steps = max(int(horizon_length) - 1, 0)
    if control_steps == 0:
        return np.empty((0, 4), dtype=float)
    axes: list[np.ndarray] = []
    try:
        for name in INPUT_NAMES:
            values = np.asarray(mpc.data.prediction(("_u", name)), dtype=float)
            axes.append(values.reshape(-1))
    except (AttributeError, KeyError, TypeError, ValueError):
        return np.repeat(
            np.asarray(fallback_command, dtype=float).reshape(1, 4),
            control_steps,
            axis=0,
        )
    available = min((len(axis) for axis in axes), default=0)
    if available < 1:
        return np.repeat(
            np.asarray(fallback_command, dtype=float).reshape(1, 4),
            control_steps,
            axis=0,
        )
    extracted = np.column_stack([axis[-available:] for axis in axes])
    if available >= control_steps:
        return extracted[:control_steps].copy()
    padding = np.repeat(extracted[-1:, :], control_steps - available, axis=0)
    return np.vstack([extracted, padding])


class DeterministicNMPCController:
    """Adapter preserving the existing 13-state deterministic NMPC behavior."""

    def __init__(
        self,
        *,
        bounds: Mapping[str, float],
        obstacle_specs: Sequence[Mapping[str, Any]],
        margin: float,
        horizon_steps: int,
        timestep_s: float,
        max_iter: int = 60,
        cached=None,
        covariance_options: CovariancePropagationOptions | None = None,
        chance_options: ChanceConstraintOptions | None = None,
        clock_ns: Callable[[], int] | None = None,
        joint_risk_options: Any | None = None,
    ):
        self._obstacle_specs = tuple(dict(item) for item in obstacle_specs)
        self._margin = float(margin)
        self._horizon_steps = int(horizon_steps)
        self._timestep_s = float(timestep_s)
        self._clock_ns = clock_ns
        if self._horizon_steps < 1:
            raise ValueError("horizon_steps must be >= 1")
        if self._timestep_s <= 0.0:
            raise ValueError("timestep_s must be > 0")
        self._covariance_options = (
            CovariancePropagationOptions() if covariance_options is None else covariance_options
        )
        self._chance_options = (
            ChanceConstraintOptions() if chance_options is None else chance_options
        )
        self._joint_risk_options = joint_risk_options
        self._joint_risk_enabled = bool(
            joint_risk_options is not None
            and getattr(joint_risk_options, "enabled", True)
        )
        self._joint_formulation = str(
            getattr(joint_risk_options, "formulation", "required")
        ).strip().lower()
        if self._joint_formulation != "required":
            raise ValueError(
                "joint risk formulation must be required"
            )
        self._joint_delta_enabled = self._joint_risk_enabled
        if self._chance_options.enabled and not self._covariance_options.enabled:
            raise ValueError(
                "spherical chance constraints require covariance propagation to be enabled"
            )
        if self._joint_risk_enabled:
            if not self._chance_options.enabled:
                raise ValueError("joint risk allocation requires enabled chance constraints")
            if not self._obstacle_specs:
                raise ValueError("joint risk allocation requires at least one obstacle")
            if self._chance_options.risk_budget.semantics != "joint":
                raise ValueError("joint optimization requires joint risk semantics")
            total = float(getattr(joint_risk_options, "total_epsilon", 0.1))
            if abs(total - self._chance_options.risk_budget.total_epsilon) > 1e-12:
                raise ValueError("joint optimizer and chance-profile budgets must match")
            delta_min = float(getattr(joint_risk_options, "delta_min", 1e-4))
            cells = self._horizon_steps * len(self._obstacle_specs)
            if self._joint_delta_enabled and delta_min * cells > total + 1e-12:
                raise ValueError(
                    "joint risk allocation is infeasible: delta_min * horizon_steps * "
                    "obstacle_count exceeds total_epsilon"
                )
        self._covariance_propagator = HorizonCovariancePropagator(
            self._covariance_options,
            self._timestep_s,
        )
        self._last_nominal_states: np.ndarray | None = None
        self._last_nominal_controls: np.ndarray | None = None
        self._solve_attempt_counter = 0
        self._objective_value_fun = None
        # Latest true NLP objective (including slack and input regularisation).
        # Used by the opt-in joint-risk diagnostics.
        self.last_objective_value: float | None = None
        # Maximum slack variable actually seen by the NLP.  ControlSolution.slacks
        # is a post-solve diagnostic against re-projected radii and can differ.
        self.last_nlp_slack_max: float | None = None

        if cached is not None:
            if self._chance_options.enabled:
                raise ValueError(
                    "a deterministic cached controller cannot be reused for enabled "
                    "chance constraints"
                )
            self.model, self.mpc, self._obstacle_tvp_idx, self._goal_state = cached
            self.mpc.reset_history()
        else:
            self.model, self._obstacle_tvp_idx = build_model(
                self._timestep_s,
                self._obstacle_specs,
                joint_risk=self._joint_risk_enabled,
                joint_formulation=self._joint_formulation,
            )
            self.mpc = build_controller(
                self.model,
                self._obstacle_specs,
                self._obstacle_tvp_idx,
                bounds,
                self._margin,
                self._horizon_steps,
                self._timestep_s,
                max_iter=max_iter,
                penalty=(
                    self._chance_options.slack_penalty if self._chance_options.enabled else 1e4
                ),
                soft_obstacle_constraints=self._chance_options.soft_constraint,
                slack_penalty_mode=self._chance_options.slack_penalty_mode,
                slack_quadratic_penalty=self._chance_options.slack_quadratic_penalty,
                slack_hinge_deadband_m=self._chance_options.slack_hinge_deadband_m,
                joint_risk=self._joint_risk_enabled,
                joint_formulation=self._joint_formulation,
                joint_delta_min=getattr(joint_risk_options, "delta_min", 1e-4),
                joint_delta_max=getattr(joint_risk_options, "delta_max", 0.2),
                joint_total_epsilon=getattr(joint_risk_options, "total_epsilon", 0.1),
            )
            self._goal_state = {
                "pos": {"x": 0.0, "y": 0.0, "z": 1.0},
                "euler": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            }
            self.mpc.set_tvp_fun(
                make_mpc_tvp_fun(
                    self.mpc.get_tvp_template(),
                    self._goal_state,
                    self._obstacle_specs,
                    self._obstacle_tvp_idx,
                    self._horizon_steps,
                    self._timestep_s,
                    self._margin,
                )
            )
            self.mpc.setup()

    @property
    def horizon_steps(self) -> int:
        return self._horizon_steps

    def reset(self, belief: VehicleBelief) -> None:
        self.mpc.reset_history()
        for key in (
            "obstacle_predictions",
            "obstacle_projected_sigmas",
            "obstacle_betas",
            "obstacle_risk_allocations",
            "obstacle_safe_distances",
        ):
            self._goal_state.pop(key, None)
        self._last_nominal_states = None
        self._last_nominal_controls = None
        self.last_objective_value = None
        self.last_nlp_slack_max = None
        self.mpc.x0 = self._mpc_state(belief)
        if self._joint_delta_enabled:
            initial_risk = min(
                self._joint_risk_options.delta_max,
                self._joint_risk_options.total_epsilon
                / (self._horizon_steps * len(self._obstacle_specs)),
            )
            for i in range(len(self._obstacle_specs)):
                self.mpc.u0[f"delta_{i}"] = initial_risk
        self.mpc.set_initial_guess()
        if self._joint_risk_enabled:
            for k in range(self._horizon_steps + 1):
                initial_budget = (
                    k * len(self._obstacle_specs) * initial_risk
                    if self._joint_delta_enabled
                    else 0.0
                )
                self.mpc.opt_x_num["_x", k, 0, -1, "risk_budget_used"] = initial_budget

    def _mpc_state(self, belief: VehicleBelief) -> np.ndarray:
        if self._joint_risk_enabled:
            # Budget is per receding horizon, not an episode-wide allowance.
            return np.r_[belief.mean_state_13, 0.0].reshape(-1, 1)
        return belief.mean_state_13.reshape(-1, 1)

    def _seed_nominal(
        self,
        belief: VehicleBelief,
        goal: ControlGoal,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Shift the prior solution or construct a deterministic first-tick seed."""
        steps = self._horizon_steps + 1
        if (
            self._last_nominal_states is not None
            and self._last_nominal_states.shape == (steps, 13)
            and self._last_nominal_controls is not None
            and self._last_nominal_controls.shape == (self._horizon_steps, 4)
        ):
            states = np.vstack([self._last_nominal_states[1:], self._last_nominal_states[-1:]])
            controls = np.vstack(
                [self._last_nominal_controls[1:], self._last_nominal_controls[-1:]]
            )
            states[0] = belief.mean_state_13
            return states, controls

        states = np.repeat(belief.mean_state_13.reshape(1, 13), steps, axis=0)
        interpolation = np.linspace(0.0, 1.0, steps)[:, None]
        states[:, :3] = belief.mean_state_13[:3] + interpolation * (
            goal.position - belief.mean_state_13[:3]
        )
        controls = np.zeros((self._horizon_steps, 4), dtype=float)
        return states, controls

    def _objective_value(self) -> float | None:
        """Evaluate the assembled do-mpc objective at the current solution.

        CasADi/IPOPT does not expose ``f`` through ``solver_stats`` in this
        backend, so build a tiny function from the already-assembled NLP
        expression.  The objective includes tracking, input regularisation and
        the configured slack cost, making it the correct quantity for comparing
        two allocations at the same state.
        """
        try:
            if self._objective_value_fun is None:
                x_symbol = ca.vertcat(self.mpc._opt_x)
                p_symbol = ca.vertcat(self.mpc._opt_p)
                self._objective_value_fun = ca.Function(
                    "objective_value",
                    [x_symbol, p_symbol],
                    [self.mpc._nlp_obj],
                )
            return float(
                self._objective_value_fun(
                    self.mpc.opt_x_num.cat,
                    self.mpc.opt_p_num.cat,
                )
            )
        except (AttributeError, KeyError, RuntimeError, TypeError, ValueError):
            return None

    def _nlp_slack_max(self) -> float | None:
        """Return the maximum slack variable in the solved NLP, if exposed."""
        try:
            values = np.asarray(
                self.mpc.opt_x_num_unscaled["_eps"],
                dtype=float,
            ).reshape(-1)
            return float(np.max(values)) if values.size else 0.0
        except (AttributeError, KeyError, RuntimeError, TypeError, ValueError):
            return None

    def _validate_profile_shapes(
        self,
        seed_states: np.ndarray,
        seed_controls: np.ndarray,
        obstacle_predictions: np.ndarray,
        chance_profile,
    ) -> None:
        """Reject invalid horizon shapes before the solver is ever called."""
        expected_steps = self._horizon_steps + 1
        if seed_states.shape != (expected_steps, 13):
            raise ChanceProfileShapeError(
                f"seed_states must have shape ({expected_steps}, 13), got {seed_states.shape}"
            )
        if seed_controls.shape != (self._horizon_steps, 4):
            raise ChanceProfileShapeError(
                f"seed_controls must have shape ({self._horizon_steps}, 4), "
                f"got {seed_controls.shape}"
            )
        if obstacle_predictions.shape != (
            len(self._obstacle_specs),
            expected_steps,
            3,
        ):
            raise ChanceProfileShapeError(
                "obstacle_predictions must have shape "
                f"({len(self._obstacle_specs)}, {expected_steps}, 3), "
                f"got {obstacle_predictions.shape}"
            )
        expected_constraint_shape = (
            expected_steps,
            len(self._obstacle_specs),
        )
        if chance_profile.risk_allocations.shape != expected_constraint_shape:
            raise ChanceProfileShapeError(
                "risk_allocations must have shape "
                f"{expected_constraint_shape}, got "
                f"{chance_profile.risk_allocations.shape}"
            )
        if chance_profile.safety_radii_m.shape != expected_constraint_shape:
            raise ChanceProfileShapeError(
                "safety_radii_m must have shape "
                f"{expected_constraint_shape}, got "
                f"{chance_profile.safety_radii_m.shape}"
            )

    def solve(
        self,
        belief: VehicleBelief,
        obstacles: Sequence[ObstacleBelief],
        goal: ControlGoal,
        time_s: float,
    ) -> ControlSolution:
        if len(obstacles) != len(self._obstacle_specs):
            raise ValueError(
                f"controller expects {len(self._obstacle_specs)} obstacle beliefs, "
                f"got {len(obstacles)}"
            )
        timing_recorder = TimingRecorder(clock_ns=getattr(self, "_clock_ns", None))
        prediction_steps = self._horizon_steps + 1
        with timing_recorder.measure("seed_trajectory_time_ms"):
            if obstacles:
                obstacle_predictions = np.asarray(
                    [
                        obstacle.mean_positions(prediction_steps, self._timestep_s)
                        for obstacle in obstacles
                    ],
                    dtype=float,
                )
            else:
                obstacle_predictions = np.empty((0, prediction_steps, 3), dtype=float)

            seed_states, seed_controls = self._seed_nominal(belief, goal)
        with timing_recorder.measure("covariance_propagation_time_ms"):
            if self._covariance_options.enabled:
                (
                    tightening_vehicle_covariances,
                    tightening_obstacle_covariances,
                ) = self._covariance_propagator.propagate(
                    belief,
                    obstacles,
                    seed_states,
                    seed_controls,
                )
            else:
                tightening_vehicle_covariances = np.zeros(
                    (prediction_steps, 12, 12),
                    dtype=float,
                )
                tightening_obstacle_covariances = np.zeros(
                    (prediction_steps, len(obstacles), 6, 6),
                    dtype=float,
                )
        with timing_recorder.measure("chance_profile_time_ms"):
            base_safety_radii = np.asarray(
                [
                    obstacle.shape.bounding_radius_m + self._margin + DRONE_RADIUS
                    for obstacle in obstacles
                ],
                dtype=float,
            )
            chance_profile = build_spherical_chance_profile(
                vehicle_positions=seed_states[:, :3],
                obstacle_positions=obstacle_predictions,
                vehicle_covariances=tightening_vehicle_covariances,
                obstacle_covariances=tightening_obstacle_covariances,
                base_safety_radii_m=base_safety_radii,
                options=self._chance_options,
                timing_recorder=timing_recorder,
                first_constrained_step=FIRST_CONSTRAINED_NODE,
            )

            self._validate_profile_shapes(
                seed_states,
                seed_controls,
                obstacle_predictions,
                chance_profile,
            )

            self._solve_attempt_counter += 1
            solve_attempt_id = f"{self._solve_attempt_counter:06d}"
            obstacle_ids = tuple(
                str(getattr(obstacle, "name", "")) or f"obstacle_{index}"
                for index, obstacle in enumerate(obstacles)
            )
            node_indices = tuple(range(prediction_steps))
            enforced_profile = (
                build_chance_profile_snapshot(
                    control_tick=0,
                    solve_attempt_id=solve_attempt_id,
                    phase=ENFORCED_PHASE,
                    trajectory_source="seed_trajectory",
                    allocation_method=chance_profile.risk_allocation_method,
                    risk_semantics=chance_profile.risk_semantics,
                    node_indices=node_indices,
                    obstacle_ids=obstacle_ids,
                    epsilon=chance_profile.risk_allocations,
                    projected_sigma=chance_profile.projected_sigmas_m,
                    tightening_margin=chance_profile.tightenings_m,
                    tightened_radius=chance_profile.safety_radii_m,
                    active_mask=_constrained_mask(prediction_steps, len(obstacles)),
                    joint_budget=chance_profile.configured_total_epsilon,
                    allocated_budget=chance_profile.allocated_epsilon,
                    remaining_budget=chance_profile.remaining_epsilon,
                )
                if self._chance_options.enabled
                else None
            )

        with timing_recorder.measure("tvp_update_time_ms"):
            self._goal_state["pos"] = {
                axis: float(value) for axis, value in zip(("x", "y", "z"), goal.position)
            }
            # The TVP helper accepts a quaternion directly to avoid an unnecessary
            # quaternion -> Euler -> quaternion conversion at the interface boundary.
            self._goal_state["quaternion_wxyz"] = goal.quaternion_wxyz
            self._goal_state["obstacle_predictions"] = obstacle_predictions
            self._goal_state["obstacle_projected_sigmas"] = chance_profile.projected_sigmas_m.T
            self._goal_state["obstacle_betas"] = chance_profile.gaussian_quantiles.T
            self._goal_state["obstacle_risk_allocations"] = chance_profile.risk_allocations.T
            self._goal_state["obstacle_safe_distances"] = chance_profile.safety_radii_m.T
            if self._joint_risk_enabled:
                self._goal_state["obstacle_safe_distances"] = np.repeat(
                    base_safety_radii[:, None], prediction_steps, axis=1,
                )
            self._goal_state["prediction_time_s"] = float(time_s)

        with timing_recorder.measure("nlp_solve_time_ms"):
            command = np.asarray(
                self.mpc.make_step(self._mpc_state(belief)),
                dtype=float,
            ).reshape(-1)[:4]
        if self._joint_risk_enabled:
            from simulation.control.ccmpc_13state.experimental_joint_risk.profile import (
                allocation_profile, extract_allocation,
            )
            allocation = extract_allocation(
                self.mpc, self._horizon_steps, len(obstacles),
            )
            chance_profile, joint_bounds_ok = allocation_profile(
                chance_profile, allocation, self._joint_risk_options,
            )
            enforced_profile = build_chance_profile_snapshot(
                control_tick=0, solve_attempt_id=solve_attempt_id,
                phase=ENFORCED_PHASE, trajectory_source="seed_trajectory",
                allocation_method=chance_profile.risk_allocation_method,
                risk_semantics="joint", node_indices=node_indices,
                obstacle_ids=obstacle_ids, epsilon=chance_profile.risk_allocations,
                projected_sigma=chance_profile.projected_sigmas_m,
                tightening_margin=chance_profile.tightenings_m,
                tightened_radius=chance_profile.safety_radii_m,
                active_mask=_constrained_mask(prediction_steps, len(obstacles)),
                joint_budget=chance_profile.configured_total_epsilon,
                allocated_budget=chance_profile.allocated_epsilon,
                remaining_budget=chance_profile.remaining_epsilon,
            )
        self.last_objective_value = self._objective_value()
        self.last_nlp_slack_max = self._nlp_slack_max()
        solver_stats = getattr(self.mpc, "solver_stats", {})
        primary_solver_success = bool(solver_stats.get("success", True))
        if self._joint_risk_enabled:
            primary_solver_success = primary_solver_success and joint_bounds_ok
        primary_solver_status = str(
            solver_stats.get(
                "return_status",
                "SUCCESS" if primary_solver_success else "FAILED",
            )
        )
        solver_iterations = max(0, int(solver_stats.get("iter_count", 0)))
        iteration_history = solver_stats.get("iterations", {})

        def extract_from_history(name: str) -> SolverResidual:
            values = iteration_history.get(name, ())
            if not values:
                return SolverResidual(
                    status=ResidualStatus.UNAVAILABLE,
                    value=None,
                    source=name,
                    detail="field_not_exposed_by_backend",
                )
            return extract_residual(values, source=name)

        primal = extract_from_history("inf_pr")
        dual = extract_from_history("inf_du")
        primal_residual = primal.value
        dual_residual = dual.value
        residual_statuses = {primal.status.value, dual.status.value}
        if residual_statuses == {"UNAVAILABLE"}:
            residual_status = "UNAVAILABLE"
        elif "INVALID" in residual_statuses:
            residual_status = "INVALID"
        else:
            residual_status = "AVAILABLE"
        nominal_states = _extract_nominal_states(self.mpc, belief.mean_state_13)
        horizon_length = nominal_states.shape[0]
        nominal_controls = _extract_nominal_controls(
            self.mpc,
            command,
            horizon_length,
        )
        self._last_nominal_states = nominal_states.copy()
        self._last_nominal_controls = nominal_controls.copy()
        if (
            self._covariance_options.enabled
            and horizon_length != tightening_vehicle_covariances.shape[0]
        ):
            (
                predicted_covariances,
                predicted_obstacle_covariances,
            ) = self._covariance_propagator.propagate(
                belief,
                obstacles,
                nominal_states,
                nominal_controls,
            )
        else:
            predicted_covariances = tightening_vehicle_covariances[:horizon_length].copy()
            predicted_obstacle_covariances = tightening_obstacle_covariances[:horizon_length].copy()

        used_obstacle_predictions = obstacle_predictions
        used_sigmas = chance_profile.projected_sigmas_m
        used_risks = chance_profile.risk_allocations
        used_safety_radii = chance_profile.safety_radii_m
        used_risk_metadata = chance_profile
        evaluation_radii = used_safety_radii
        with timing_recorder.measure("post_solve_diagnostic_time_ms"):
            post_solve_diagnostic_profile = None
            resampled = horizon_length != prediction_steps
            if resampled:
                # The optimiser returned a different node count (e.g. the
                # prediction could not be read back).  Resample obstacle means
                # and the ENFORCED display fields onto the returned nodes.
                target = np.linspace(0.0, 1.0, horizon_length)
                source = np.linspace(0.0, 1.0, prediction_steps)

                def resample(values: np.ndarray) -> np.ndarray:
                    return np.column_stack(
                        [np.interp(target, source, values[:, i]) for i in range(len(obstacles))]
                    ).reshape(horizon_length, len(obstacles))

                used_obstacle_predictions = np.asarray(
                    [
                        np.column_stack(
                            [np.interp(target, source, positions[:, axis]) for axis in range(3)]
                        )
                        for positions in obstacle_predictions
                    ],
                    dtype=float,
                ).reshape(len(obstacles), horizon_length, 3)
                used_sigmas = resample(chance_profile.projected_sigmas_m)
                used_risks = resample(chance_profile.risk_allocations)
                used_safety_radii = resample(chance_profile.safety_radii_m)
                evaluation_radii = used_safety_radii
            if self._chance_options.enabled and len(obstacles):
                # Post-solve check, run on EVERY solve: the enforced radii were
                # built by projecting covariance on the SEED trajectory's
                # collision normals, while the solver moved the trajectory.
                # Re-projecting on the optimised trajectory gives the radius the
                # chance requirement actually needs at the solution; margins and
                # slacks are evaluated against it, so a seed/solution normal
                # mismatch shows up as slack instead of being hidden.  The
                # enforced fields keep describing what the solver received.
                diagnostic_first = min(FIRST_CONSTRAINED_NODE, horizon_length - 1)
                diagnostic_metadata = build_spherical_chance_profile(
                    vehicle_positions=nominal_states[:, :3],
                    obstacle_positions=used_obstacle_predictions,
                    vehicle_covariances=predicted_covariances,
                    obstacle_covariances=predicted_obstacle_covariances,
                    base_safety_radii_m=base_safety_radii,
                    options=self._chance_options,
                    first_constrained_step=diagnostic_first,
                )
                if self._joint_risk_enabled:
                    diagnostic_metadata, _ = allocation_profile(
                        diagnostic_metadata, used_risks, self._joint_risk_options,
                    )
                post_solve_diagnostic_profile = build_chance_profile_snapshot(
                    control_tick=0,
                    solve_attempt_id=solve_attempt_id,
                    phase=POST_SOLVE_DIAGNOSTIC_PHASE,
                    trajectory_source="optimized_trajectory",
                    allocation_method=diagnostic_metadata.risk_allocation_method,
                    risk_semantics=diagnostic_metadata.risk_semantics,
                    node_indices=tuple(range(horizon_length)),
                    obstacle_ids=obstacle_ids,
                    epsilon=diagnostic_metadata.risk_allocations,
                    projected_sigma=diagnostic_metadata.projected_sigmas_m,
                    tightening_margin=diagnostic_metadata.tightenings_m,
                    tightened_radius=diagnostic_metadata.safety_radii_m,
                    active_mask=_constrained_mask(
                        horizon_length, len(obstacles), first=diagnostic_first
                    ),
                    joint_budget=diagnostic_metadata.configured_total_epsilon,
                    allocated_budget=diagnostic_metadata.allocated_epsilon,
                    remaining_budget=diagnostic_metadata.remaining_epsilon,
                )
                evaluation_radii = diagnostic_metadata.safety_radii_m
            # Row 0 is the measured current state: its radius is the untightened
            # base radius (zero risk), so a positive slack there means the
            # vehicle is already inside the deterministic safety distance.
            margins, slacks = evaluate_spherical_constraints(
                vehicle_positions=nominal_states[:, :3],
                obstacle_positions=used_obstacle_predictions,
                safety_radii_m=evaluation_radii,
                distance_smoothing_m2=self._chance_options.distance_smoothing_m2,
            )
            certificate = None
            if self._chance_options.enabled:
                certificate = certify_achieved_risk(
                    vehicle_positions=nominal_states[:, :3],
                    obstacle_positions=used_obstacle_predictions,
                    base_safety_radii_m=base_safety_radii,
                    projected_sigmas_m=(
                        diagnostic_metadata.projected_sigmas_m
                        if post_solve_diagnostic_profile is not None
                        else np.zeros((horizon_length, len(obstacles)))
                    ),
                    options=self._chance_options,
                    first_constrained_step=min(FIRST_CONSTRAINED_NODE, horizon_length - 1),
                    distance_smoothing_m2=self._chance_options.distance_smoothing_m2,
                    tolerance=self._chance_options.risk_budget.tolerance,
                )
        if self._chance_options.enabled:
            solver_status = (
                "SOLVED_SAFE"
                if not slacks.size
                or float(np.max(slacks)) <= self._chance_options.slack_tolerance_m
                else "SOLVED_WITH_SLACK"
            )
        else:
            solver_status = "SOLVED_DETERMINISTIC"

        return ControlSolution(
            command=command,
            nominal_states=nominal_states,
            predicted_covariances=predicted_covariances,
            chance_margins=margins,
            risk_allocations=used_risks,
            slacks=slacks,
            solver_status=solver_status,
            predicted_obstacle_covariances=predicted_obstacle_covariances,
            projected_uncertainties=used_sigmas,
            tightened_safety_radii=used_safety_radii,
            risk_semantics=used_risk_metadata.risk_semantics,
            risk_allocation_method=used_risk_metadata.risk_allocation_method,
            risk_budget_total=used_risk_metadata.configured_total_epsilon,
            risk_budget_allocated=used_risk_metadata.allocated_epsilon,
            risk_budget_remaining=used_risk_metadata.remaining_epsilon,
            risk_constraint_count=used_risk_metadata.active_constraint_count,
            risk_budget_status=used_risk_metadata.budget_status,
            primary_solver_status=primary_solver_status,
            primary_solver_success=primary_solver_success,
            primary_solver_iterations=solver_iterations,
            primary_solver_primal_residual=primal_residual,
            primary_solver_dual_residual=dual_residual,
            residual_status=residual_status,
            primary_solver_primal_residual_status=primal.status.value,
            primary_solver_dual_residual_status=dual.status.value,
            primary_solver_residual_source="ipopt_iteration_history",
            primary_solver_residual_required_for_acceptance=False,
            primary_solver_residual_required_for_assurance=True,
            enforced_chance_profile=(enforced_profile if enforced_profile is not None else None),
            post_solve_diagnostic_profile=post_solve_diagnostic_profile,
            chance_profile_application_status=(
                "APPLIED" if enforced_profile is not None else "NOT_APPLICABLE_DETERMINISTIC"
            ),
            chance_profile_enforced_profile_id=(
                enforced_profile.provenance.profile_id if enforced_profile is not None else ""
            ),
            chance_profile_solve_attempt_id=solve_attempt_id,
            controller_timing=timing_recorder.snapshot(),
            achieved_risk_total=None if certificate is None else certificate.total,
            achieved_risk_max=None if certificate is None else certificate.maximum,
            risk_certificate_status=(
                "NOT_APPLICABLE" if certificate is None else certificate.status
            ),
        )
