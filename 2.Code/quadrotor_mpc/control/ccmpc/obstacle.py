"""
Obstacle representation and tracking for Chance-Constrained MPC.

Formulas from:
  "Robust Vision-based Obstacle Avoidance for Micro Aerial Vehicles
   in Dynamic Environments" — Lin, Zhu, Alonso-Mora, ICRA 2020

Equations:
  (6)  Constant velocity prediction
  (7)  Box-to-ellipsoid bounding
  (Collision) Omega matrix definition
"""

from __future__ import annotations

import math

import numpy as np
import numpy.typing as npt

from .utils import (
    Omega_half,
    Omega_matrix,
    box_to_ellipsoid_axes,
    yaw_to_rotation,
)


class EllipsoidalObstacle:
    """An ellipsoidal obstacle with position, velocity, size, and orientation.

    The obstacle is a bounding ellipsoid circumscribing a box detection,
    with Kalman-filter-style uncertainty tracking.
    """

    def __init__(
        self,
        position: npt.NDArray[np.float64] | list[float],
        size: npt.NDArray[np.float64] | list[float],
        yaw: float = 0.0,
        velocity: npt.NDArray[np.float64] | list[float] | None = None,
        pos_uncertainty: float = 0.05,
        vel_uncertainty: float = 0.1,
    ):
        # Position and velocity
        self.p_hat: npt.NDArray[np.float64] = np.array(position, dtype=np.float64)
        self.v_hat: npt.NDArray[np.float64] = (
            np.array(velocity, dtype=np.float64) if velocity is not None else np.zeros(3)
        )

        # Size (l, w, h) and ellipsoidal axes (a, b, c) per Eq 7
        self.size: npt.NDArray[np.float64] = np.array(size, dtype=np.float64)
        self.axes: npt.NDArray[np.float64] = box_to_ellipsoid_axes(self.size)

        # Orientation
        self.yaw: float = yaw
        self.R_o: npt.NDArray[np.float64] = yaw_to_rotation(yaw)

        # Uncertainty (Kalman filter style)
        self.Sigma: npt.NDArray[np.float64] = np.diag([pos_uncertainty**2] * 3)
        self.Sigma_v: npt.NDArray[np.float64] = np.diag([vel_uncertainty**2] * 3)

    def predict(self, dt: float) -> None:
        r"""Tracker time update between measurements (paper Eq 6).

        This advances the *estimate* in place.  Horizon predictions inside the
        controllers must use :meth:`predicted_mean_and_covariance` instead.

        .. math::

            \hat{p}^{k+1} &= \hat{p}^k + \hat{v}^k \Delta t \\
            \Sigma^{k+1} &= \Sigma^k + \Sigma_v \Delta t^2
        """
        self.p_hat += self.v_hat * dt
        self.Sigma += self.Sigma_v * dt**2

    def predicted_mean_and_covariance(
        self,
        elapsed_s: float,
    ) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        r"""Open-loop constant-velocity prediction over ``elapsed_s`` seconds.

        .. math::

            \hat{p}(t) = \hat{p} + \hat{v}\,t, \qquad
            \Sigma(t) = \Sigma + t^2\,\Sigma_v

        ``Sigma_v`` is the velocity-estimate covariance, so a constant but
        uncertain velocity produces position covariance growing with
        :math:`t^2` (Theory Ch. 14.8, cross-covariance neglected).  This single
        definition is shared by the CC-MPC constraint and the ODE residual
        checker; the recursive ``Sigma += Sigma_v * dt**2`` form grows only
        linearly in the step count and made the two disagree.
        """
        elapsed = float(elapsed_s)
        mean = self.p_hat + self.v_hat * elapsed
        covariance = self.Sigma + self.Sigma_v * elapsed**2
        return mean, covariance

    def get_normal(self, p_mav: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        r"""Unit normal vector from obstacle to MAV.

        .. math::

            n_o = \frac{\hat{p}_{mav} - \hat{p}_o}{\|\hat{p}_{mav} - \hat{p}_o\|}
        """
        diff = p_mav - self.p_hat
        dist = np.linalg.norm(diff)
        if dist < 1e-10:
            return np.array([1.0, 0.0, 0.0])
        return diff / dist

    def get_omega(self, mav_radius: float) -> npt.NDArray[np.float64]:
        r"""Compute the collision matrix :math:`\Omega`.

        See Eq (Collision) in the paper.
        """
        return Omega_matrix(self.axes, mav_radius, self.R_o)

    def get_omega_half(self, mav_radius: float) -> npt.NDArray[np.float64]:
        r"""Symmetric matrix square root :math:`\Omega^{1/2}`."""
        return Omega_half(self.get_omega(mav_radius))

    def distance_to(self, p_mav: npt.NDArray[np.float64]) -> float:
        """Euclidean distance from MAV to obstacle center."""
        return float(np.linalg.norm(p_mav - self.p_hat))

    def gaussian_pdf(self, measurement: npt.NDArray[np.float64]) -> float:
        """Gaussian PDF for data association (Eq 5).

        Evaluates p(measurement | predicted_state, predicted_cov).
        Higher value = more likely association.
        """
        diff = measurement - self.p_hat
        cov = self.Sigma
        det = np.linalg.det(2.0 * math.pi * cov)
        if det <= 0:
            return 0.0
        mahal = diff @ np.linalg.solve(cov, diff)
        return float(np.exp(-0.5 * mahal) / np.sqrt(det))

    def kalman_update(
        self,
        measurement: npt.NDArray[np.float64],
        R: npt.NDArray[np.float64] | None = None,
    ) -> None:
        """Kalman filter measurement update for position.

        Args:
            measurement: Observed position (3,).
            R: Measurement noise covariance (3x3). Default: identity scaled.
        """
        if R is None:
            R = np.eye(3) * 0.05**2
        # Kalman gain
        S = self.Sigma + R
        K = np.linalg.solve(S, self.Sigma).T  # K = Sigma @ inv(S)
        # Update
        innovation = measurement - self.p_hat
        self.p_hat += K @ innovation
        self.Sigma = (np.eye(3) - K) @ self.Sigma


def associate_detections(
    tracks: list[EllipsoidalObstacle],
    detections: list[tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]],
    threshold: float = 0.01,
    max_distance: float = 2.0,
) -> list[tuple[int, int]]:
    """Associate detections to existing tracks using Gaussian PDF (Eq 5).

    Args:
        tracks: Existing tracked obstacles.
        detections: List of (position, covariance) tuples from sensor.
        threshold: Minimum PDF value for association.
        max_distance: Maximum Euclidean distance for association.

    Returns:
        List of (track_idx, detection_idx) associations.
    """
    associations: list[tuple[int, int]] = []
    assigned_tracks = set()
    assigned_dets = set()

    for ti, track in enumerate(tracks):
        best_pd = 0.0
        best_di = -1
        for di, (pos, _) in enumerate(detections):
            if di in assigned_dets:
                continue
            dist = np.linalg.norm(pos - track.p_hat)
            if dist > max_distance:
                continue
            pd = track.gaussian_pdf(pos)
            if pd > best_pd and pd >= threshold:
                best_pd = pd
                best_di = di

        if best_di >= 0:
            associations.append((ti, best_di))
            assigned_tracks.add(ti)
            assigned_dets.add(best_di)

    return associations


class ObstacleManager:
    """Manages multiple ellipsoidal obstacles over time.

    Handles:
    - Updating positions of moving obstacles
    - Predicting states over the MPC horizon
    - Returning the closest N obstacles to the MPC
    """

    def __init__(self, obstacles: list[EllipsoidalObstacle] | None = None):
        self.obstacles: list[EllipsoidalObstacle] = obstacles if obstacles is not None else []

    @classmethod
    def from_config(cls, config: str | dict) -> ObstacleManager:
        """Create obstacles from YAML simulation config."""
        if isinstance(config, str):
            import yaml as _yaml

            with open(config) as f:
                config_data = _yaml.safe_load(f)
        else:
            config_data = config

        obs_list: list[EllipsoidalObstacle] = []
        for obs_cfg in config_data.get("obstacles", []):
            obs_list.append(
                EllipsoidalObstacle(
                    position=obs_cfg["position"],
                    size=obs_cfg["size"],
                    yaw=obs_cfg.get("yaw", 0.0),
                    velocity=obs_cfg.get("velocity", [0.0, 0.0, 0.0]),
                )
            )
        return cls(obs_list)

    def update(self, dt: float) -> None:
        """Advance all obstacles by dt (moving obstacles)."""
        for obs in self.obstacles:
            obs.predict(dt)

    def get_closest(
        self,
        p_mav: npt.NDArray[np.float64],
        k: int = 2,
    ) -> list[EllipsoidalObstacle]:
        """Return the k closest obstacles to the MAV."""
        if not self.obstacles:
            return []
        distances = [obs.distance_to(p_mav) for obs in self.obstacles]
        sorted_pairs = sorted(zip(distances, self.obstacles))
        return [obs for _, obs in sorted_pairs[:k]]
