"""Headless native closed loop with recording and machine-readable progress.

Used by the desktop workbench (through ``quadrotor-mpc-native --headless``) so a
simulation can run in a background process without the MuJoCo window, while
still producing the same replayable recording directory as an interactive run.
"""

from __future__ import annotations

import json
from collections.abc import Callable, Mapping
from dataclasses import replace
from pathlib import Path
from typing import Any

from quadrotor_mpc.application.native.telemetry import (
    NativeRunRecorder,
    RecordingOptions,
    step_to_sample,
)

PROGRESS_PREFIX = "@@progress "
RESULT_PREFIX = "@@result "

#: Sample fields streamed to the workbench on every tick (kept small on purpose).
PROGRESS_FIELDS = (
    "time_s",
    "position",
    "min_clearance_m",
    "goal_distance_m",
    "solver_time_ms",
    "maximum_slack_m",
    "achieved_risk_total",
    "risk_certificate_status",
    "fallback_active",
    "horizon_assurance_status",
    "collided",
)


class HeadlessRecordingRuntime:
    """``CoupledRuntime`` without a window: records and reports every tick."""

    exit_on_completion = True

    def __init__(
        self,
        config: Any,
        *,
        base_dir: str | Path,
        enable_recording: bool = True,
        on_sample: Callable[[dict[str, Any]], None] | None = None,
        should_stop: Callable[[], bool] | None = None,
    ):
        options = config.recording
        if not enable_recording:
            options = RecordingOptions(
                enabled=False,
                output_dir=options.output_dir,
                max_buffer_samples=options.max_buffer_samples,
            )
        self.recorder = NativeRunRecorder(
            options,
            config.name,
            config.to_mapping(),
            base_dir=base_dir,
        )
        self._on_sample = on_sample
        self._should_stop = should_stop
        self.completion_reason: str | None = None

    def open(self, _plant: Any, _context: Any) -> None:
        return None

    def is_running(self) -> bool:
        return not (self._should_stop is not None and self._should_stop())

    def poll_commands(self) -> list:
        return []

    def on_step(self, step: Any) -> bool:
        sample = step_to_sample(step)
        self.recorder.record_step(step, sample)
        if self._on_sample is not None:
            self._on_sample(sample)
        return True

    def on_idle(self, _paused: bool) -> None:
        return None

    def on_reset(self) -> None:
        self.recorder.reset_episode()

    def on_completed(self, reason: str) -> None:
        self.completion_reason = str(reason)
        self.recorder.record_event("completed", 0.0, payload={"reason": self.completion_reason})

    def close(self) -> None:
        return None

    def finalize(self, result: Mapping[str, Any]) -> Path | None:
        return self.recorder.finalize(result)


def apply_run_overrides(
    config: Any,
    *,
    mode: str | None = None,
    noise_scale: float | None = None,
    seed: int | None = None,
    duration_s: float | None = None,
) -> Any:
    """Return a copy of ``config`` for one controller mode / noise / seed choice.

    Reuses the paired Monte Carlo transformation so an interactive run is
    configured exactly like the corresponding validation trial.
    """
    from quadrotor_mpc.application.validation.monte_carlo import effective_native_config

    updated = config
    if mode is not None or noise_scale is not None or seed is not None:
        updated = effective_native_config(
            config,
            mode=mode or _infer_mode(config),
            covariance_scale=1.0 if noise_scale is None else float(noise_scale),
            seed=int(config.estimation.seed if seed is None else seed),
        )
        # effective_native_config renames the scenario for Monte Carlo trials;
        # keep the original name so recordings stay recognisable.
        updated = replace(updated, name=config.name if mode is None else f"{config.name}-{mode}")
    if duration_s is not None:
        if duration_s <= 0.0:
            raise ValueError("duration must be > 0")
        updated = replace(updated, duration_s=float(duration_s))
    return updated


def _infer_mode(config: Any) -> str:
    chance = config.chance_constraints
    if not chance.enabled:
        return "deterministic"
    return "individual" if chance.risk_budget.semantics == "individual" else "joint_uniform"


def progress_line(sample: Mapping[str, Any], total_steps: int, step_index: int) -> str:
    payload = {key: _jsonable(sample.get(key)) for key in PROGRESS_FIELDS}
    payload["step"] = int(step_index)
    payload["total_steps"] = int(total_steps)
    return PROGRESS_PREFIX + json.dumps(payload, separators=(",", ":"))


def result_line(result: Mapping[str, Any], recording: Path | None) -> str:
    payload = {
        "termination_reason": str(result.get("termination_reason", "")),
        "collided": bool(result.get("collided", False)),
        "recording": None if recording is None else str(recording),
    }
    return RESULT_PREFIX + json.dumps(payload, separators=(",", ":"))


def _jsonable(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return value if value == value and abs(value) != float("inf") else None
    try:
        import numpy as np

        array = np.asarray(value, dtype=float)
        if array.ndim == 0:
            return _jsonable(float(array))
        return [_jsonable(float(item)) for item in array.reshape(-1)]
    except (TypeError, ValueError):
        return str(value)
