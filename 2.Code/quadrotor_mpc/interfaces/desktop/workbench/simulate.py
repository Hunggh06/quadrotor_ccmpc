"""Simulation page: configure one native run, watch it live, open the result."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PySide6.QtCore import QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from quadrotor_mpc.interfaces.desktop.workbench import theme
from quadrotor_mpc.interfaces.desktop.workbench.jobs import ProcessJob, module_command
from quadrotor_mpc.interfaces.desktop.workbench.widgets import (
    Card,
    KpiTile,
    SegmentedControl,
    draw_endpoints,
    draw_obstacles,
    field,
    fmt,
    make_plot,
    make_trajectory_plot,
    move_obstacle,
    page_header,
    reference_line,
    series_pen,
)

SCENARIO_DESCRIPTIONS = {
    "mujoco_native.yaml": "Static obstacles · ground-truth state",
    "mujoco_native_dynamic.yaml": "Crossing obstacle · ground-truth state",
    "mujoco_native_estimation.yaml": "Crossing obstacle · estimated state (ESEKF)",
    "mujoco_native_ccmpc.yaml": "Crossing obstacle · estimation + chance constraints",
}

NOISE_PRESETS = (("Low · 0.25 Σ", 0.25), ("Nominal · 1 Σ", 1.0), ("High · 4 Σ", 4.0))


@dataclass(frozen=True)
class ScenarioChoice:
    path: Path
    name: str
    description: str
    mode: str
    duration_s: float
    seed: int
    estimation: bool


def discover_scenarios(code_root: Path) -> list[ScenarioChoice]:
    from quadrotor_mpc.interfaces.desktop.viewer import load_native_mujoco_config

    choices: list[ScenarioChoice] = []
    for path in sorted((code_root / "config").glob("mujoco_native*.yaml")):
        try:
            config = load_native_mujoco_config(path)
        except Exception:  # noqa: BLE001 - a broken file must not hide the others
            continue
        chance = config.chance_constraints
        mode = (
            "deterministic"
            if not chance.enabled
            else ("individual" if chance.risk_budget.semantics == "individual" else "joint_uniform")
        )
        choices.append(
            ScenarioChoice(
                path=path,
                name=config.name,
                description=SCENARIO_DESCRIPTIONS.get(path.name, path.stem),
                mode=mode,
                duration_s=float(config.duration_s),
                seed=int(config.estimation.seed),
                estimation=bool(config.estimation.enabled),
            )
        )
    order = list(SCENARIO_DESCRIPTIONS)
    choices.sort(key=lambda item: order.index(item.path.name) if item.path.name in order else 99)
    return choices


class SimulationPage(QWidget):
    open_result = Signal(str)
    status_message = Signal(str)

    def __init__(self, code_root: Path, parent: QWidget | None = None):
        super().__init__(parent)
        self.code_root = code_root
        self.scenarios = discover_scenarios(code_root)
        self.job: ProcessJob | None = None
        self._recording: str | None = None
        self._reset_buffers()

        root = QVBoxLayout(self)
        root.setContentsMargins(24, 20, 24, 20)
        root.setSpacing(14)
        root.addWidget(
            page_header(
                "Simulate",
                "Run one closed-loop NMPC flight in MuJoCo. Watch it live here, or open the "
                "3-D viewer with the safety panel.",
            )
        )
        columns = QHBoxLayout()
        columns.setSpacing(16)
        root.addLayout(columns, 1)
        columns.addWidget(self._build_settings(), 0)
        columns.addLayout(self._build_live(), 1)

        self._plot_timer = QTimer(self)
        self._plot_timer.setInterval(120)
        self._plot_timer.timeout.connect(self._refresh_plots)
        self._scenario_changed(self.scenario.currentIndex())

    # ------------------------------------------------------------------ layout
    def _build_settings(self) -> Card:
        card = Card("Run settings")
        card.setFixedWidth(370)

        self.scenario = QComboBox()
        for choice in self.scenarios:
            self.scenario.addItem(choice.description, choice)
        self.scenario_note = QLabel("")
        self.scenario_note.setObjectName("Muted")
        self.scenario_note.setWordWrap(True)

        self.controller = SegmentedControl(
            [
                ("deterministic", "Deterministic"),
                ("individual", "Individual"),
                ("joint_uniform", "Joint risk"),
            ]
        )
        self.noise = QComboBox()
        for label, value in NOISE_PRESETS:
            self.noise.addItem(label, value)
        self.noise.setCurrentIndex(1)
        self.seed = QSpinBox()
        self.seed.setRange(0, 1_000_000)
        self.duration = QDoubleSpinBox()
        self.duration.setRange(0.5, 120.0)
        self.duration.setDecimals(1)
        self.duration.setSuffix(" s")
        for spin in (self.seed, self.duration):
            spin.setButtonSymbols(QSpinBox.ButtonSymbols.NoButtons)
        self.record = QCheckBox("Save recording for Results / replay")
        self.record.setChecked(True)

        card.body.addWidget(field("Scenario", self.scenario, self.scenario_note))
        card.body.addWidget(field("Controller", self.controller))
        card.body.addWidget(field("Noise level", self.noise))
        pair = QHBoxLayout()
        pair.setSpacing(10)
        pair.addWidget(field("Seed", self.seed))
        pair.addWidget(field("Duration", self.duration))
        card.body.addLayout(pair)
        card.body.addWidget(self.record)
        card.body.addSpacing(6)

        self.run_button = QPushButton("▶  Run simulation")
        self.run_button.setObjectName("Primary")
        self.run_button.clicked.connect(lambda: self.start(headless=True))
        self.viewer_button = QPushButton("Open 3-D viewer")
        self.viewer_button.setToolTip("Runs in the MuJoCo window with the live safety panel")
        self.viewer_button.clicked.connect(lambda: self.start(headless=False))
        self.stop_button = QPushButton("■  Stop")
        self.stop_button.setObjectName("Danger")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.stop)
        card.body.addWidget(self.run_button)
        card.body.addWidget(self.viewer_button)
        card.body.addWidget(self.stop_button)
        card.body.addStretch(1)
        note = QLabel(
            "Controller, noise and seed are applied exactly like a Monte Carlo trial, so a "
            "run here can be reproduced in a campaign."
        )
        note.setObjectName("Muted")
        note.setWordWrap(True)
        card.body.addWidget(note)
        default = next(
            (i for i, c in enumerate(self.scenarios) if c.path.name == "mujoco_native_ccmpc.yaml"),
            0,
        )
        self.scenario.setCurrentIndex(default)
        self.scenario.currentIndexChanged.connect(self._scenario_changed)
        return card

    def _build_live(self) -> QVBoxLayout:
        layout = QVBoxLayout()
        layout.setSpacing(12)
        tiles = QGridLayout()
        tiles.setSpacing(10)
        self.tiles = {
            "time": KpiTile("Sim time"),
            "goal": KpiTile("Distance to goal"),
            "clearance": KpiTile("Min clearance"),
            "solver": KpiTile("Solver time"),
            "risk": KpiTile("Horizon risk"),
            "safety": KpiTile("Safety"),
        }
        for index, tile in enumerate(self.tiles.values()):
            tiles.addWidget(tile, 0, index)
        layout.addLayout(tiles)

        progress_row = QHBoxLayout()
        self.progress = QProgressBar()
        self.progress.setRange(0, 1000)
        self.progress.setValue(0)
        self.progress.setTextVisible(False)
        self.progress_label = QLabel("Idle")
        self.progress_label.setObjectName("Muted")
        self.open_button = QPushButton("Open in Results →")
        self.open_button.setVisible(False)
        self.open_button.clicked.connect(
            lambda: self._recording and self.open_result.emit(self._recording)
        )
        progress_row.addWidget(self.progress, 1)
        progress_row.addWidget(self.progress_label)
        progress_row.addWidget(self.open_button)
        layout.addLayout(progress_row)

        self.tabs = QTabWidget()
        trajectory_tab = QWidget()
        trajectory_layout = QVBoxLayout(trajectory_tab)
        trajectory_layout.setContentsMargins(0, 8, 0, 0)
        self.trajectory_plot = make_trajectory_plot()
        trajectory_layout.addWidget(self.trajectory_plot)
        self.tabs.addTab(trajectory_tab, "Trajectory")

        signals_tab = QWidget()
        signals_layout = QVBoxLayout(signals_tab)
        signals_layout.setContentsMargins(0, 8, 0, 0)
        self.clearance_plot = make_plot("Minimum obstacle clearance", "clearance [m]")
        self.solver_plot = make_plot("Solver time per tick", "time [ms]")
        self.risk_plot = make_plot("Post-solve horizon risk (sum)", "risk")
        for plot in (self.clearance_plot, self.solver_plot, self.risk_plot):
            signals_layout.addWidget(plot)
        self.solver_plot.setXLink(self.clearance_plot)
        self.risk_plot.setXLink(self.clearance_plot)
        self.tabs.addTab(signals_tab, "Signals")

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(2000)
        self.tabs.addTab(self.log, "Log")
        layout.addWidget(self.tabs, 1)
        return layout

    # ----------------------------------------------------------------- actions
    def current_choice(self) -> ScenarioChoice | None:
        data = self.scenario.currentData()
        return data if isinstance(data, ScenarioChoice) else None

    def _scenario_changed(self, _index: int) -> None:
        choice = self.current_choice()
        if choice is None:
            return
        self.controller.set_value(choice.mode)
        self.seed.setValue(choice.seed)
        self.duration.setValue(choice.duration_s)
        self.noise.setEnabled(choice.estimation)
        self.scenario_note.setText(
            f"{choice.path.name}"
            + ("" if choice.estimation else "  ·  ground truth: noise level not used")
        )
        self._draw_static_scene()

    def build_arguments(self, *, headless: bool) -> list[str]:
        choice = self.current_choice()
        if choice is None:
            raise RuntimeError("no native scenario found in config/")
        arguments = [
            "--config",
            str(choice.path),
            "--mode",
            self.controller.value(),
            "--seed",
            str(self.seed.value()),
            "--duration",
            f"{self.duration.value():g}",
        ]
        if choice.estimation:
            arguments += ["--noise-scale", f"{float(self.noise.currentData()):g}"]
        if not self.record.isChecked():
            arguments.append("--no-record")
        if headless:
            arguments += ["--headless", "--progress-json"]
        return arguments

    def start(self, *, headless: bool) -> None:
        if self.job is not None and self.job.is_running():
            return
        self._reset_buffers()
        self._draw_static_scene()
        self._recording = None
        self.open_button.setVisible(False)
        self.log.clear()
        for tile in self.tiles.values():
            tile.set("—")
        command = module_command(
            "quadrotor_mpc.interfaces.cli.native", self.build_arguments(headless=headless)
        )
        self.job = ProcessJob(command, self.code_root, self)
        self.job.line.connect(self.log.appendPlainText)
        self.job.progress.connect(self._on_progress)
        self.job.result.connect(self._on_result)
        self.job.finished.connect(self._on_finished)
        self._set_running(True)
        self.progress_label.setText("Starting… (first run compiles the NLP)")
        self.status_message.emit("Simulation running" + ("" if headless else " in 3-D viewer"))
        if not headless:
            self.tabs.setCurrentWidget(self.log)
        self.job.start()
        self._plot_timer.start()

    def stop(self) -> None:
        if self.job is not None:
            self.job.stop()

    def _set_running(self, running: bool) -> None:
        self.run_button.setEnabled(not running)
        self.viewer_button.setEnabled(not running)
        self.stop_button.setEnabled(running)
        self.scenario.setEnabled(not running)

    # ------------------------------------------------------------- streaming
    def _reset_buffers(self) -> None:
        self._t: list[float] = []
        self._xy: list[tuple[float, float]] = []
        self._clearance: list[float] = []
        self._solver: list[float] = []
        self._risk: list[float] = []
        self._min_clearance = float("inf")
        self._dirty = False

    def _on_progress(self, sample: dict) -> None:
        time_s = float(sample.get("time_s") or 0.0)
        position = sample.get("position") or [np.nan, np.nan, np.nan]
        self._t.append(time_s)
        self._xy.append((float(position[0]), float(position[1])))
        self._clearance.append(_num(sample.get("min_clearance_m")))
        self._solver.append(_num(sample.get("solver_time_ms")))
        self._risk.append(_num(sample.get("achieved_risk_total")))
        self._dirty = True
        total = max(int(sample.get("total_steps") or 1), 1)
        step = int(sample.get("step") or 0)
        self.progress.setValue(int(1000 * min(step / total, 1.0)))
        self.progress_label.setText(f"{step}/{total} ticks")
        self._update_tiles(sample)

    def _update_tiles(self, sample: dict) -> None:
        self.tiles["time"].set(fmt(_num(sample.get("time_s")), "s", 2))
        self.tiles["goal"].set(fmt(_num(sample.get("goal_distance_m")), "m", 2))
        current = _num(sample.get("min_clearance_m"))
        if np.isfinite(current):
            self._min_clearance = min(self._min_clearance, current)
        clearance = self._min_clearance
        level = "critical" if clearance <= 0 else ("warning" if clearance < 0.1 else "good")
        self.tiles["clearance"].set(
            fmt(clearance, "m", 3),
            "collision" if clearance <= 0 else ("tight" if clearance < 0.1 else "clear"),
            level,
        )
        solver = _num(sample.get("solver_time_ms"))
        self.tiles["solver"].set(
            fmt(solver, "ms", 1),
            "over 50 ms period" if solver > 50 else "within period",
            "warning" if solver > 50 else "good",
        )
        status = str(sample.get("risk_certificate_status") or "")
        risk = _num(sample.get("achieved_risk_total"))
        if status in ("CERTIFIED", "EXCEEDED"):
            self.tiles["risk"].set(
                fmt(risk, "", 4),
                "within budget" if status == "CERTIFIED" else "over budget",
                "good" if status == "CERTIFIED" else "critical",
            )
        else:
            self.tiles["risk"].set("n/a", "deterministic")
        if sample.get("collided"):
            self.tiles["safety"].set("Collision", "geometric contact", "critical")
        elif sample.get("fallback_active"):
            self.tiles["safety"].set("Fallback", "primary rejected", "serious")
        else:
            self.tiles["safety"].set("Nominal", "primary NMPC", "good")

    def _refresh_plots(self) -> None:
        if not self._dirty:
            return
        self._dirty = False
        t = np.asarray(self._t)
        xy = np.asarray(self._xy).reshape(-1, 2)
        self._trajectory_curve.setData(xy[:, 0], xy[:, 1])
        self._clearance_curve.setData(t, np.asarray(self._clearance))
        self._solver_curve.setData(t, np.asarray(self._solver))
        risk = np.asarray(self._risk)
        if np.isfinite(risk).any():
            self._risk_curve.setData(t, risk)
            self.risk_plot.setYRange(
                0.0, max(self._risk_budget * 1.3, float(np.nanmax(risk)) * 1.1), padding=0
            )
        if len(t):
            self._update_dynamic_obstacles(float(t[-1]))

    def _on_result(self, payload: dict) -> None:
        self._recording = payload.get("recording")

    def _on_finished(self, code: int, stopped: bool) -> None:
        self._plot_timer.stop()
        self._refresh_plots()
        self._set_running(False)
        if stopped:
            self.progress_label.setText("Stopped")
            self.status_message.emit("Simulation stopped")
        elif code == 0:
            self.progress.setValue(1000)
            self.progress_label.setText("Finished")
            self.status_message.emit("Simulation finished")
            self.open_button.setVisible(bool(self._recording))
        else:
            self.progress_label.setText(f"Failed (exit {code}) — see Log")
            self.tabs.setCurrentWidget(self.log)
            self.status_message.emit("Simulation failed")

    # ---------------------------------------------------------------- drawing
    def _draw_static_scene(self) -> None:
        from quadrotor_mpc.interfaces.desktop.viewer import load_native_mujoco_config

        choice = self.current_choice()
        plots = (self.trajectory_plot, self.clearance_plot, self.solver_plot, self.risk_plot)
        for plot in plots:
            plot.clear()
        if choice is None:
            return
        config = load_native_mujoco_config(choice.path)
        self._obstacles = [dict(item) for item in config.obstacles]
        start = np.array([config.start["x"], config.start["y"], config.start["z"]])
        goal = np.array(
            [config.goal_position["x"], config.goal_position["y"], config.goal_position["z"]]
        )
        from quadrotor_mpc.core.obstacle_motion import predict_obstacle_positions

        ticks = max(int(config.duration_s / config.mpc_timestep_s), 1)
        paths = (
            np.swapaxes(
                predict_obstacle_positions(self._obstacles, 0.0, ticks + 1, config.mpc_timestep_s),
                0,
                1,
            )
            if self._obstacles
            else None
        )
        self._obstacle_items = draw_obstacles(
            self.trajectory_plot, self._obstacles, paths, at="start"
        )
        self._dt = float(config.mpc_timestep_s)
        draw_endpoints(self.trajectory_plot, start, goal)
        color = theme.CONTROLLER_COLORS.get(self.controller.value(), theme.ACCENT)
        self._trajectory_curve = self.trajectory_plot.plot(
            [], [], pen=series_pen(color, 2.5), name="vehicle"
        )
        self._clearance_curve = self.clearance_plot.plot([], [], pen=series_pen(color), name="run")
        reference_line(self.clearance_plot, 0.0, "contact", theme.CRITICAL)
        self._solver_curve = self.solver_plot.plot([], [], pen=series_pen(color), name="run")
        reference_line(self.solver_plot, 1000.0 * config.mpc_timestep_s, "period", theme.WARNING)
        self._risk_curve = self.risk_plot.plot([], [], pen=series_pen(color), name="run")
        budget = config.chance_constraints.risk_budget.total_epsilon
        reference_line(self.risk_plot, budget, "joint budget", theme.WARNING)
        self._risk_budget = float(budget)
        self.risk_plot.setYRange(0.0, budget * 1.3, padding=0)

    def _update_dynamic_obstacles(self, time_s: float) -> None:
        from quadrotor_mpc.core.obstacle_motion import predict_obstacle_positions

        if not self._obstacles:
            return
        positions = predict_obstacle_positions(self._obstacles, time_s, 1, self._dt)[:, 0, :2]
        for item, position in zip(self._obstacle_items, positions):
            move_obstacle(item, float(position[0]), float(position[1]))


def _num(value) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")
