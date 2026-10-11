"""Monte Carlo page: configure, run, resume and watch a paired campaign."""

from __future__ import annotations

import time
from datetime import datetime
from pathlib import Path

import yaml
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QFormLayout,
    QGridLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from quadrotor_mpc.application.catalog import list_campaigns, output_roots
from quadrotor_mpc.interfaces.desktop.workbench import theme
from quadrotor_mpc.interfaces.desktop.workbench.jobs import ProcessJob, module_command
from quadrotor_mpc.interfaces.desktop.workbench.widgets import Card, KpiTile, page_header

NOISE_OPTIONS = (
    ("quarter_sigma", 0.25, "0.25 Σ"),
    ("nominal_sigma", 1.0, "1 Σ"),
    ("four_sigma", 4.0, "4 Σ"),
)
NOISE_DISPLAY = {label: text for label, _scale, text in NOISE_OPTIONS}
MODES = (
    ("deterministic", "Deterministic"),
    ("individual", "Individual risk"),
    ("joint_uniform", "Joint risk"),
)


def build_protocol(
    *,
    base_protocol: Path,
    trials: int,
    first_seed: int,
    modes: list[str],
    noise_levels: list[tuple[str, float]],
    protocol: str,
    name: str,
) -> dict:
    """Start from the shipped protocol so statistics/claim rules stay identical."""
    mapping = yaml.safe_load(base_protocol.read_text(encoding="utf-8"))
    mapping["name"] = name
    mapping["base_config"] = str(
        (base_protocol.parent / mapping.get("base_config", "mujoco_native_ccmpc.yaml")).resolve()
    )
    mapping["trials"] = int(trials)
    mapping["first_seed"] = int(first_seed)
    mapping["modes"] = list(modes)
    mapping["noise_levels"] = [
        {"label": label, "covariance_scale": float(scale)} for label, scale in noise_levels
    ]
    mapping.setdefault("monte_carlo", {})["protocol"] = protocol
    return mapping


class MonteCarloPage(QWidget):
    open_result = Signal(str)
    status_message = Signal(str)

    def __init__(
        self,
        code_root: Path,
        parent: QWidget | None = None,
        *,
        data_roots: list[Path] | None = None,
        work_dir: Path | None = None,
    ):
        super().__init__(parent)
        self.code_root = code_root
        self.data_roots = data_roots or output_roots(code_root)
        self.work_dir = work_dir or code_root / "outputs" / "workbench"
        self.job: ProcessJob | None = None
        self._run_dir: str | None = None
        self._started = 0.0

        root = QVBoxLayout(self)
        root.setContentsMargins(24, 20, 24, 20)
        root.setSpacing(14)
        root.addWidget(
            page_header(
                "Monte Carlo",
                "Paired seeds across controllers and noise levels. Runs in the background; "
                "an interrupted campaign can be resumed.",
            )
        )
        columns = QHBoxLayout()
        columns.setSpacing(16)
        root.addLayout(columns, 1)
        columns.addWidget(self._build_settings(), 0)
        columns.addLayout(self._build_live(), 1)
        self.refresh_resumable()

    def _build_settings(self) -> Card:
        card = Card("Campaign")
        card.setFixedWidth(340)
        form = QFormLayout()
        form.setSpacing(10)
        self.trials = QSpinBox()
        self.trials.setRange(1, 1000)
        self.trials.setValue(50)
        self.trials.setSuffix(" seeds")
        self.first_seed = QSpinBox()
        self.first_seed.setRange(0, 1_000_000)
        self.first_seed.setValue(1000)
        self.workers = QSpinBox()
        self.workers.setRange(1, 64)
        self.workers.setValue(3)
        self.batch = QSpinBox()
        self.batch.setRange(1, 100)
        self.batch.setValue(5)
        self.protocol = QComboBox()
        self.protocol.addItem("Algorithmic comparison", "algorithmic_comparison")
        self.protocol.addItem("Real-time qualification", "realtime_qualification")
        for spin in (self.trials, self.first_seed, self.workers, self.batch):
            spin.setButtonSymbols(QSpinBox.ButtonSymbols.NoButtons)
        form.addRow("Trials / cell", self.trials)
        form.addRow("First seed", self.first_seed)
        form.addRow("Workers", self.workers)
        form.addRow("Batch size", self.batch)
        form.addRow("Protocol", self.protocol)
        card.body.addLayout(form)

        card.body.addWidget(_section("Controllers"))
        self.mode_boxes: dict[str, QCheckBox] = {}
        for key, label in MODES:
            box = QCheckBox(label)
            box.setChecked(True)
            box.toggled.connect(self._update_estimate)
            self.mode_boxes[key] = box
            card.body.addWidget(box)
        card.body.addWidget(_section("Noise levels"))
        noise_row = QHBoxLayout()
        self.noise_boxes: dict[str, QCheckBox] = {}
        for label, _scale, text in NOISE_OPTIONS:
            box = QCheckBox(text)
            box.setChecked(True)
            box.toggled.connect(self._update_estimate)
            self.noise_boxes[label] = box
            noise_row.addWidget(box)
        card.body.addLayout(noise_row)
        self.allow_dirty = QCheckBox("Allow uncommitted source (smoke test only)")
        self.allow_dirty.setToolTip(
            "Release evidence requires a clean Git tree. Smoke runs are labelled "
            "FAIL_DIRTY_SOURCE and must not be reported."
        )
        card.body.addWidget(self.allow_dirty)
        self.estimate = QLabel("")
        self.estimate.setObjectName("Muted")
        card.body.addWidget(self.estimate)
        self.trials.valueChanged.connect(self._update_estimate)

        self.start_button = QPushButton("▶  Start campaign")
        self.start_button.setObjectName("Primary")
        self.start_button.clicked.connect(self.start)
        self.stop_button = QPushButton("■  Stop (resumable)")
        self.stop_button.setObjectName("Danger")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.stop)
        card.body.addWidget(self.start_button)
        card.body.addWidget(self.stop_button)

        card.body.addWidget(_section("Resume"))
        resume_row = QHBoxLayout()
        self.resume_combo = QComboBox()
        self.resume_button = QPushButton("Resume")
        self.resume_button.clicked.connect(self.resume)
        resume_row.addWidget(self.resume_combo, 1)
        resume_row.addWidget(self.resume_button)
        card.body.addLayout(resume_row)
        card.body.addStretch(1)
        self._update_estimate()
        return card

    def _build_live(self) -> QVBoxLayout:
        layout = QVBoxLayout()
        layout.setSpacing(12)
        tiles = QGridLayout()
        tiles.setSpacing(10)
        self.tiles = {
            "progress": KpiTile("Trials done"),
            "eta": KpiTile("Remaining"),
            "collisions": KpiTile("Collisions"),
            "slowest": KpiTile("Worst solver p99"),
        }
        for index, tile in enumerate(self.tiles.values()):
            tiles.addWidget(tile, 0, index)
        layout.addLayout(tiles)
        row = QHBoxLayout()
        self.progress = QProgressBar()
        self.progress.setTextVisible(False)
        self.progress.setRange(0, 1)
        self.progress_label = QLabel("Idle")
        self.progress_label.setObjectName("Muted")
        self.open_button = QPushButton("Open report →")
        self.open_button.setVisible(False)
        self.open_button.clicked.connect(
            lambda: self._run_dir and self.open_result.emit(self._run_dir)
        )
        row.addWidget(self.progress, 1)
        row.addWidget(self.progress_label)
        row.addWidget(self.open_button)
        layout.addLayout(row)

        self.tabs = QTabWidget()
        self.table = QTableWidget(0, 7)
        self.table.setHorizontalHeaderLabels(
            ["#", "Noise", "Controller", "Seed", "Collision", "Clearance", "Solver p99"]
        )
        self.table.verticalHeader().setVisible(False)
        self.table.setAlternatingRowColors(True)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.tabs.addTab(self.table, "Trials")
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(5000)
        self.tabs.addTab(self.log, "Log")
        layout.addWidget(self.tabs, 1)
        return layout

    # -------------------------------------------------------------- helpers
    def selected_modes(self) -> list[str]:
        return [key for key, box in self.mode_boxes.items() if box.isChecked()]

    def selected_noise(self) -> list[tuple[str, float]]:
        return [
            (label, scale)
            for label, scale, _text in NOISE_OPTIONS
            if self.noise_boxes[label].isChecked()
        ]

    def planned_trials(self) -> int:
        return self.trials.value() * len(self.selected_modes()) * len(self.selected_noise())

    def _update_estimate(self) -> None:
        planned = self.planned_trials()
        self.estimate.setText(f"{planned} episodes planned")
        self.start_button.setEnabled(planned > 0 and not (self.job and self.job.is_running()))

    def refresh_resumable(self) -> None:
        self.resume_combo.clear()
        for campaign in list_campaigns(self.data_roots):
            if campaign.resumable:
                self.resume_combo.addItem(
                    f"{campaign.path.name}  ({campaign.completed_trials}/"
                    f"{campaign.planned_trials})",
                    str(campaign.path),
                )
        has_any = self.resume_combo.count() > 0
        if not has_any:
            self.resume_combo.addItem("No interrupted campaign")
        self.resume_combo.setEnabled(has_any)
        self.resume_button.setEnabled(has_any)

    def write_protocol(self) -> Path:
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        mapping = build_protocol(
            base_protocol=self.code_root / "config" / "native_monte_carlo.yaml",
            trials=self.trials.value(),
            first_seed=self.first_seed.value(),
            modes=self.selected_modes(),
            noise_levels=self.selected_noise(),
            protocol=str(self.protocol.currentData()),
            name=f"workbench-{stamp}",
        )
        target = self.work_dir / f"protocol-{stamp}.yaml"
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(yaml.safe_dump(mapping, sort_keys=False), encoding="utf-8")
        return target

    # -------------------------------------------------------------- actions
    def start(self) -> None:
        protocol_path = self.write_protocol()
        arguments = [
            "--config",
            str(protocol_path),
            "--workers",
            str(self.workers.value()),
            "--batch-size",
            str(self.batch.value()),
        ]
        if self.allow_dirty.isChecked():
            arguments.append("--allow-dirty-source")
        self._launch(arguments, self.planned_trials())

    def resume(self) -> None:
        directory = self.resume_combo.currentData()
        if not directory:
            return
        protocol_path = Path(directory) / "protocol.yaml"
        arguments = [
            "--config",
            str(protocol_path),
            "--resume",
            str(directory),
            "--workers",
            str(self.workers.value()),
            "--batch-size",
            str(self.batch.value()),
        ]
        if self.allow_dirty.isChecked():
            arguments.append("--allow-dirty-source")
        self._launch(arguments, 0)

    def _launch(self, arguments: list[str], planned: int) -> None:
        if self.job is not None and self.job.is_running():
            return
        self.table.setRowCount(0)
        self.log.clear()
        self._collisions = 0
        self._worst_p99 = 0.0
        self._run_dir = None
        self.open_button.setVisible(False)
        self.progress.setRange(0, max(planned, 1))
        self.progress.setValue(0)
        for tile in self.tiles.values():
            tile.set("—")
        self.job = ProcessJob(
            module_command("quadrotor_mpc.interfaces.cli.monte_carlo", arguments),
            self.code_root,
            self,
        )
        self.job.line.connect(self.log.appendPlainText)
        self.job.trial.connect(self._on_trial)
        self.job.output_dir.connect(self._on_output_dir)
        self.job.finished.connect(self._on_finished)
        self._started = time.monotonic()
        self._set_running(True)
        self.progress_label.setText("Starting… (compiling controllers)")
        self.status_message.emit("Monte Carlo campaign running")
        self.job.start()

    def stop(self) -> None:
        if self.job is not None:
            self.job.stop()

    def _set_running(self, running: bool) -> None:
        self.start_button.setEnabled(not running and self.planned_trials() > 0)
        self.resume_button.setEnabled(not running and self.resume_combo.isEnabled())
        self.stop_button.setEnabled(running)

    def _on_output_dir(self, path: str) -> None:
        self._run_dir = path

    def _on_trial(self, index: int, total: int, text: str) -> None:
        fields = _parse_trial(text)
        self.progress.setRange(0, total)
        self.progress.setValue(index)
        elapsed = time.monotonic() - self._started
        remaining = elapsed / max(index, 1) * (total - index)
        self.progress_label.setText(f"{index}/{total} episodes")
        self.tiles["progress"].set(f"{index}/{total}")
        self.tiles["eta"].set(_duration(remaining), f"elapsed {_duration(elapsed)}")
        collided = fields.get("collision", "0") not in ("0", "False", "false")
        self._collisions += int(collided)
        self.tiles["collisions"].set(
            str(self._collisions),
            "geometric contacts",
            "critical" if self._collisions else "good",
        )
        p99 = _float(fields.get("p99", "").rstrip("ms"))
        if p99 is not None:
            self._worst_p99 = max(self._worst_p99, p99)
            self.tiles["slowest"].set(
                f"{self._worst_p99:.0f} ms",
                "over 50 ms period" if self._worst_p99 > 50 else "within period",
                "warning" if self._worst_p99 > 50 else "good",
            )
        row = self.table.rowCount()
        self.table.insertRow(row)
        values = [
            str(index),
            NOISE_DISPLAY.get(fields.get("noise", ""), fields.get("noise", "")),
            theme.CONTROLLER_LABELS.get(fields.get("controller", ""), fields.get("controller", "")),
            fields.get("seed", ""),
            "yes" if collided else "no",
            fields.get("clearance", ""),
            fields.get("p99", ""),
        ]
        for column, value in enumerate(values):
            item = QTableWidgetItem(value)
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            if column == 4 and collided:
                item.setForeground(_brush(theme.CRITICAL))
            self.table.setItem(row, column, item)
        self.table.scrollToBottom()

    def _on_finished(self, code: int, stopped: bool) -> None:
        self._set_running(False)
        self.refresh_resumable()
        if stopped:
            self.progress_label.setText("Stopped — can be resumed")
            self.status_message.emit("Campaign stopped (resumable)")
        elif code == 0:
            self.progress_label.setText("Completed")
            self.status_message.emit("Campaign completed")
            self.open_button.setVisible(bool(self._run_dir))
        else:
            self.progress_label.setText(f"Failed (exit {code}) — see Log")
            self.tabs.setCurrentWidget(self.log)
            self.status_message.emit("Campaign failed")


def _section(text: str) -> QLabel:
    label = QLabel(text)
    label.setObjectName("CardTitle")
    return label


def _parse_trial(text: str) -> dict[str, str]:
    """Parse ``nominal_sigma joint_uniform seed=1000  collision=0 clearance=…``."""
    parts = text.split()
    fields: dict[str, str] = {}
    if len(parts) >= 2:
        fields["noise"], fields["controller"] = parts[0], parts[1]
    for part in parts[2:]:
        if "=" in part:
            key, value = part.split("=", 1)
            fields[key] = value
    return fields


def _float(text: str) -> float | None:
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def _duration(seconds: float) -> str:
    seconds = max(0, int(seconds))
    hours, rest = divmod(seconds, 3600)
    minutes, secs = divmod(rest, 60)
    return f"{hours}h {minutes:02d}m" if hours else f"{minutes}m {secs:02d}s"


def _brush(color: str):
    from PySide6.QtGui import QBrush, QColor

    return QBrush(QColor(color))
