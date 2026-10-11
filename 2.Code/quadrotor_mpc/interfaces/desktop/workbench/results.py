"""Results page: browse, inspect and compare simulations and campaigns."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt, QUrl, Signal
from PySide6.QtGui import QDesktopServices, QPixmap
from PySide6.QtWidgets import (
    QAbstractItemView,
    QGridLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QSplitter,
    QStackedWidget,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from quadrotor_mpc.application.catalog import (
    CampaignSummary,
    NativeRunSummary,
    list_campaigns,
    list_native_runs,
    load_campaign_cells,
    load_native_timeseries,
    output_roots,
)
from quadrotor_mpc.interfaces.desktop.workbench import theme
from quadrotor_mpc.interfaces.desktop.workbench.widgets import (
    Card,
    KpiTile,
    StatusChip,
    draw_endpoints,
    draw_obstacles,
    fmt,
    make_plot,
    make_trajectory_plot,
    page_header,
    reference_line,
    series_pen,
)

COMPARE_COLORS = (theme.SERIES[0], theme.SERIES[1], theme.SERIES[2])

CLAIM_TEXT = {
    "EMPIRICALLY_SUPPORTED_NOT_PROVEN": ("Supported (empirical)", "good"),
    "NOT_APPLICABLE_DETERMINISTIC": ("Baseline", ""),
}


def _item(text: str, align_center: bool = False) -> QTableWidgetItem:
    item = QTableWidgetItem(text)
    item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
    if align_center:
        item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
    return item


def _semantics_label(value: str) -> str:
    return {
        "disabled": "Deterministic",
        "individual": "Individual risk",
        "joint": "Joint risk",
    }.get(value, value or "—")


class ResultsPage(QWidget):
    status_message = Signal(str)

    def __init__(
        self,
        code_root: Path,
        parent: QWidget | None = None,
        *,
        data_roots: list[Path] | None = None,
    ):
        super().__init__(parent)
        self.code_root = code_root
        self.data_roots = data_roots or output_roots(code_root)
        self.native_runs: list[NativeRunSummary] = []
        self.campaigns: list[CampaignSummary] = []

        root = QVBoxLayout(self)
        root.setContentsMargins(24, 20, 24, 20)
        root.setSpacing(14)
        header_row = QHBoxLayout()
        header_row.addWidget(
            page_header(
                "Results",
                "Recorded simulations and Monte Carlo campaigns. Select several "
                "simulations to overlay them.",
            ),
            1,
        )
        refresh = QPushButton("⟳  Refresh")
        refresh.clicked.connect(self.refresh)
        header_row.addWidget(refresh, 0, Qt.AlignmentFlag.AlignBottom)
        root.addLayout(header_row)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        root.addWidget(splitter, 1)
        splitter.addWidget(self._build_lists())
        self.detail = QStackedWidget()
        self.empty = QLabel("Select a simulation or campaign on the left.")
        self.empty.setObjectName("Muted")
        self.empty.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.native_view = NativeRunView()
        self.campaign_view = CampaignView()
        for widget in (self.empty, self.native_view, self.campaign_view):
            self.detail.addWidget(widget)
        splitter.addWidget(self.detail)
        splitter.setChildrenCollapsible(False)
        splitter.setSizes([470, 900])
        self.refresh()

    def _build_lists(self) -> QWidget:
        container = QWidget()
        container.setMinimumWidth(420)
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 8, 0)
        self.search = QLineEdit()
        self.search.setPlaceholderText("Filter by name, controller, status…")
        self.search.textChanged.connect(self._apply_filter)
        layout.addWidget(self.search)
        self.list_tabs = QTabWidget()
        self.native_table = self._table(["Created", "Controller", "Outcome", "Scenario"], stretch=3)
        self.native_table.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        self.native_table.itemSelectionChanged.connect(self._native_selected)
        self.campaign_table = self._table(["Created", "Trials", "Status", "Campaign"], stretch=3)
        self.campaign_table.itemSelectionChanged.connect(self._campaign_selected)
        self.list_tabs.addTab(self.native_table, "Simulations")
        self.list_tabs.addTab(self.campaign_table, "Campaigns")
        layout.addWidget(self.list_tabs, 1)
        buttons = QHBoxLayout()
        self.folder_button = QPushButton("Open folder")
        self.folder_button.clicked.connect(self._open_folder)
        buttons.addWidget(self.folder_button)
        buttons.addStretch(1)
        layout.addLayout(buttons)
        return container

    @staticmethod
    def _table(headers: list[str], stretch: int = 1) -> QTableWidget:
        table = QTableWidget(0, len(headers))
        table.setHorizontalHeaderLabels(headers)
        table.verticalHeader().setVisible(False)
        table.setAlternatingRowColors(True)
        table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        table.setShowGrid(False)
        header = table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(stretch, QHeaderView.ResizeMode.Stretch)
        return table

    # ---------------------------------------------------------------- data
    def refresh(self) -> None:
        roots = self.data_roots
        self.native_runs = list_native_runs(roots)
        self.campaigns = list_campaigns(roots)
        self.native_table.setRowCount(0)
        for run in self.native_runs:
            row = self.native_table.rowCount()
            self.native_table.insertRow(row)
            outcome = (
                "✖ collision"
                if run.collision
                else (
                    f"▲ {run.fallback_activations} fallback" if run.fallback_activations else "✔ ok"
                )
            )
            for column, text in enumerate(
                (_short_time(run.created), _semantics_label(run.risk_semantics), outcome, run.name)
            ):
                item = _item(text)
                item.setToolTip(f"{run.name}\n{run.path}")
                self.native_table.setItem(row, column, item)
            self.native_table.item(row, 0).setData(Qt.ItemDataRole.UserRole, str(run.path))
        self.campaign_table.setRowCount(0)
        for campaign in self.campaigns:
            row = self.campaign_table.rowCount()
            self.campaign_table.insertRow(row)
            status = _stage_text(campaign.stage_status or campaign.status)[0]
            for column, text in enumerate(
                (
                    _short_time(campaign.created),
                    f"{campaign.completed_trials}/{campaign.planned_trials}",
                    status,
                    campaign.name,
                )
            ):
                item = _item(text)
                item.setToolTip(str(campaign.path))
                self.campaign_table.setItem(row, column, item)
            self.campaign_table.item(row, 0).setData(Qt.ItemDataRole.UserRole, str(campaign.path))
        self._apply_filter(self.search.text())
        self.status_message.emit(
            f"{len(self.native_runs)} simulations · {len(self.campaigns)} campaigns"
        )

    def _apply_filter(self, text: str) -> None:
        needle = text.strip().lower()
        for table in (self.native_table, self.campaign_table):
            for row in range(table.rowCount()):
                haystack = " ".join(
                    table.item(row, column).text().lower() for column in range(table.columnCount())
                )
                table.setRowHidden(row, bool(needle) and needle not in haystack)

    def show_path(self, path: str) -> None:
        """Select a run/campaign by directory (used after a job finishes)."""
        self.refresh()
        target = Path(path).resolve()
        for table, tab in ((self.native_table, 0), (self.campaign_table, 1)):
            for row in range(table.rowCount()):
                stored = table.item(row, 0).data(Qt.ItemDataRole.UserRole)
                if stored and Path(stored).resolve() == target:
                    self.list_tabs.setCurrentIndex(tab)
                    table.clearSelection()
                    table.selectRow(row)
                    return

    def _selected_paths(self, table: QTableWidget) -> list[Path]:
        rows = sorted({index.row() for index in table.selectedIndexes()})
        return [Path(table.item(row, 0).data(Qt.ItemDataRole.UserRole)) for row in rows]

    def _native_selected(self) -> None:
        paths = self._selected_paths(self.native_table)[:3]
        if not paths:
            return
        try:
            series = [load_native_timeseries(path) for path in paths]
        except (OSError, KeyError, ValueError) as exc:
            self.status_message.emit(f"Could not load run: {exc}")
            return
        self.native_view.show_runs(series)
        self.detail.setCurrentWidget(self.native_view)

    def _campaign_selected(self) -> None:
        paths = self._selected_paths(self.campaign_table)
        if not paths:
            return
        self.campaign_view.show_campaign(paths[0])
        self.detail.setCurrentWidget(self.campaign_view)

    def _open_folder(self) -> None:
        table = self.native_table if self.list_tabs.currentIndex() == 0 else self.campaign_table
        paths = self._selected_paths(table)
        if paths:
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(paths[0])))


class NativeRunView(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 0, 0, 0)
        layout.setSpacing(12)
        self.title = QLabel("")
        self.title.setObjectName("PageTitle")
        self.subtitle = QLabel("")
        self.subtitle.setObjectName("Muted")
        layout.addWidget(self.title)
        layout.addWidget(self.subtitle)
        tiles = QGridLayout()
        tiles.setSpacing(10)
        self.tiles = {
            "outcome": KpiTile("Outcome"),
            "clearance": KpiTile("Min clearance"),
            "goal": KpiTile("Final distance to goal"),
            "solver": KpiTile("Solver p99"),
            "risk": KpiTile("Max horizon risk"),
            "fallback": KpiTile("Fallback ticks"),
        }
        for index, tile in enumerate(self.tiles.values()):
            tiles.addWidget(tile, 0, index)
        layout.addLayout(tiles)
        plots = QGridLayout()
        plots.setSpacing(10)
        self.trajectory = make_trajectory_plot()
        self.clearance = make_plot("Minimum obstacle clearance", "clearance [m]")
        self.solver = make_plot("Solver time per tick", "time [ms]")
        self.risk = make_plot("Post-solve horizon risk (sum)", "risk")
        self.solver.setXLink(self.clearance)
        self.risk.setXLink(self.clearance)
        plots.addWidget(self.trajectory, 0, 0, 3, 1)
        plots.addWidget(self.clearance, 0, 1)
        plots.addWidget(self.solver, 1, 1)
        plots.addWidget(self.risk, 2, 1)
        plots.setColumnStretch(0, 5)
        plots.setColumnStretch(1, 6)
        layout.addLayout(plots, 1)

    def show_runs(self, runs: list) -> None:
        primary = runs[0]
        compare = len(runs) > 1
        self.title.setText(primary.name if not compare else f"Comparing {len(runs)} simulations")
        self.subtitle.setText(
            "  ·  ".join(run.path.name for run in runs) if compare else str(primary.path)
        )
        for plot in (self.trajectory, self.clearance, self.solver, self.risk):
            plot.clear()
        # Obstacles are drawn where they were at the run's closest approach.
        closest = (
            int(np.nanargmin(primary.min_clearance_m))
            if np.isfinite(primary.min_clearance_m).any()
            else -1
        )
        draw_obstacles(
            self.trajectory, primary.obstacles, primary.obstacle_paths, at=max(closest, 0)
        )
        if closest >= 0:
            point = primary.position[closest]
            marker = pg.ScatterPlotItem(
                [float(point[0])],
                [float(point[1])],
                size=11,
                symbol="x",
                pen=pg.mkPen(theme.WARNING, width=2),
                brush=pg.mkBrush(theme.WARNING),
                name="closest approach",
            )
            self.trajectory.addItem(marker)
        draw_endpoints(self.trajectory, primary.start, primary.goal)
        reference_line(self.clearance, 0.0, "contact", theme.CRITICAL)
        reference_line(self.solver, primary.control_period_ms, "period", theme.WARNING)
        if primary.risk_budget_total is not None:
            reference_line(self.risk, primary.risk_budget_total, "joint budget", theme.WARNING)
        for index, run in enumerate(runs):
            color = COMPARE_COLORS[index] if compare else _run_color(run)
            label = (
                f"{_semantics_label(str(run.summary.get('risk_semantics', '')))} · "
                f"{_short_time(str(run.summary.get('created_utc', run.path.name)))[-5:]}"
                if compare
                else "run"
            )
            pen = series_pen(color, 2.5)
            self.trajectory.plot(run.position[:, 0], run.position[:, 1], pen=pen, name=label)
            self.clearance.plot(run.time_s, run.min_clearance_m, pen=series_pen(color), name=label)
            self.solver.plot(run.time_s, run.solver_time_ms, pen=series_pen(color), name=label)
            if np.isfinite(run.achieved_risk_total).any():
                risk = np.where(
                    np.isfinite(run.achieved_risk_total), run.achieved_risk_total, np.nan
                )
                self.risk.plot(
                    run.time_s, risk, pen=series_pen(color), name=label, connect="finite"
                )
        if primary.risk_budget_total is not None:
            peaks = [
                float(np.nanmax(r.achieved_risk_total))
                for r in runs
                if np.isfinite(r.achieved_risk_total).any()
            ]
            top = max([primary.risk_budget_total * 1.3] + [p * 1.1 for p in peaks])
            self.risk.setYRange(0.0, top, padding=0)
        self._fill_tiles(primary)
        if compare:
            self.subtitle.setText(
                "Tiles show the first selected run · " + "  ·  ".join(r.path.name for r in runs)
            )

    def _fill_tiles(self, run) -> None:
        summary = run.summary
        collided = bool(summary.get("collision", False))
        self.tiles["outcome"].set(
            "Collision" if collided else "Clear",
            str(summary.get("termination_reason", "")).replace("_", " "),
            "critical" if collided else "good",
        )
        clearance = float(np.nanmin(run.min_clearance_m)) if len(run.min_clearance_m) else None
        self.tiles["clearance"].set(
            fmt(clearance, "m", 3),
            "",
            "critical" if clearance is not None and clearance <= 0 else None,
        )
        goal = float(run.goal_distance_m[-1]) if len(run.goal_distance_m) else None
        self.tiles["goal"].set(fmt(goal, "m", 2))
        finite = run.solver_time_ms[np.isfinite(run.solver_time_ms)]
        p99 = float(np.percentile(finite, 99)) if finite.size else None
        over = p99 is not None and p99 > run.control_period_ms
        self.tiles["solver"].set(
            fmt(p99, "ms", 1),
            f"period {run.control_period_ms:.0f} ms",
            "warning" if over else "good",
        )
        risk = run.achieved_risk_total[np.isfinite(run.achieved_risk_total)]
        if risk.size and run.risk_budget_total is not None:
            peak = float(risk.max())
            ok = peak <= run.risk_budget_total
            self.tiles["risk"].set(
                fmt(peak, "", 4),
                f"budget {run.risk_budget_total:g}",
                "good" if ok else "critical",
            )
        else:
            self.tiles["risk"].set("n/a", "no joint budget")
        fallback = int(np.nansum(run.fallback_active))
        self.tiles["fallback"].set(str(fallback), "", "serious" if fallback else "good")


def _run_color(run) -> str:
    semantics = str(run.summary.get("risk_semantics", ""))
    key = {"disabled": "deterministic", "individual": "individual", "joint": "joint_uniform"}.get(
        semantics, "deterministic"
    )
    return theme.CONTROLLER_COLORS[key]


class CampaignView(QScrollArea):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setWidgetResizable(True)
        self.setFrameShape(QScrollArea.Shape.NoFrame)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        body = QWidget()
        self.setWidget(body)
        layout = QVBoxLayout(body)
        layout.setContentsMargins(8, 0, 8, 0)
        layout.setSpacing(12)
        self.title = QLabel("")
        self.title.setObjectName("PageTitle")
        self.subtitle = QLabel("")
        self.subtitle.setObjectName("Muted")
        self.subtitle.setWordWrap(True)
        layout.addWidget(self.title)
        layout.addWidget(self.subtitle)
        chips = QHBoxLayout()
        self.chips = {
            "stage": StatusChip(),
            "execution": StatusChip(),
            "real_time": StatusChip(),
            "claim": StatusChip(),
        }
        for chip in self.chips.values():
            chips.addWidget(chip)
        chips.addStretch(1)
        layout.addLayout(chips)

        gates = Card("Gates by noise level and controller")
        self.cells = QTableWidget(0, 8)
        self.cells.setHorizontalHeaderLabels(
            [
                "Noise",
                "Controller",
                "Success",
                "Collisions",
                "Bound",
                "Clearance",
                "Solver p99",
                "Claim",
            ]
        )
        self.cells.verticalHeader().setVisible(False)
        self.cells.setAlternatingRowColors(True)
        self.cells.setShowGrid(False)
        self.cells.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.cells.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        for column in (1, 7):
            self.cells.horizontalHeader().setSectionResizeMode(
                column, QHeaderView.ResizeMode.ResizeToContents
            )
        self.cells.setMinimumHeight(260)
        gates.body.addWidget(self.cells)
        layout.addWidget(gates)

        charts = QHBoxLayout()
        self.bound_plot = make_plot("Collision upper bound vs gate", "rate", "noise level")
        self.clearance_plot = make_plot("Mean minimum clearance", "clearance [m]", "noise level")
        charts.addWidget(self.bound_plot)
        charts.addWidget(self.clearance_plot)
        layout.addLayout(charts)

        report = Card("Campaign report figure")
        self.figure = QLabel("")
        self.figure.setMinimumWidth(10)
        self.figure.setAlignment(Qt.AlignmentFlag.AlignCenter)
        report.body.addWidget(self.figure)
        buttons = QHBoxLayout()
        self.report_button = QPushButton("Open report.md")
        self.report_button.clicked.connect(lambda: self._open("report.md"))
        self.folder_button = QPushButton("Open folder")
        self.folder_button.clicked.connect(lambda: self._open(""))
        buttons.addWidget(self.report_button)
        buttons.addWidget(self.folder_button)
        buttons.addStretch(1)
        report.body.addLayout(buttons)
        layout.addWidget(report)
        layout.addStretch(1)
        self._path: Path | None = None

    def _open(self, name: str) -> None:
        if self._path is not None:
            target = self._path / name if name else self._path
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(target)))

    def show_campaign(self, path: Path) -> None:
        from quadrotor_mpc.application.catalog import load_campaign_summary

        self._path = path
        summary = load_campaign_summary(path)
        self.title.setText(summary.name)
        self.subtitle.setText(
            f"{path.name} · {summary.completed_trials}/{summary.planned_trials} episodes · "
            f"protocol {summary.protocol.replace('_', ' ')}"
        )
        if not (path / "aggregate.json").is_file():
            self.cells.setRowCount(0)
            for key, chip in self.chips.items():
                chip.set(
                    "in progress" if key == "stage" else "—", "warning" if key == "stage" else ""
                )
            self.figure.clear()
            return
        aggregate, cells = load_campaign_cells(path)
        overall = aggregate.get("overall", {})
        stage = str(overall.get("stage_status", ""))
        self.chips["stage"].set(*_stage_text(stage))
        execution = overall.get("execution_integrity", "")
        self.chips["execution"].set(
            f"execution {execution.lower()}", "good" if execution == "PASS" else "critical"
        )
        real_time = overall.get("real_time", "")
        self.chips["real_time"].set(
            f"real-time {real_time.lower()}", "good" if real_time == "PASS" else "warning"
        )
        claim = overall.get("probabilistic_claim", "")
        self.chips["claim"].set(
            "claim supported" if claim.startswith("EMPIRICALLY") else "claim blocked",
            "good" if claim.startswith("EMPIRICALLY") else "warning",
        )
        self._fill_table(cells)
        self._fill_charts(cells)
        figure = path / "report.png"
        self._pixmap = QPixmap(str(figure)) if figure.is_file() else None
        if self._pixmap is None:
            self.figure.setText("No figure")
        self._rescale_figure()

    def _rescale_figure(self) -> None:
        pixmap = getattr(self, "_pixmap", None)
        if pixmap is None or pixmap.isNull():
            return
        width = max(200, min(pixmap.width(), self.viewport().width() - 70))
        self.figure.setPixmap(
            pixmap.scaledToWidth(width, Qt.TransformationMode.SmoothTransformation)
        )

    def resizeEvent(self, event) -> None:  # noqa: N802 - Qt API
        super().resizeEvent(event)
        self._rescale_figure()

    def _fill_table(self, cells) -> None:
        self.cells.setRowCount(0)
        for cell in cells:
            row = self.cells.rowCount()
            self.cells.insertRow(row)
            if cell.claim_status in CLAIM_TEXT:
                claim_text, level = CLAIM_TEXT[cell.claim_status]
            else:
                blockers = [b for b in cell.claim_blockers if b != "INSUFFICIENT_TRIALS"] or list(
                    cell.claim_blockers
                )
                claim_text = (
                    "blocked · " + blockers[0].replace("_", " ").lower()
                    if blockers
                    else cell.claim_status.replace("_", " ").lower()
                )
                if len(cell.claim_blockers) > 1:
                    claim_text += f" +{len(cell.claim_blockers) - 1}"
                level = "warning" if blockers else ""
            values = [
                f"{cell.covariance_scale:g} Σ",
                theme.CONTROLLER_LABELS.get(cell.controller, cell.controller),
                f"{cell.success_rate * 100:.0f}%",
                f"{cell.collision_events}/{cell.trials}",
                fmt(cell.collision_upper_bound, "", 3),
                fmt(cell.min_clearance_mean_m, "m", 3),
                fmt(cell.solver_p99_median_ms, "ms", 1),
                claim_text,
            ]
            for column, text in enumerate(values):
                item = _item(text, align_center=column < 7)
                if column == 7:
                    item.setToolTip("\n".join(cell.claim_blockers) or cell.claim_status)
                if column == 4:
                    item.setToolTip("One-sided upper confidence bound on the collision rate")
                if column == 1:
                    item.setForeground(
                        pg.mkColor(theme.CONTROLLER_COLORS.get(cell.controller, theme.TEXT))
                    )
                if column == 3 and cell.collision_events:
                    item.setForeground(pg.mkColor(theme.CRITICAL))
                if column == 7 and level:
                    item.setForeground(pg.mkColor(theme.status_color(level)))
                self.cells.setItem(row, column, item)

    def _fill_charts(self, cells) -> None:
        for plot in (self.bound_plot, self.clearance_plot):
            plot.clear()
        scales = sorted({cell.covariance_scale for cell in cells})
        controllers = [c for c in theme.CONTROLLER_COLORS if any(x.controller == c for x in cells)]
        ticks = [[(index, f"{scale:g} Σ") for index, scale in enumerate(scales)]]
        width = 0.8 / max(len(controllers), 1)
        for offset, controller in enumerate(controllers):
            color = theme.CONTROLLER_COLORS[controller]
            subset = {
                cell.covariance_scale: cell for cell in cells if cell.controller == controller
            }
            x = np.arange(len(scales)) - 0.4 + width * (offset + 0.5)
            bound = [
                (subset[s].collision_upper_bound or 0.0) if s in subset else 0.0 for s in scales
            ]
            clearance = [
                (subset[s].min_clearance_mean_m or 0.0) if s in subset else 0.0 for s in scales
            ]
            for plot, values in ((self.bound_plot, bound), (self.clearance_plot, clearance)):
                plot.addItem(
                    pg.BarGraphItem(
                        x=x,
                        height=values,
                        width=width * 0.9,
                        brush=pg.mkBrush(color),
                        pen=pg.mkPen(theme.SURFACE, width=2),
                        name=theme.CONTROLLER_LABELS[controller],
                    )
                )
        reference_line(self.bound_plot, 0.10, "gate 0.10", theme.WARNING)
        tallest = {
            self.bound_plot: max([c.collision_upper_bound or 0.0 for c in cells] + [0.1]),
            self.clearance_plot: max([c.min_clearance_mean_m or 0.0 for c in cells] + [0.1]),
        }
        for plot in (self.bound_plot, self.clearance_plot):
            plot.getPlotItem().getAxis("bottom").setTicks(ticks)
            plot.setYRange(0.0, tallest[plot] * 1.45, padding=0)
            plot.setXRange(-0.6, len(scales) - 0.4, padding=0)


def _stage_text(stage: str) -> tuple[str, str]:
    return {
        "PASS": ("pass", "good"),
        "VALIDATED_WITH_LIMITATIONS": ("validated with limitations", "warning"),
        "INCOMPLETE": ("too few trials for a claim", "warning"),
        "FAIL": ("fail", "critical"),
        "RUNNING": ("running", "warning"),
        "COMPLETED": ("completed", "good"),
    }.get(stage, (stage.replace("_", " ").lower() or "unknown", ""))


def _short_time(text: str) -> str:
    """Render a UTC timestamp (ISO or ``20261011T131033Z``) in local time."""
    from datetime import datetime, timezone

    raw = str(text).strip()
    parsed = None
    try:
        parsed = datetime.fromisoformat(raw.replace("Z", "+00:00"))
    except ValueError:
        try:
            parsed = datetime.strptime(raw[:16], "%Y%m%dT%H%M%S").replace(tzinfo=timezone.utc)
        except ValueError:
            parsed = None
    if parsed is None:
        return raw
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone().strftime("%Y-%m-%d %H:%M")
