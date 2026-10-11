"""Reusable workbench widgets: cards, KPI tiles, status chips and plots."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QButtonGroup,
    QFrame,
    QGraphicsEllipseItem,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from quadrotor_mpc.interfaces.desktop.workbench import theme


class Card(QFrame):
    """Rounded surface with an optional uppercase title."""

    def __init__(self, title: str = "", parent: QWidget | None = None):
        super().__init__(parent)
        self.setObjectName("Card")
        self.body = QVBoxLayout(self)
        self.body.setContentsMargins(16, 14, 16, 16)
        self.body.setSpacing(10)
        if title:
            label = QLabel(title)
            label.setObjectName("CardTitle")
            self.body.addWidget(label)


class KpiTile(QFrame):
    """Hero number with label and a status hint (never colour alone)."""

    def __init__(self, label: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.setObjectName("Card")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 10, 14, 12)
        layout.setSpacing(2)
        self._label = QLabel(label)
        self._label.setObjectName("KpiLabel")
        self._value = QLabel("—")
        self._value.setObjectName("KpiValue")
        self._hint = QLabel("")
        self._hint.setObjectName("KpiHint")
        layout.addWidget(self._label)
        layout.addWidget(self._value)
        layout.addWidget(self._hint)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

    def set(self, value: str, hint: str = "", level: str | None = None) -> None:
        self._value.setText(value)
        color = theme.status_color(level) if level else theme.TEXT
        self._value.setStyleSheet(f"color: {color};")
        marker = {"good": "● ", "warning": "▲ ", "serious": "▲ ", "critical": "■ "}.get(
            level or "", ""
        )
        self._hint.setText(f"{marker}{hint}" if hint else "")

    @property
    def value_text(self) -> str:
        return self._value.text()


class StatusChip(QLabel):
    """Pill with icon + text; colour reinforces, text carries meaning."""

    def __init__(self, text: str = "", level: str = "", parent: QWidget | None = None):
        super().__init__(parent)
        self.set(text, level)

    def set(self, text: str, level: str = "") -> None:
        color = theme.status_color(level)
        icon = {"good": "✔", "warning": "▲", "serious": "▲", "critical": "✖"}.get(level, "•")
        self.setText(f"{icon}  {text}")
        self.setStyleSheet(
            f"color: {color}; border: 1px solid {color}; border-radius: 10px;"
            f"padding: 2px 10px; background: transparent; font-weight: 600;"
        )


class SegmentedControl(QWidget):
    """Mutually exclusive buttons (used for controller selection)."""

    def __init__(self, options: Sequence[tuple[str, str]], parent: QWidget | None = None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        self._group = QButtonGroup(self)
        self._group.setExclusive(True)
        self._buttons: dict[str, QPushButton] = {}
        for index, (key, text) in enumerate(options):
            button = QPushButton(text)
            button.setObjectName("Segment")
            button.setStyleSheet(
                "QPushButton { border-radius: 0; padding: 6px 4px; }"
                f"QPushButton:checked {{ background: {theme.ACCENT}; border-color: {theme.ACCENT};"
                " color: #ffffff; font-weight: 600; }"
            )
            button.setCheckable(True)
            button.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            self._group.addButton(button, index)
            self._buttons[key] = button
            layout.addWidget(button)
        first = next(iter(self._buttons.values()))
        first.setChecked(True)

    def value(self) -> str:
        for key, button in self._buttons.items():
            if button.isChecked():
                return key
        return next(iter(self._buttons))

    def set_value(self, key: str) -> None:
        if key in self._buttons:
            self._buttons[key].setChecked(True)


def field(label: str, widget: QWidget, note: QWidget | None = None) -> QWidget:
    """Label above input — stays readable in a narrow settings column."""
    box = QWidget()
    box.setStyleSheet("background: transparent;")
    layout = QVBoxLayout(box)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setSpacing(4)
    caption = QLabel(label)
    caption.setObjectName("FieldLabel")
    layout.addWidget(caption)
    layout.addWidget(widget)
    if note is not None:
        layout.addWidget(note)
    return box


def page_header(title: str, subtitle: str) -> QWidget:
    widget = QWidget()
    layout = QVBoxLayout(widget)
    layout.setContentsMargins(0, 0, 0, 6)
    layout.setSpacing(2)
    heading = QLabel(title)
    heading.setObjectName("PageTitle")
    sub = QLabel(subtitle)
    sub.setObjectName("PageSubtitle")
    sub.setWordWrap(True)
    layout.addWidget(heading)
    layout.addWidget(sub)
    return widget


def make_plot(
    title: str,
    left: str,
    bottom: str = "time [s]",
    legend_offset: tuple[int, int] = (-8, 8),
) -> pg.PlotWidget:
    plot = pg.PlotWidget()
    plot.setMinimumHeight(170)
    item = plot.getPlotItem()
    item.setTitle(title, color=theme.TEXT_SECONDARY, size="9pt")
    item.setLabel("left", left, color=theme.TEXT_MUTED)
    item.setLabel("bottom", bottom, color=theme.TEXT_MUTED)
    item.showGrid(x=True, y=True, alpha=0.12)
    for axis in ("left", "bottom"):
        item.getAxis(axis).setPen(pg.mkPen(theme.AXIS))
        item.getAxis(axis).setTextPen(pg.mkPen(theme.TEXT_MUTED))
        item.getAxis(axis).enableAutoSIPrefix(False)
    legend = item.addLegend(offset=legend_offset, labelTextColor=theme.TEXT_SECONDARY)
    legend.setBrush(pg.mkBrush(theme.SURFACE + "cc"))
    plot.setMenuEnabled(False)
    return plot


def series_pen(color: str, width: float = 2.0, dashed: bool = False) -> pg.mkPen:
    style = Qt.PenStyle.DashLine if dashed else Qt.PenStyle.SolidLine
    return pg.mkPen(color, width=width, style=style)


def reference_line(plot: pg.PlotWidget, value: float, label: str, color: str) -> None:
    line = pg.InfiniteLine(
        pos=value,
        angle=0,
        pen=pg.mkPen(color, width=1, style=Qt.PenStyle.DashLine),
        label=label,
        labelOpts={"color": color, "position": 0.03, "anchors": [(0, 1), (0, 1)]},
    )
    plot.addItem(line)


def circle_xy(center_x: float, center_y: float, radius: float, points: int = 64) -> np.ndarray:
    angles = np.linspace(0.0, 2.0 * np.pi, points)
    return np.column_stack([center_x + radius * np.cos(angles), center_y + radius * np.sin(angles)])


def make_trajectory_plot() -> pg.PlotWidget:
    plot = make_plot("Top-down trajectory", "y [m]", "x [m]", legend_offset=(-8, -8))
    plot.setAspectLocked(True)
    plot.setMinimumHeight(260)
    return plot


def draw_obstacles(
    plot: pg.PlotWidget,
    obstacles: Sequence[dict],
    paths: np.ndarray | None,
    *,
    at: str | int = "end",
) -> list[tuple[QGraphicsEllipseItem, pg.TextItem, float]]:
    """Draw obstacle discs plus dotted motion paths for moving obstacles.

    ``paths`` is ``(ticks, n_obstacles, 3)``; the disc is placed at the first or
    last sample (``at``). Returns the items so callers can animate them.
    """
    fill = pg.mkBrush(theme.CRITICAL + "33")
    pen = pg.mkPen(theme.CRITICAL, width=1.5)
    items: list[tuple[QGraphicsEllipseItem, pg.TextItem, float]] = []
    for index, obstacle in enumerate(obstacles):
        radius = float(obstacle.get("radius", 0.2))
        if paths is not None and paths.ndim == 3 and paths.shape[1] > index and len(paths):
            track = paths[:, index, :2]
            if np.ptp(track, axis=0).max() > 1e-6:
                plot.plot(
                    track[:, 0],
                    track[:, 1],
                    pen=pg.mkPen(theme.CRITICAL, width=1, style=Qt.PenStyle.DotLine),
                )
            if isinstance(at, int):
                center = track[min(max(at, 0), len(track) - 1)]
            else:
                center = track[0] if at == "start" else track[-1]
        else:
            center = np.array([float(obstacle.get("x", 0.0)), float(obstacle.get("y", 0.0))])
        disc = QGraphicsEllipseItem(
            float(center[0]) - radius, float(center[1]) - radius, 2 * radius, 2 * radius
        )
        disc.setPen(pen)
        disc.setBrush(fill)
        plot.addItem(disc)
        name = str(obstacle.get("name", f"obstacle {index}")).replace("_", " ")
        text = pg.TextItem(name, color=theme.TEXT_MUTED, anchor=(0.5, 1.1))
        text.setPos(float(center[0]), float(center[1]) + radius)
        plot.addItem(text)
        items.append((disc, text, radius))
    return items


def move_obstacle(
    item: tuple[QGraphicsEllipseItem, pg.TextItem, float], x: float, y: float
) -> None:
    disc, text, radius = item
    disc.setRect(x - radius, y - radius, 2 * radius, 2 * radius)
    text.setPos(x, y + radius)


def draw_endpoints(plot: pg.PlotWidget, start: np.ndarray, goal: np.ndarray) -> None:
    plot.plot(
        [float(start[0])],
        [float(start[1])],
        pen=None,
        symbol="o",
        symbolSize=10,
        symbolBrush=theme.TEXT_SECONDARY,
        symbolPen=pg.mkPen(theme.SURFACE, width=2),
        name="start",
    )
    plot.plot(
        [float(goal[0])],
        [float(goal[1])],
        pen=None,
        symbol="star",
        symbolSize=16,
        symbolBrush=theme.GOOD,
        symbolPen=pg.mkPen(theme.SURFACE, width=2),
        name="goal",
    )


def fmt(value: float | None, unit: str = "", digits: int = 3) -> str:
    if value is None or not np.isfinite(value):
        return "—"
    return f"{value:.{digits}f}{(' ' + unit) if unit else ''}"
