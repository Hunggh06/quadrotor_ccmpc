"""Workbench main window."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

from PySide6.QtCore import QSize, Qt
from PySide6.QtGui import QAction, QFont, QKeySequence
from PySide6.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from quadrotor_mpc.infrastructure.resources import resource_root
from quadrotor_mpc.interfaces.desktop.workbench import theme
from quadrotor_mpc.interfaces.desktop.workbench.montecarlo import MonteCarloPage
from quadrotor_mpc.interfaces.desktop.workbench.results import ResultsPage
from quadrotor_mpc.interfaces.desktop.workbench.simulate import SimulationPage

PAGES = (("Simulate", "▶"), ("Monte Carlo", "▦"), ("Results", "▤"))


def git_state(code_root: Path) -> str:
    try:
        output = subprocess.run(
            ["git", "status", "--porcelain", "--", "."],
            cwd=code_root,
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
        commit = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=code_root,
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return "git: unavailable"
    if commit.returncode != 0:
        return "git: not a repository"
    dirty = any(line and not line[3:].startswith("outputs/") for line in output.stdout.splitlines())
    return f"commit {commit.stdout.strip()} · {'uncommitted changes' if dirty else 'clean'}"


class WorkbenchWindow(QMainWindow):
    def __init__(
        self,
        code_root: Path | None = None,
        *,
        data_roots: list[Path] | None = None,
        work_dir: Path | None = None,
    ):
        super().__init__()
        self.code_root = Path(code_root or resource_root())
        self.setWindowTitle("Quadrotor MPC Workbench")
        self.resize(1480, 920)
        self.setMinimumSize(QSize(1180, 720))

        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        sidebar = QWidget()
        sidebar.setObjectName("Sidebar")
        sidebar.setFixedWidth(210)
        side = QVBoxLayout(sidebar)
        side.setContentsMargins(0, 0, 0, 12)
        brand = QLabel("Quadrotor MPC")
        brand.setObjectName("Brand")
        sub = QLabel("Chance-constrained NMPC workbench")
        sub.setObjectName("BrandSub")
        sub.setWordWrap(True)
        side.addWidget(brand)
        side.addWidget(sub)
        self.nav = QListWidget()
        self.nav.setObjectName("NavList")
        for title, icon in PAGES:
            item = QListWidgetItem(f"{icon}   {title}")
            item.setSizeHint(QSize(190, 40))
            self.nav.addItem(item)
        side.addWidget(self.nav, 1)
        hint = QLabel("Ctrl+1/2/3 switch pages\nCtrl+R refresh results")
        hint.setObjectName("BrandSub")
        side.addWidget(hint)
        layout.addWidget(sidebar)

        self.stack = QStackedWidget()
        self.simulation = SimulationPage(self.code_root)
        self.monte_carlo = MonteCarloPage(self.code_root, data_roots=data_roots, work_dir=work_dir)
        self.results = ResultsPage(self.code_root, data_roots=data_roots)
        for page in (self.simulation, self.monte_carlo, self.results):
            self.stack.addWidget(page)
            page.status_message.connect(self._status)
        layout.addWidget(self.stack, 1)
        self.nav.currentRowChanged.connect(self._switch)
        self.nav.setCurrentRow(0)

        self.simulation.open_result.connect(self.show_result)
        self.monte_carlo.open_result.connect(self.show_result)

        self.git_label = QLabel(git_state(self.code_root))
        self.statusBar().addPermanentWidget(self.git_label)
        self.statusBar().showMessage(f"Workspace: {self.code_root}")

        for index in range(len(PAGES)):
            action = QAction(self)
            action.setShortcut(QKeySequence(f"Ctrl+{index + 1}"))
            action.triggered.connect(lambda _checked=False, i=index: self.nav.setCurrentRow(i))
            self.addAction(action)
        refresh = QAction(self)
        refresh.setShortcut(QKeySequence("Ctrl+R"))
        refresh.triggered.connect(self.results.refresh)
        self.addAction(refresh)

    def _switch(self, index: int) -> None:
        self.stack.setCurrentIndex(index)
        if self.stack.currentWidget() is self.results:
            self.results.refresh()
        if self.stack.currentWidget() is self.monte_carlo:
            self.monte_carlo.refresh_resumable()

    def _status(self, message: str) -> None:
        self.statusBar().showMessage(message, 8000)
        self.git_label.setText(git_state(self.code_root))

    def show_result(self, path: str) -> None:
        self.nav.setCurrentRow(2)
        self.results.show_path(path)

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt API
        for page in (self.simulation, self.monte_carlo):
            if page.job is not None and page.job.is_running():
                page.job.stop()
        super().closeEvent(event)


def create_application(argv: list[str] | None = None) -> QApplication:
    app = QApplication.instance() or QApplication(argv or sys.argv)
    app.setApplicationName("Quadrotor MPC Workbench")
    app.setStyle("Fusion")
    font = QFont()
    font.setFamilies(["Segoe UI", "Inter", "Helvetica Neue", "Arial"])
    font.setPointSizeF(10.0)
    app.setFont(font)
    app.setStyleSheet(theme.STYLESHEET)
    theme.configure_pyqtgraph()
    return app


def run(code_root: Path | None = None) -> int:
    app = create_application()
    window = WorkbenchWindow(code_root)
    window.show()
    return app.exec()


__all__ = ["WorkbenchWindow", "create_application", "run", "Qt"]
