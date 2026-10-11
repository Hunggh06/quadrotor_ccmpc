"""Dark technical theme for the workbench (Qt stylesheet + plot colours).

Chart colours follow a validated dark categorical palette (blue, orange, aqua:
all-pairs CVD-safe on the dark surface) and a reserved status palette that is
never reused for series.
"""

from __future__ import annotations

# Surfaces & ink
PAGE = "#0d0d0d"
SURFACE = "#1a1a19"
SURFACE_RAISED = "#222220"
SURFACE_HOVER = "#2a2a28"
BORDER = "#2c2c2a"
AXIS = "#383835"
TEXT = "#f0efec"
TEXT_SECONDARY = "#c3c2b7"
TEXT_MUTED = "#898781"

# Categorical slots (dark steps) — assigned by controller identity, never rank.
SERIES = ("#3987e5", "#d95926", "#199e70")
CONTROLLER_COLORS = {
    "deterministic": SERIES[0],
    "individual": SERIES[1],
    "joint_uniform": SERIES[2],
}
ACCENT = SERIES[0]

# Status (reserved; always paired with a text label)
GOOD = "#0ca30c"
WARNING = "#fab219"
SERIOUS = "#ec835a"
CRITICAL = "#d03b3b"

CONTROLLER_LABELS = {
    "deterministic": "Deterministic",
    "individual": "Individual risk",
    "joint_uniform": "Joint risk",
}

FONT_FAMILY = '"Segoe UI", "Inter", "Helvetica Neue", Arial, sans-serif'
MONO_FAMILY = '"Cascadia Mono", "JetBrains Mono", Consolas, "DejaVu Sans Mono", monospace'

STYLESHEET = f"""
QWidget {{
    background: {PAGE};
    color: {TEXT};
    font-family: {FONT_FAMILY};
    font-size: 10pt;
}}
QToolTip {{
    background: {SURFACE_RAISED}; color: {TEXT}; border: 1px solid {AXIS}; padding: 4px;
}}
#Sidebar {{ background: {SURFACE}; border-right: 1px solid {BORDER}; }}
#Brand {{ font-size: 13pt; font-weight: 600; padding: 18px 16px 4px 16px; background: transparent; }}
#BrandSub {{ color: {TEXT_MUTED}; font-size: 8.5pt; padding: 0 16px 14px 16px; background: transparent; }}
#NavList {{ background: transparent; border: none; outline: none; }}
#NavList::item {{
    color: {TEXT_SECONDARY}; padding: 10px 16px; margin: 2px 8px; border-radius: 6px;
}}
#NavList::item:hover {{ background: {SURFACE_HOVER}; color: {TEXT}; }}
#NavList::item:selected {{ background: {SURFACE_RAISED}; color: {TEXT}; border-left: 3px solid {ACCENT}; }}
#PageTitle {{ font-size: 15pt; font-weight: 600; background: transparent; }}
#PageSubtitle {{ color: {TEXT_MUTED}; background: transparent; }}
#Card {{ background: {SURFACE}; border: 1px solid {BORDER}; border-radius: 10px; }}
#Card QLabel, #Card QCheckBox, #Card QRadioButton {{ background: transparent; }}
#CardTitle {{ color: {TEXT_SECONDARY}; font-size: 8.5pt; font-weight: 600;
    letter-spacing: 0.6px; text-transform: uppercase; }}
#KpiValue {{ font-size: 15pt; font-weight: 600; font-family: {MONO_FAMILY}; }}
#KpiLabel {{ color: {TEXT_MUTED}; font-size: 8.5pt; }}
#KpiHint {{ color: {TEXT_SECONDARY}; font-size: 8.5pt; }}
#Muted {{ color: {TEXT_MUTED}; background: transparent; }}
QLabel {{ background: transparent; }}
QPushButton {{
    background: {SURFACE_RAISED}; color: {TEXT}; border: 1px solid {AXIS};
    border-radius: 6px; padding: 7px 14px;
}}
QPushButton:hover {{ background: {SURFACE_HOVER}; }}
QPushButton:disabled {{ color: {TEXT_MUTED}; border-color: {BORDER}; }}
QPushButton#Primary {{ background: {ACCENT}; border: 1px solid {ACCENT}; color: #ffffff; font-weight: 600; }}
QPushButton#Primary:hover {{ background: #4f97ec; }}
QPushButton#Primary:disabled {{ background: {SURFACE_RAISED}; border-color: {BORDER}; color: {TEXT_MUTED}; }}
QPushButton#Danger {{ border-color: {CRITICAL}; color: #ffb4b4; }}
QPushButton#Danger:disabled {{ border-color: {BORDER}; color: {TEXT_MUTED}; }}
QPushButton#Segment {{ border-radius: 0; padding: 6px 4px; }}
QPushButton#Segment:checked {{ background: {ACCENT}; border-color: {ACCENT}; color: #ffffff; }}
QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit {{
    background: {SURFACE_RAISED}; border: 1px solid {AXIS}; border-radius: 6px;
    padding: 5px 8px; selection-background-color: {ACCENT};
}}
QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus, QLineEdit:focus {{ border-color: {ACCENT}; }}
QComboBox QAbstractItemView {{ background: {SURFACE_RAISED}; border: 1px solid {AXIS};
    selection-background-color: {SURFACE_HOVER}; }}
QCheckBox {{ spacing: 8px; }}
QCheckBox::indicator {{ width: 15px; height: 15px; border: 1px solid {TEXT_MUTED};
    border-radius: 4px; background: {SURFACE_RAISED}; }}
QCheckBox::indicator:hover {{ border-color: {ACCENT}; }}
QCheckBox::indicator:checked {{ background: {ACCENT}; border-color: {ACCENT}; }}
#FieldLabel {{ color: {TEXT_SECONDARY}; font-size: 9pt; background: transparent; }}
QProgressBar {{ background: {SURFACE_RAISED}; border: 1px solid {BORDER}; border-radius: 5px;
    height: 10px; text-align: center; color: {TEXT_SECONDARY}; font-size: 8pt; }}
QProgressBar::chunk {{ background: {ACCENT}; border-radius: 4px; }}
QTableWidget, QTreeWidget, QListWidget {{
    background: {SURFACE}; alternate-background-color: #1f1f1d; border: 1px solid {BORDER};
    border-radius: 8px; gridline-color: {BORDER}; selection-background-color: #23364f;
    selection-color: {TEXT};
}}
QHeaderView::section {{ background: {SURFACE_RAISED}; color: {TEXT_SECONDARY}; border: none;
    border-bottom: 1px solid {AXIS}; padding: 6px; font-weight: 600; }}
QTabWidget::pane {{ border: none; }}
QTabBar::tab {{ background: transparent; color: {TEXT_MUTED}; padding: 8px 14px; border: none;
    border-bottom: 2px solid transparent; }}
QTabBar::tab:selected {{ color: {TEXT}; border-bottom: 2px solid {ACCENT}; }}
QTabBar::tab:hover {{ color: {TEXT}; }}
QPlainTextEdit {{ background: {SURFACE}; border: 1px solid {BORDER}; border-radius: 8px;
    font-family: {MONO_FAMILY}; font-size: 9pt; color: {TEXT_SECONDARY}; }}
QScrollBar:vertical {{ background: transparent; width: 10px; margin: 2px; }}
QScrollBar::handle:vertical {{ background: {AXIS}; border-radius: 4px; min-height: 30px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ height: 0; width: 0; }}
QScrollBar:horizontal {{ background: transparent; height: 10px; margin: 2px; }}
QScrollBar::handle:horizontal {{ background: {AXIS}; border-radius: 4px; min-width: 30px; }}
QSplitter::handle {{ background: {PAGE}; }}
QStatusBar {{ background: {SURFACE}; color: {TEXT_MUTED}; border-top: 1px solid {BORDER}; }}
QStatusBar QLabel {{ background: transparent; }}
"""


def configure_pyqtgraph() -> None:
    import pyqtgraph as pg

    pg.setConfigOptions(antialias=True, background=SURFACE, foreground=TEXT_SECONDARY)


def status_color(level: str) -> str:
    return {
        "good": GOOD,
        "warning": WARNING,
        "serious": SERIOUS,
        "critical": CRITICAL,
    }.get(level, TEXT_MUTED)
