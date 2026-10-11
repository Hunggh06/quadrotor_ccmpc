#!/usr/bin/env python3
"""Launch the Quadrotor MPC desktop workbench (simulate, Monte Carlo, results)."""

from __future__ import annotations

import argparse
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workspace",
        type=Path,
        help="code root that holds config/ and outputs/ (default: the installed package data)",
    )
    args = parser.parse_args(argv)
    try:
        from quadrotor_mpc.interfaces.desktop.workbench.app import run
    except ImportError as exc:  # pragma: no cover - depends on optional UI extras
        raise SystemExit(
            f"The workbench needs PySide6 and pyqtgraph: {exc}. "
            "Install with `uv sync --all-extras` or `pip install -e .[full]`."
        ) from exc
    return run(args.workspace.resolve() if args.workspace else None)


if __name__ == "__main__":
    raise SystemExit(main())
