"""3.Notebooks: structure checks (always) and full execution (opt-in).

Execution runs every learning notebook top to bottom with ``QMPC_NOTEBOOK_FAST=1``.
It takes a few minutes, so it only runs when ``QMPC_RUN_NOTEBOOKS=1`` (CI sets it).
"""

from __future__ import annotations

import importlib.util
import json
import os
import re
import unittest
from pathlib import Path

NOTEBOOKS = Path(__file__).resolve().parents[2] / "3.Notebooks"
TRACK = sorted(NOTEBOOKS.glob("[0-9][0-9]_*.ipynb"))
EXTRAS = sorted((NOTEBOOKS / "extras").glob("*.ipynb"))


def _sources(path: Path) -> list[tuple[str, str]]:
    notebook = json.loads(path.read_text(encoding="utf-8"))
    return [(cell["cell_type"], "".join(cell["source"])) for cell in notebook["cells"]]


class NotebookStructureTests(unittest.TestCase):
    def test_learning_track_exists_and_is_listed(self):
        self.assertGreaterEqual(len(TRACK), 9)
        readme = (NOTEBOOKS / "README.md").read_text(encoding="utf-8")
        for path in TRACK:
            self.assertIn(path.name, readme)
        self.assertIn("extras/", readme)
        self.assertGreaterEqual(len(EXTRAS), 1)

    def test_track_notebooks_use_the_package_not_copies(self):
        for path in TRACK:
            code = [source for kind, source in _sources(path) if kind == "code"]
            self.assertEqual(code[0].strip(), "from nbsetup import *", path.name)
            joined = "\n".join(code)
            self.assertIn("quadrotor_mpc", joined, path.name)
            for duplicated in ("def continuous_dynamics", "def erfinv", "def Omega_matrix"):
                self.assertNotIn(duplicated, joined, f"{path.name} re-implements {duplicated}")

    def test_no_machine_specific_paths_or_saved_figures(self):
        for path in TRACK + EXTRAS:
            for _, source in _sources(path):
                self.assertIsNone(re.search(r"/home/|C:\\\\Users|savefig\(", source), path.name)

    def test_extras_are_marked_as_not_implemented(self):
        for path in EXTRAS:
            self.assertIn("not implemented in `quadrotor_mpc`", _sources(path)[0][1], path.name)


@unittest.skipUnless(
    os.environ.get("QMPC_RUN_NOTEBOOKS") == "1"
    and importlib.util.find_spec("nbclient") is not None
    and importlib.util.find_spec("mujoco") is not None,
    "set QMPC_RUN_NOTEBOOKS=1 with the notebooks and native extras installed",
)
class NotebookExecutionTests(unittest.TestCase):
    def test_learning_track_executes(self):
        import nbformat
        from nbclient import NotebookClient

        os.environ["QMPC_NOTEBOOK_FAST"] = "1"
        for path in TRACK:
            with self.subTest(notebook=path.name):
                notebook = nbformat.read(path, as_version=4)
                NotebookClient(
                    notebook,
                    timeout=900,
                    kernel_name="python3",
                    resources={"metadata": {"path": str(NOTEBOOKS)}},
                ).execute()


if __name__ == "__main__":
    unittest.main()
