"""Background jobs: run CLI commands in a child process and stream their output.

Simulations and campaigns run out-of-process so the UI never freezes, MuJoCo
and Qt never share an event loop, and Stop always works (kill).
"""

from __future__ import annotations

import json
import os
import re
import sys
from collections.abc import Sequence
from pathlib import Path

from PySide6.QtCore import QObject, QProcess, QProcessEnvironment, Signal

from quadrotor_mpc.application.native.headless import PROGRESS_PREFIX, RESULT_PREFIX

TRIAL_LINE = re.compile(r"^\[(\d+)/(\d+)\]\s+(.*)$")
OUTPUT_LINE = re.compile(r"^output:\s+(.+)$")


def python_executable() -> str:
    """Console interpreter for children (pythonw has no usable stdout)."""
    executable = Path(sys.executable)
    if executable.name.lower() == "pythonw.exe":
        candidate = executable.with_name("python.exe")
        if candidate.exists():
            return str(candidate)
    return str(executable)


def module_command(module: str, arguments: Sequence[str]) -> list[str]:
    return [python_executable(), "-u", "-m", module, *arguments]


class ProcessJob(QObject):
    """Wrap ``QProcess``; parse progress/result lines into signals."""

    line = Signal(str)
    progress = Signal(dict)
    trial = Signal(int, int, str)
    output_dir = Signal(str)
    result = Signal(dict)
    finished = Signal(int, bool)  # exit code, stopped by user

    def __init__(self, command: Sequence[str], working_directory: Path, parent=None):
        super().__init__(parent)
        self.command = list(command)
        self._buffer = ""
        self._stopped = False
        self.process = QProcess(self)
        self.process.setWorkingDirectory(str(working_directory))
        self.process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        environment = QProcessEnvironment.systemEnvironment()
        environment.insert("PYTHONUNBUFFERED", "1")
        environment.insert("PYTHONIOENCODING", "utf-8")
        if sys.platform.startswith("linux") and not os.environ.get("MUJOCO_GL"):
            environment.insert("MUJOCO_GL", "egl")
        self.process.setProcessEnvironment(environment)
        self.process.readyReadStandardOutput.connect(self._read)
        self.process.finished.connect(self._finished)
        self.process.errorOccurred.connect(self._error)

    def start(self) -> None:
        self.line.emit("$ " + " ".join(self.command[2:] if len(self.command) > 2 else self.command))
        self.process.start(self.command[0], self.command[1:])

    def stop(self) -> None:
        if self.process.state() == QProcess.ProcessState.NotRunning:
            return
        self._stopped = True
        self.process.terminate()
        if not self.process.waitForFinished(2500):
            self.process.kill()

    def is_running(self) -> bool:
        return self.process.state() != QProcess.ProcessState.NotRunning

    def _read(self) -> None:
        chunk = bytes(self.process.readAllStandardOutput()).decode("utf-8", errors="replace")
        self._buffer += chunk.replace("\r\n", "\n")
        while "\n" in self._buffer:
            text, self._buffer = self._buffer.split("\n", 1)
            self._handle(text.rstrip())

    def _handle(self, text: str) -> None:
        if not text:
            return
        if text.startswith(PROGRESS_PREFIX):
            try:
                self.progress.emit(json.loads(text[len(PROGRESS_PREFIX) :]))
            except json.JSONDecodeError:
                pass
            return
        if text.startswith(RESULT_PREFIX):
            try:
                self.result.emit(json.loads(text[len(RESULT_PREFIX) :]))
            except json.JSONDecodeError:
                pass
            return
        if "UserWarning" in text or "warnings.warn(" in text:
            return
        match = TRIAL_LINE.match(text)
        if match:
            self.trial.emit(int(match.group(1)), int(match.group(2)), match.group(3))
        output = OUTPUT_LINE.match(text)
        if output:
            self.output_dir.emit(output.group(1).strip())
        self.line.emit(text)

    def _finished(self, code: int, _status) -> None:
        if self._buffer.strip():
            self._handle(self._buffer.strip())
            self._buffer = ""
        self.finished.emit(int(code), self._stopped)

    def _error(self, error) -> None:
        if error == QProcess.ProcessError.FailedToStart:
            self.line.emit(f"failed to start: {self.command[0]}")
            self.finished.emit(-1, False)
