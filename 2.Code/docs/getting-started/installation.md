# Installation

> **In short:** install with `uv`, check that MuJoCo and Qt can open, run the
> tests. Takes about ten minutes.
>
> **Next:** [first run](first-run.md).

All commands run from the `2.Code/` directory.

## 1. Install the environment

Python 3.10 or newer is required (CI tests 3.10 and 3.12). The locked
environment is the preferred path:

```bash
git clone https://github.com/whoisLePhuc/quadrotor_mpc.git
cd quadrotor_mpc/2.Code
uv sync --all-extras --locked
```

`--all-extras` installs the optional native stack (MuJoCo, do-mpc, CasADi) and
the desktop UI (PySide6, pyqtgraph). Prefix commands with `uv run`, or activate
`.venv` first.

<details>
<summary>Without uv (pip)</summary>

```bash
python -m venv .venv
# Linux/macOS: source .venv/bin/activate
# Windows PowerShell: .\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -e ".[full,dev]"
```

The legacy split requirement files are still available for a source checkout:

```bash
python -m pip install -r requirements.txt
python -m pip install -r requirements-ui.txt
```

</details>

## 2. Operating-system packages

MuJoCo and the Qt windows need a working OpenGL/EGL runtime. On minimal
Debian/Ubuntu installations install:

```bash
sudo apt install libgl1 libegl1 libxcb-cursor0 libxkbcommon-x11-0
```

Choose the MuJoCo rendering backend with `MUJOCO_GL`:

| Situation | Setting |
|---|---|
| Interactive window on a Linux desktop | `MUJOCO_GL=glfw` |
| Headless server or CI | `MUJOCO_GL=egl` (Qt: `QT_QPA_PLATFORM=offscreen`) |
| Windows | leave unset |

The ODE track, headless native runs and Monte Carlo campaigns do not need a
visible window.

## 3. Check the installation

```bash
uv run quadrotor-mpc-native --validate-config          # parses the YAML, no MuJoCo needed
uv run quadrotor-mpc-monte-carlo --validate-config
uv run pytest -q                                       # full test suite
```

If all three succeed, continue with the [first run](first-run.md).

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `ModuleNotFoundError: PySide6` or `mujoco` | installed without extras; rerun `uv sync --all-extras --locked` |
| MuJoCo window does not open on Linux | missing OpenGL packages above, or `MUJOCO_GL` not set to `glfw` |
| Qt error about the `xcb` platform plugin | install `libxcb-cursor0` and `libxkbcommon-x11-0` |
| Monte Carlo refuses to start: dirty source | release campaigns require a clean Git tree; see [Monte Carlo campaigns](../guides/monte-carlo.md#source-provenance) |
