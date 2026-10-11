@echo off
rem Launch the Quadrotor MPC desktop workbench (no terminal needed).
cd /d "%~dp0\2.Code"
if exist ".venv\Scripts\pythonw.exe" (
    start "" ".venv\Scripts\pythonw.exe" -m quadrotor_mpc.interfaces.cli.app
    exit /b 0
)
where uv >nul 2>nul
if %errorlevel%==0 (
    uv run quadrotor-mpc-app
    exit /b %errorlevel%
)
echo Could not find 2.Code\.venv or uv.
echo Run "uv sync --all-extras --locked" inside 2.Code first.
pause
