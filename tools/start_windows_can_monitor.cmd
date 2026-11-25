@echo off
setlocal
set "TASK_ROOT=%~dp0.."
start "" /D "%TASK_ROOT%" "%TASK_ROOT%\.venv\Scripts\pythonw.exe" "%~dp0windows_can_monitor.py"
