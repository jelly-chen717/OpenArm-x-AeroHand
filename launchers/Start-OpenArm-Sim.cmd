@echo off
cd /d "%~dp0..\openarm"
".venv\Scripts\python.exe" tools\start_simulation.py --scene default %*
if errorlevel 1 pause
