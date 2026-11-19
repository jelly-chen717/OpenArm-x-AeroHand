@echo off
cd /d "%~dp0..\openarm"
".venv\Scripts\python.exe" tools\start_simulation.py --scene piano %*
if errorlevel 1 pause
