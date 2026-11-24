@echo off
cd /d "%~dp0"
"%~dp0.venv\Scripts\python.exe" -m tools.aero_score.gui %*
if errorlevel 1 pause
