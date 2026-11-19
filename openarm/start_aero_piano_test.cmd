@echo off
cd /d "%~dp0"
"%~dp0.venv\Scripts\python.exe" -m tools.aero_piano_test.gui %*
if errorlevel 1 pause
