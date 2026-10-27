@echo off
cd /d "%~dp0"
"%~dp0.venv-aero\Scripts\python.exe" -m aero_open_sdk
if errorlevel 1 pause
