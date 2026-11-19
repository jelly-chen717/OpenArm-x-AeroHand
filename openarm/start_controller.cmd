@echo off
cd /d "%~dp0"
"%~dp0.venv\Scripts\python.exe" -m openarm_app.controller_ui %*
if errorlevel 1 pause
