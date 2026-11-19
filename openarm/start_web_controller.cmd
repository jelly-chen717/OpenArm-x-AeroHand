@echo off
cd /d "%~dp0"
".venv\Scripts\python.exe" -m openarm_app.web_controller %*
if errorlevel 1 pause
