@echo off
cd /d "%~dp0"
"%~dp0.venv\Scripts\python.exe" -m openarm_app.graphics --configure
if errorlevel 1 exit /b 1
"%~dp0.venv\Scripts\python.exe" -c "from openarm_app.graphics import record_graphics; record_graphics('logs/graphics-explorer-reference')"
