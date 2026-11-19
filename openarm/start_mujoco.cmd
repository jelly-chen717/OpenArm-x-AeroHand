@echo off
cd /d "%~dp0"
"%~dp0.venv\Scripts\python.exe" -m openarm_app.graphics --configure
if errorlevel 1 goto end
"%~dp0.venv\Scripts\python.exe" -m openarm_app.sim_server %*
:end
pause
