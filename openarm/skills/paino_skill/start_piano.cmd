@echo off
cd /d "%~dp0..\.."
".venv\Scripts\python.exe" -m openarm_app.graphics --configure
if errorlevel 1 goto end
".venv\Scripts\python.exe" -m openarm_app.sim_server --scene piano %*
:end
pause
