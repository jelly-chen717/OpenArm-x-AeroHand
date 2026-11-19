@echo off
cd /d "%~dp0.."
"%~dp0..\openarm\.venv\Scripts\python.exe" "%~dp0..\apps\openarm_monitor.py" --open %*
if errorlevel 1 pause
