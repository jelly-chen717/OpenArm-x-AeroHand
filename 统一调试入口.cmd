@echo off
cd /d "%~dp0"
"%~dp0openarm\.venv\Scripts\python.exe" "%~dp0apps\debug_launcher.py" %*
if errorlevel 1 pause
