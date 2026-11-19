@echo off
cd /d "%~dp0"
"%~dp0.venv\Scripts\python.exe" "%~dp0tools\debug_entry.py" %*
if errorlevel 1 pause
