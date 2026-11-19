@echo off
cd /d "%~dp0..\.."
".venv\Scripts\python.exe" -m paino_skill.controller %*
pause
