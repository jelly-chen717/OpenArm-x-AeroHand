@echo off
cd /d "%~dp0"
".venv\Scripts\python.exe" -m paino_skill.right_melody_aero --viewer --bpm 60
if errorlevel 1 pause
