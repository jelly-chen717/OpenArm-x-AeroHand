@echo off
cd /d "%~dp0"
"%~dp0midi_piano\.venv\Scripts\python.exe" "%~dp0piano_midi_gui.py"
if errorlevel 1 pause
