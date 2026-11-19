@echo off
cd /d "%~dp0"
"%~dp0midi_piano\.venv\Scripts\python.exe" "%~dp0seven_note_preview.py"
if errorlevel 1 pause
