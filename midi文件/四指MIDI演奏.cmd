@echo off
cd /d "%~dp0"
"%~dp0midi_piano\.venv\Scripts\python.exe" "%~dp0piano_four_midi.py" --run
pause
