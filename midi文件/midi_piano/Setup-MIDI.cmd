@echo off
setlocal
cd /d "%~dp0"
python -m venv "%~dp0.venv"
if errorlevel 1 goto failed
"%~dp0.venv\Scripts\python.exe" -m pip install -r "%~dp0requirements.txt"
if errorlevel 1 goto failed
echo Setup complete. Open Start-MIDI.cmd.
pause
exit /b 0
:failed
echo Setup failed. Python 3.10+ and package download access are required.
pause
exit /b 1
