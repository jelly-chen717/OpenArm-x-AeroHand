@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"
if not exist "%~dp0.venv\Scripts\python.exe" goto missing
"%~dp0.venv\Scripts\python.exe" -c "import mido, serial"
if errorlevel 1 goto missing
"%~dp0.venv\Scripts\python.exe" "%~dp0menu.py" "%~1"
set "result=%errorlevel%"
pause
exit /b %result%
:missing
echo Run Setup-MIDI.cmd first.
pause
exit /b 1
