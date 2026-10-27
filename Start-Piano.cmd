@echo off
cd /d "%~dp0"
echo 1. Simulation - no hardware
echo 2. Play 3 cycles
echo 3. Play continuously
choice /c 123 /n /m "Select [1/2/3]: "
if errorlevel 3 goto continuous
if errorlevel 2 goto trial
"%~dp0.venv-aero\Scripts\python.exe" "%~dp0piano_player.py"
goto end
:trial
"%~dp0.venv-aero\Scripts\python.exe" "%~dp0piano_player.py" --run --cycles 3
goto end
:continuous
"%~dp0.venv-aero\Scripts\python.exe" "%~dp0piano_player.py" --run --cycles 0
:end
pause
