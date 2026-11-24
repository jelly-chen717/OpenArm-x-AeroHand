@echo off
setlocal
pushd "%~dp0" || exit /b 1
"openarm\.venv\Scripts\python.exe" "apps\openarm_monitor.py" --mode hardware --left-channel 0x51 --right-channel 0x52 --open --reuse-running %*
set "launch_result=%errorlevel%"
popd
if not "%launch_result%"=="0" pause
exit /b %launch_result%
