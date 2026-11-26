@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0launchers\Stop-OpenArm-Monitor.ps1" %*
set "stop_result=%errorlevel%"
pause
exit /b %stop_result%
