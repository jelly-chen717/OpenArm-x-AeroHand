@echo off
setlocal
pushd "%~dp0.." || exit /b 1
"openarm\.venv\Scripts\python.exe" "apps\debug_launcher.py" %*
set "launch_result=%errorlevel%"
popd
if not "%launch_result%"=="0" pause
exit /b %launch_result%
