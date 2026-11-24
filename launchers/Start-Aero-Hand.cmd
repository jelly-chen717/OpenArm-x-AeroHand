@echo off
setlocal
pushd "%~dp0.." || exit /b 1
".venv-aero\Scripts\python.exe" -m aero_open_sdk
set "launch_result=%errorlevel%"
popd
if not "%launch_result%"=="0" pause
exit /b %launch_result%
