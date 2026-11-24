@echo off
setlocal
pushd "%~dp0..\openarm" || exit /b 1
".venv\Scripts\python.exe" "tools\start_simulation.py" --scene piano --reuse-running %*
set "launch_result=%errorlevel%"
popd
if not "%launch_result%"=="0" pause
exit /b %launch_result%
