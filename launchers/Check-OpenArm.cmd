@echo off
"%~dp0..\.venv-aero\Scripts\python.exe" "%~dp0..\apps\openarm_driver.py" status %*
set "check_result=%errorlevel%"
pause
exit /b %check_result%
