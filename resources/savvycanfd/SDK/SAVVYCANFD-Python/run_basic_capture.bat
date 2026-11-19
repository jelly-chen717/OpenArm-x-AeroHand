@echo off
setlocal
set "DIR=%~dp0"
set "PYTHONPATH=%DIR%;%PYTHONPATH%"
python "%DIR%examples\basic_capture.py" %*
endlocal
