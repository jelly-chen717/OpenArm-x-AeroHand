@echo off
setlocal
set "DIR=%~dp0"
set "PYTHONPATH=%DIR%;%PYTHONPATH%"
python "%DIR%examples\recv_loop.py" %*
endlocal
