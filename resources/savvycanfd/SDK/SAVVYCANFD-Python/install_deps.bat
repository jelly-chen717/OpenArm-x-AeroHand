@echo off
REM Optional: install Python dev tools. The core usb_can package needs no
REM external Python dependencies (pure ctypes).
setlocal
echo usb_can has no required runtime dependencies.
echo Optional: pytest (run tests), ipython (interactive).
echo.
choice /M "Install pytest + ipython"
if errorlevel 2 (
    echo Skipped.
    exit /b 0
)
python -m pip install --upgrade pip
python -m pip install pytest ipython
endlocal
