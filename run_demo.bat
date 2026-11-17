@echo off
rem RADA-Align final-design demo (SIMULATION) - double-click to run on Windows.
rem First run: creates .venv and installs the pinned packages (needs Python 3.10+, internet).
rem Then starts the demo server and opens http://127.0.0.1:8001 . Close this window to stop.
setlocal
cd /d "%~dp0"

if exist ".venv\Scripts\python.exe" goto run

echo Creating the Python environment (.venv), first run only...
py -3.11 -m venv .venv 2>nul || py -3 -m venv .venv 2>nul || python -m venv .venv
if not exist ".venv\Scripts\python.exe" (
    echo.
    echo Python was not found. Install Python 3.11 from https://www.python.org/downloads/
    echo and tick "Add python.exe to PATH", then run this file again.
    pause
    exit /b 1
)
".venv\Scripts\python.exe" -m pip install -r requirements-lock.txt || goto fail
".venv\Scripts\python.exe" -m pip install -e . --no-deps || goto fail

:run
if not defined RADA_NO_BROWSER start "" cmd /c "timeout /t 3 /nobreak >nul & start http://127.0.0.1:8001"
".venv\Scripts\python.exe" -m rada_align.psd %*
exit /b %errorlevel%

:fail
echo.
echo Installing the packages failed (see the messages above).
pause
exit /b 1
