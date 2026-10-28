@echo off
setlocal EnableExtensions
cd /d "%~dp0"
title rs-calibration web dry-run :6007
call "%~dp0..\scripts\env_python.bat" || goto :fail

REM Prefer linked sensors package (vendor\sensors junction)
if not exist "%~dp0vendor\sensors" (
  if not exist "%~dp0vendor" mkdir "%~dp0vendor"
  echo [rs-calib] creating junction vendor\sensors -^> ..\sensors\src\sensors
  mklink /J "%~dp0vendor\sensors" "%~dp0..\sensors\src\sensors" >nul
)

set "PYTHONPATH=%~dp0src;%~dp0vendor;%PYTHONPATH%"

echo [rs-calib] dry-run web  http://127.0.0.1:6007/
"%PYTHON%" -m rs_calib web --host 127.0.0.1 --port 6007
set "ERR=%ERRORLEVEL%"
goto :end

:fail
set "ERR=1"
:end
echo.
echo Exit code %ERR%. Press any key to close.
pause >nul
exit /b %ERR%
