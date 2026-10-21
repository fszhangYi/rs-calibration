@echo off
chcp 65001 >nul
cd /d "%~dp0"

set "PYTHONPATH=%~dp0src;%~dp0vendor"
set "PY=C:\Users\user\miniconda3\python.exe"

if not exist "%PY%" (
  where python >nul 2>&1
  if errorlevel 1 (
    echo [错误] 未找到 Python。请先安装 Miniconda 或把 python 加入 PATH。
    pause
    exit /b 1
  )
  set "PY=python"
)

echo 工作目录: %CD%
echo PYTHONPATH=%PYTHONPATH%
echo 启动: RS Eye-to-Hand 标定上位机 ...
"%PY%" -m rs_calib
set "ERR=%ERRORLEVEL%"
if not "%ERR%"=="0" (
  echo.
  echo [错误] 退出码 %ERR%
  pause
)
exit /b %ERR%
