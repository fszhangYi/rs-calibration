@echo off
chcp 65001 >nul
cd /d "%~dp0"

set "PYTHONPATH=%~dp0src;%~dp0vendor"
set "PY=C:\Users\user\miniconda3\python.exe"
set "HOST=127.0.0.1"
set "PORT=6006"

if not exist "%PY%" (
  where python >nul 2>&1
  if errorlevel 1 (
    echo [错误] 未找到 Python。请先安装 Miniconda 或把 python 加入 PATH。
    pause
    exit /b 1
  )
  set "PY=python"
)

REM 若 6006 已被占用，先结束占用进程（常见于上次未关干净）
for /f "tokens=5" %%P in ('netstat -ano ^| findstr /R /C:":%PORT% .*LISTENING"') do (
  echo 端口 %PORT% 被 PID %%P 占用，正在结束...
  taskkill /F /PID %%P >nul 2>&1
)
timeout /t 1 /nobreak >nul

echo 工作目录: %CD%
echo PYTHONPATH=%PYTHONPATH%
echo 启动 Web 上位机: http://%HOST%:%PORT%/
echo 关闭本窗口即可停止服务。
echo.
start "" "http://%HOST%:%PORT%/"
"%PY%" -m rs_calib web --host %HOST% --port %PORT%
set "ERR=%ERRORLEVEL%"
if not "%ERR%"=="0" (
  echo.
  echo [错误] 退出码 %ERR%
  pause
)
exit /b %ERR%
