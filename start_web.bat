@echo off
chcp 65001 >nul
cd /d "%~dp0"

set "PYTHONPATH=%~dp0src;%~dp0vendor"
set "PY=C:\Users\user\miniconda3\python.exe"
set "HOST=127.0.0.1"
set "PORT=6006"
set "URL=http://%HOST%:%PORT%/"

if not exist "%PY%" (
  where python >nul 2>&1
  if errorlevel 1 (
    echo [错误] 未找到 Python。请先安装 Miniconda 或把 python 加入 PATH。
    pause
    exit /b 1
  )
  set "PY=python"
)

REM 若端口已被占用，先结束占用进程（常见于上次未关干净）
for /f "tokens=5" %%P in ('netstat -ano ^| findstr /R /C:":%PORT% .*LISTENING"') do (
  echo 端口 %PORT% 被 PID %%P 占用，正在结束...
  taskkill /F /PID %%P >nul 2>&1
)
timeout /t 1 /nobreak >nul

echo 工作目录: %CD%
echo PYTHONPATH=%PYTHONPATH%
echo [检查] OpenCV 4.x + calibrateHandEye …
"%PY%" -m rs_calib.infra.ensure_opencv
if errorlevel 1 (
  echo [错误] OpenCV HandEye 不可用，双算法无法启用。已尝试自动修复失败。
  echo 请手动执行: "%PY%" -m pip install "opencv-python>=4.8,^<5"
  pause
  exit /b 1
)
echo 启动 Web 上位机: %URL%
echo 后端就绪后会自动打开浏览器；页面点「安全退出」可关闭前后端。
echo 也可设 RS_CALIB_NO_BROWSER=1 禁止自动弹窗。
echo.

"%PY%" -m rs_calib web --host %HOST% --port %PORT%
set "ERR=%ERRORLEVEL%"
if not "%ERR%"=="0" (
  echo.
  echo [错误] 退出码 %ERR%
  pause
)
exit /b %ERR%
