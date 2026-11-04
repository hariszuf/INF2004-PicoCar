@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Start-PicoCarDashboard.ps1" %*
exit /b %errorlevel%
