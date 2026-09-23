@echo off
cd /d "%~dp0"
if not exist "CarRemote_v6.exe" (
  echo CarRemote_v6.exe not found.
  pause
  exit /b 1
)
start "" "CarRemote_v6.exe"
if errorlevel 1 pause
