@echo off
cd /d "%~dp0"
py -3 -c "import serial" >nul 2>&1
if errorlevel 1 (
  py -3 -m pip install -r requirements.txt
  if errorlevel 1 pause & exit /b 1
)
py -3 ccd_viewer.py
if errorlevel 1 pause
