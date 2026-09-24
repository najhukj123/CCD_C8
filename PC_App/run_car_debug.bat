@echo off
cd /d "%~dp0"
set "CCD_PORT=%~1"
if not defined CCD_PORT set "CCD_PORT=COM18"
py -3 car_debug_console.py %CCD_PORT%
if errorlevel 1 pause
