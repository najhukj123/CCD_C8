@echo off
cd /d "%~dp0"
py -3 car_debug_console.py
if errorlevel 1 pause
