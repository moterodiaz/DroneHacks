@echo off
rem DroneHacks SITL: the graphical test environment (Windows).
rem Needs Python 3.10+ with numpy and pyyaml (sitl\requirements.txt); tkinter ships with Python.
cd /d "%~dp0"
where py >nul 2>nul
if %errorlevel%==0 (
  py -m dronesim gui
) else (
  python -m dronesim gui
)
if errorlevel 1 pause
