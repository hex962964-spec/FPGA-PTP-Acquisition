@echo off
cd /d "%~dp0"
where py >nul 2>&1
if errorlevel 1 goto python
py -3 receiver.py
goto done
:python
python receiver.py
:done
if errorlevel 1 pause
