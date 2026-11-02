@echo off
cd /d "%~dp0"
where py >nul 2>&1
if errorlevel 1 goto python
py -3 tools\udp_quality_receiver\dashboard_server.py --http-port 8775 --open
goto done
:python
python tools\udp_quality_receiver\dashboard_server.py --http-port 8775 --open
:done
if errorlevel 1 pause
