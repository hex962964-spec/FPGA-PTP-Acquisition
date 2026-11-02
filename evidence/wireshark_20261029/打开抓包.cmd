@echo off
start "" "D:\1.ruanjian\9.wireshark\Wireshark.exe" -r "%~dp0fpga_capture_20261029.pcapng" -X "lua_script:%~dp0fpga_udp_v2.lua" -Y "udp.dstport==6000" -g 1 -o "gui.qt.font_name:Consolas,14,-1,5,50,0,0,0,0,0"
