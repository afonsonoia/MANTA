@echo off
title MANTA FPV — Live Video Stream Receiver
cd /d "%~dp0"
echo ========================================================
echo   MANTA Companion Computer - SSH Live Video Stream
echo ========================================================
python stream_receiver.py %*
pause
