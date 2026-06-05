@echo off
REM Seri monitor: idf.py -p PORT monitor
setlocal
cd /d "%~dp0"
call "%~dp0_idf_export.cmd" || exit /b 1
call "%~dp0_idf_port.cmd" %*
set "PORT=%HEXNET_FLASH_PORT%"
echo [monitor] %PORT% — cikis: Ctrl+]
idf.py -p %PORT% monitor
exit /b %ERRORLEVEL%
