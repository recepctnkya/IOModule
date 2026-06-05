@echo off
REM Temiz derleme: idf.py fullclean
setlocal
cd /d "%~dp0"
call "%~dp0_idf_export.cmd" || exit /b 1
idf.py fullclean
exit /b %ERRORLEVEL%
