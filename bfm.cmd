@echo off
REM Build + Flash + Monitor (tek komut)
setlocal
cd /d "%~dp0"
call "%~dp0_idf_export.cmd" || exit /b 1
set "PORT=COM40"
if not "%~1"=="" set "PORT=%~1"
echo [bfm] build...
idf.py build || exit /b 1
echo [bfm] flash %PORT%...
idf.py -p %PORT% -b 460800 flash || exit /b 1
echo [bfm] monitor %PORT% (cikis: Ctrl+])
idf.py -p %PORT% monitor
exit /b %ERRORLEVEL%
