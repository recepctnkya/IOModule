@echo off
REM USB flash: idf.py -p PORT flash (cikti: build\vango_medium_v0_50.bin)
setlocal
cd /d "%~dp0"
call "%~dp0_idf_export.cmd" || exit /b 1
call "%~dp0_idf_port.cmd" %*
set "PORT=%HEXNET_FLASH_PORT%"
if not exist "build\mqtt_tcp_custom_outbox.bin" (
  echo [flash] build yok, once derleniyor...
  idf.py build || exit /b 1
)
echo [flash] Port: %PORT%
idf.py -p %PORT% -b 460800 flash
exit /b %ERRORLEVEL%
