@echo off
REM USB flash — esptool dogrudan (idf.py flash takilirsa). Varsayilan COM40.
setlocal
cd /d "%~dp0"
call "%~dp0_idf_export.cmd" || exit /b 1
set "APP_BIN=build\mqtt_tcp_custom_outbox.bin"
if exist "build\vango_medium_v0_50.bin" set "APP_BIN=build\vango_medium_v0_50.bin"
if not exist "%APP_BIN%" (
  echo [flash_usb] build yok, derleniyor...
  idf.py build || exit /b 1
  if exist "build\vango_medium_v0_50.bin" set "APP_BIN=build\vango_medium_v0_50.bin"
)
set "PORT=COM40"
if not "%~1"=="" set "PORT=%~1"
for %%F in ("%APP_BIN%") do set "APP_NAME=%%~nxF"
cd build
echo Flashing %APP_NAME% to %PORT% at 460800. Hold BOOT if connect fails.
set ATTEMPTS=0
:retry
set /a ATTEMPTS+=1
if %ATTEMPTS% GTR 10 (
  echo Flash failed after 10 attempts.
  exit /b 1
)
echo --- attempt %ATTEMPTS% ---
python -m esptool --chip esp32s3 -p %PORT% -b 460800 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_freq 40m --flash_size 8MB 0x0 bootloader/bootloader.bin 0x8000 partition_table/partition-table.bin 0xf000 ota_data_initial.bin 0x20000 %APP_NAME%
if errorlevel 1 goto retry
echo Flash OK.
exit /b 0
