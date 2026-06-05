@echo off
REM Hexnet IO - dogru proje klasoru + flash (ESP-IDF CMD icinde calistirin)
setlocal
cd /d "%~dp0"
echo.
echo ============================================
echo  YUKLENECEK PROJE (bu klasor):
echo  %CD%
echo ============================================
echo.
echo  Uygulama: build\mqtt_tcp_custom_outbox.bin
echo  Bootloader / partition / OTA bu klasorde otomatik yazilir.
echo.
set PORT=COM40
set /p PORT=COM portu [Enter=COM40]: 
if "%PORT%"=="" set PORT=COM40
echo.
echo Port: %PORT%
echo.
call "%~dp0flash_usb.cmd" %PORT%
pause
