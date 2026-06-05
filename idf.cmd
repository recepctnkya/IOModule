@echo off
REM ESP-IDF CMD — IOModule-v0-50 proje klasorunde acik kalir (build/flash/monitor buradan)
setlocal
cd /d "%~dp0"
call "%~dp0_idf_export.cmd" || (pause & exit /b 1)

echo.
echo ============================================
echo  Hexnet IO Module — IOModule-v0-50
echo  %CD%
echo ============================================
echo.
echo  build.cmd              idf.py build
echo  flash.cmd [COM40]      idf.py flash
echo  monitor.cmd [COM40]    idf.py monitor
echo  bfm.cmd [COM40]        build + flash + monitor
echo  YUKLE.cmd              flash (port sorar)
echo  flash_usb.cmd [COM40]  esptool dogrudan (idf.py takilirsa)
echo.
echo  Ornek:  build.cmd
echo          flash.cmd COM40
echo          idf.py -p COM40 erase_flash
echo.
cmd /k
