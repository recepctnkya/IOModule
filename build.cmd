@echo off
REM Derleme: idf.py build
setlocal
cd /d "%~dp0"
call "%~dp0_idf_export.cmd" || exit /b 1
echo [build] %CD%
idf.py build
set "EC=%ERRORLEVEL%"
if not "%EC%"=="0" (
  echo [build] BASARISIZ (%EC%)
  exit /b %EC%
)
set "REL_BIN="
for %%F in (build\vango_medium_v*.bin) do set "REL_BIN=%%F"
if defined REL_BIN (
  echo [build] Platform dosyasi: %REL_BIN%
) else (
  echo [build] UYARI: build\vango_medium_v*.bin yok — CMake POST_BUILD kontrol edin.
)
exit /b 0
