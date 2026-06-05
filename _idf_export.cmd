@echo off
REM Ortak ESP-IDF ortami (setlocal KULLANMA — export PATH degiskenleri kalici olmali)
if exist "%~dp0idf_path.local.cmd" call "%~dp0idf_path.local.cmd"
if not defined IDF_PATH set "IDF_PATH=C:\Espressif\frameworks\esp-idf-v5.5.2"
if not exist "%IDF_PATH%\export.bat" (
  echo [HATA] export.bat bulunamadi: %IDF_PATH%
  exit /b 1
)
call "%IDF_PATH%\export.bat"
exit /b %ERRORLEVEL%
