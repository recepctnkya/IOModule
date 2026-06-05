@echo off
REM Varsayilan USB port (port.local.cmd ile override)
set "HEXNET_FLASH_PORT=COM21"
if exist "%~dp0port.local.cmd" call "%~dp0port.local.cmd"
if not "%~1"=="" set "HEXNET_FLASH_PORT=%~1"
