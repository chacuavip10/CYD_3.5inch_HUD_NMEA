@echo off
setlocal

REM ===== Lấy thư mục chứa file .bat =====
set SCRIPT_DIR=%~dp0

echo Script dir: %SCRIPT_DIR%

REM ===== 1. Xoá thư mục ui cũ =====
if exist "%SCRIPT_DIR%ui" (
    echo Deleting old ui folder...
    rmdir /s /q "%SCRIPT_DIR%ui"
)

REM ===== 2. Đi lên 1 cấp =====
cd /d "%SCRIPT_DIR%.."

REM ===== 3. Đường dẫn source =====
set SRC_DIR=%CD%\cyd_hud_3.5inch_eezstudio\src\ui

echo Source dir: %SRC_DIR%

REM ===== 4. Copy thư mục ui =====
if exist "%SRC_DIR%" (
    echo Copying ui folder...
    xcopy "%SRC_DIR%" "%SCRIPT_DIR%ui" /E /I /Y
) else (
    echo ERROR: Source ui folder not found!
)

echo Done.
pause