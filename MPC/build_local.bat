@echo off
cd /d "%~dp0"
set GCC=D:\ming\ucrt64\bin\gcc.exe
set PATH=D:\ming\ucrt64\bin;%PATH%
set CFLAGS=-std=c99 -Wall -Wextra -Wpedantic -Iinclude

if not exist build_local mkdir build_local

echo ============================================
echo   MPC Core Library Build
echo ============================================
echo.

echo [CC] fixed_point.c
%GCC% %CFLAGS% -c src\fixed_point.c -o build_local\fixed_point.o
if %ERRORLEVEL% NEQ 0 (echo   FAILED!) else (echo   OK)

echo [CC] linear_algebra.c
%GCC% %CFLAGS% -c src\linear_algebra.c -o build_local\linear_algebra.o
if %ERRORLEVEL% NEQ 0 (echo   FAILED!) else (echo   OK)

echo [CC] qp_solver.c
%GCC% %CFLAGS% -c src\qp_solver.c -o build_local\qp_solver.o
if %ERRORLEVEL% NEQ 0 (echo   FAILED!) else (echo   OK)

echo [CC] vehicle_model.c
%GCC% %CFLAGS% -c src\vehicle_model.c -o build_local\vehicle_model.o
if %ERRORLEVEL% NEQ 0 (echo   FAILED!) else (echo   OK)

echo [CC] mpc.c
%GCC% %CFLAGS% -c src\mpc.c -o build_local\mpc.o
if %ERRORLEVEL% NEQ 0 (echo   FAILED!) else (echo   OK)

echo.
echo --- Checking outputs ---
dir build_local\*.o 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo No .o files produced!
) else (
    echo.
    echo ============================================
    echo   Build Complete!
    echo ============================================
)
