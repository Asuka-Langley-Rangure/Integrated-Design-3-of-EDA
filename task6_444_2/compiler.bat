@echo off
setlocal enabledelayedexpansion

rem One-click build script for MyPlacement (Windows, CMake).
rem Usage: double-click or run `compiler.bat [Debug|Release]`.
rem Optionally set GENERATOR to override the CMake generator.

set "PROJECT_ROOT=%~dp0"
if "%PROJECT_ROOT:~-1%"=="\" set "PROJECT_ROOT=%PROJECT_ROOT:~0,-1%"
set "BUILD_DIR=%PROJECT_ROOT%\build"
set "BUILD_TYPE=Release"
if not "%~1"=="" set "BUILD_TYPE=%~1"

set "GEN_ARG="
if defined GENERATOR set "GEN_ARG=-G \"%GENERATOR%\""

echo [Build] Root: %PROJECT_ROOT%
echo [Build] Type: %BUILD_TYPE%
if defined GENERATOR echo [Build] Generator override: %GENERATOR%

where cmake >nul 2>&1
if errorlevel 1 (
  echo [Error] CMake not found in PATH.
  exit /b 1
)

rem Clean build dir
if exist "%BUILD_DIR%" (
  echo [Clean] Removing old build directory...
  rmdir /s /q "%BUILD_DIR%"
)

echo [Configure] Running CMake...
cmake -S "%PROJECT_ROOT%" -B "%BUILD_DIR%" -DCMAKE_BUILD_TYPE=%BUILD_TYPE% %GEN_ARG%
if errorlevel 1 goto :fail

set "PARALLEL=%NUMBER_OF_PROCESSORS%"
if "%PARALLEL%"=="" set "PARALLEL=1"
echo [Compile] Building with %PARALLEL% parallel jobs...
cmake --build "%BUILD_DIR%" --config %BUILD_TYPE% --parallel %PARALLEL%
if errorlevel 1 goto :fail

rem ---- Find the built binary (prefer config subdir first for VS multi-config) ----
set "BIN_PATH="

if exist "%BUILD_DIR%\%BUILD_TYPE%\myplace.exe" set "BIN_PATH=%BUILD_DIR%\%BUILD_TYPE%\myplace.exe"
if not defined BIN_PATH if exist "%BUILD_DIR%\%BUILD_TYPE%\myplace" set "BIN_PATH=%BUILD_DIR%\%BUILD_TYPE%\myplace"

if not defined BIN_PATH if exist "%BUILD_DIR%\myplace.exe" set "BIN_PATH=%BUILD_DIR%\myplace.exe"
if not defined BIN_PATH if exist "%BUILD_DIR%\myplace" set "BIN_PATH=%BUILD_DIR%\myplace"

if not defined BIN_PATH (
  for /f "delims=" %%i in ('dir /b /s "%BUILD_DIR%\myplace.exe" 2^>nul') do (
    set "BIN_PATH=%%i"
    goto :bin_found
  )
  for /f "delims=" %%i in ('dir /b /s "%BUILD_DIR%\myplace" 2^>nul') do (
    set "BIN_PATH=%%i"
    goto :bin_found
  )
)

:bin_found
if not defined BIN_PATH (
  echo [Warn] Build finished but myplace binary was not found.
  goto :done
)

echo [Output] Built binary: %BIN_PATH%

rem (Optional) Copy to project root for convenience
set "TARGET_BIN=%PROJECT_ROOT%\myplace.exe"
copy /y "%BIN_PATH%" "%TARGET_BIN%" >nul
if errorlevel 1 (
  echo [Warn] Copy failed, will run from build output directly.
  set "TARGET_BIN=%BIN_PATH%"
) else (
  echo [Output] Binary copied to %TARGET_BIN%
)

rem Copy compile_commands.json if present
if exist "%BUILD_DIR%\compile_commands.json" (
  copy /y "%BUILD_DIR%\compile_commands.json" "%PROJECT_ROOT%\compile_commands.json" >nul
  echo [Info] compile_commands.json updated.
)

rem ---- Run if build succeeded ----
echo [Run] Launching: %TARGET_BIN%
pushd "%PROJECT_ROOT%" >nul
"%TARGET_BIN%"
set "APP_EXIT=%ERRORLEVEL%"
popd >nul
echo [Run] myplace exited with code %APP_EXIT%.

:done
echo [Done] Build completed.
exit /b 0

:fail
echo [Error] Build failed.
exit /b 1
