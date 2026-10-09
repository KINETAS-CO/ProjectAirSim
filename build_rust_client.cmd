@echo off
REM Copyright (C) Microsoft Corporation.
REM Copyright (C) 2025 IAMAI CONSULTING CORP
REM MIT License.

setlocal EnableExtensions EnableDelayedExpansion

set "BUILD_TYPE=debug"
set "RUN_TESTS=OFF"

:parse_args
if "%~1"=="" goto args_done
if /I "%~1"=="debug" (
  set "BUILD_TYPE=debug"
  shift
  goto parse_args
)
if /I "%~1"=="release" (
  set "BUILD_TYPE=release"
  shift
  goto parse_args
)
if /I "%~1"=="--tests" (
  set "RUN_TESTS=ON"
  shift
  goto parse_args
)
if /I "%~1"=="--test" (
  set "RUN_TESTS=ON"
  shift
  goto parse_args
)
if /I "%~1"=="--help" goto usage
if /I "%~1"=="-h" goto usage
echo Unknown argument: %~1
goto usage_error

:args_done
set "ROOT_DIR=%~dp0"
set "MANIFEST_PATH=%ROOT_DIR%client\rust\Cargo.toml"

if exist "%USERPROFILE%\.cargo\bin\cargo.exe" (
  set "PATH=%USERPROFILE%\.cargo\bin;!PATH!"
)

where /q cargo
if errorlevel 1 (
  echo [ERROR] cargo was not found. Please install Rust from https://rustup.rs/
  exit /b 1
)

set "BUILD_FLAGS="
if /I "%BUILD_TYPE%"=="release" (
  set "BUILD_FLAGS=--release"
)

echo Building ProjectAirSim Rust Client (async)...
cargo build --manifest-path "%MANIFEST_PATH%" --no-default-features --features async !BUILD_FLAGS!
if errorlevel 1 exit /b 1

echo Building ProjectAirSim Rust Client (sync)...
cargo build --manifest-path "%MANIFEST_PATH%" --no-default-features --features sync !BUILD_FLAGS!
if errorlevel 1 exit /b 1

if /I "%RUN_TESTS%"=="ON" (
  cargo nextest --version >nul 2>&1
  if not errorlevel 1 (
    set "CI=1"
    echo Running async tests with nextest...
    cargo nextest run --manifest-path "%MANIFEST_PATH%" --no-default-features --features async !BUILD_FLAGS!
    if errorlevel 1 exit /b 1
    echo Running sync tests with nextest...
    cargo nextest run --manifest-path "%MANIFEST_PATH%" --no-default-features --features sync !BUILD_FLAGS!
    if errorlevel 1 exit /b 1
  ) else (
    echo Running async tests with cargo test...
    cargo test --manifest-path "%MANIFEST_PATH%" --no-default-features --features async !BUILD_FLAGS!
    if errorlevel 1 exit /b 1
    echo Running sync tests with cargo test...
    cargo test --manifest-path "%MANIFEST_PATH%" --no-default-features --features sync !BUILD_FLAGS!
    if errorlevel 1 exit /b 1
  )
)

exit /b 0

:usage
echo Usage: build_rust_client.cmd [debug^|release] [--tests]
echo.
echo Build the standalone ProjectAirSim Rust client without building SimLibs.
echo   debug       Build Debug artifacts (default).
echo   release     Build Release artifacts.
echo   --tests     Build and run the mocked unit tests. A simulator is not required.
exit /b 0

:usage_error
call :usage
exit /b 2
