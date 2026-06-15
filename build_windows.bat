@echo off
setlocal

set "ROOT=%~dp0"
if "%ROOT:~-1%"=="\" set "ROOT=%ROOT:~0,-1%"
set "CMAKE=C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"

if not exist "%CMAKE%" (
  echo CMake not found: %CMAKE%
  exit /b 1
)

"%CMAKE%" -S "%ROOT%" -B "%ROOT%\build-win" -G "Visual Studio 17 2022" -A x64
if errorlevel 1 exit /b %errorlevel%

"%CMAKE%" --build "%ROOT%\build-win" --config Release --target ah64d_auto_rudder
if errorlevel 1 exit /b %errorlevel%

echo Built: %ROOT%\build-win\Release\ah64d_auto_rudder.exe
