@echo off
rem Builds tactics_poc.exe on Windows with Visual Studio's C++ build tools, CMake and Ninja, into build\.
rem The sources are laid out as on Linux: motion\ and gta2_clone\third_party\WickedEngine\ next to tactics_poc\.

setlocal
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -find VC\Auxiliary\Build\vcvars64.bat`) do set "VCVARS=%%i"
if not defined VCVARS (
    echo Visual Studio with the C++ build tools was not found.
    exit /b 1
)
call "%VCVARS%" >nul || exit /b 1

set "ROOT=%~dp0.."
cmake -S "%ROOT%" -B "%ROOT%\build" -G Ninja -DCMAKE_BUILD_TYPE=Release || exit /b 1
cmake --build "%ROOT%\build" --target tactics_poc || exit /b 1
echo Built %ROOT%\build\tactics_poc.exe
