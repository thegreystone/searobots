@echo off
setlocal EnableExtensions DisableDelayedExpansion

rem Generates the SeaRobots showreel on Windows (works from cmd.exe and PowerShell).
rem Builds the project (unless --skip-build is passed) and runs se.hirt.searobots.viewer.Showreel.

for %%I in ("%~dp0..") do set "REPO_ROOT=%%~fI"
set "VIEWER_TARGET=%REPO_ROOT%\searobots-viewer\target"
set "DEFAULT_OUT=%REPO_ROOT%\target\showreel\searobots-showreel.mp4"

set "SKIP_BUILD=0"
set "CLEAN_BUILD=0"
set "HAS_OUT=0"
set "HAS_FFMPEG=0"
set "HAS_FRAMES=0"
set "HAS_PLAN_ONLY=0"
set "SHOWREEL_ARGS="

:parse_args
if "%~1"=="" if [%1]==[] goto :args_done
if /i "%~1"=="--help" goto :show_help
if /i "%~1"=="-h" goto :show_help
if "%~1"=="/?" goto :show_help
if /i "%~1"=="--skip-build" (
    set "SKIP_BUILD=1"
    shift
    goto :parse_args
)
if /i "%~1"=="--clean" (
    set "CLEAN_BUILD=1"
    shift
    goto :parse_args
)
if /i "%~1"=="--out" set "HAS_OUT=1"
if /i "%~1"=="--ffmpeg" set "HAS_FFMPEG=1"
if /i "%~1"=="--frames" set "HAS_FRAMES=1"
if /i "%~1"=="--plan-only" set "HAS_PLAN_ONLY=1"
set SHOWREEL_ARGS=%SHOWREEL_ARGS% "%~1"
shift
goto :parse_args

:args_done
if "%CLEAN_BUILD%"=="1" if "%SKIP_BUILD%"=="1" (
    >&2 echo Error: --clean and --skip-build cannot be used together.
    exit /b 1
)

rem -- Locate Java 25+ (prefer JAVA_HOME over PATH) --
set "JAVA_EXE="
if defined JAVA_HOME (
    set "JH=%JAVA_HOME:"=%"
    call :resolve_java_home
)
if not defined JAVA_EXE (
    for /f "delims=" %%J in ('where java.exe 2^>nul') do (
        if not defined JAVA_EXE set "JAVA_EXE=%%J"
    )
)
if not defined JAVA_EXE (
    >&2 echo Error: Java 25 or newer was not found. Set JAVA_HOME or add java.exe to PATH.
    exit /b 1
)

set "JAVA_VER="
for /f "tokens=1-3" %%A in ('""%JAVA_EXE%" -version 2>&1"') do (
    if /i "%%B"=="version" if not defined JAVA_VER set "JAVA_VER=%%~C"
)
set "JAVA_MAJOR="
if defined JAVA_VER (
    for /f "tokens=1 delims=.-_" %%M in ("%JAVA_VER%") do set "JAVA_MAJOR=%%M"
)
if not defined JAVA_MAJOR goto :check_build
echo %JAVA_MAJOR%| findstr /r "^[0-9][0-9]*$" >nul
if errorlevel 1 goto :check_build
if %JAVA_MAJOR% geq 25 goto :check_build
>&2 echo Error: Java 25 or newer is required (found %JAVA_VER% at "%JAVA_EXE%").
>&2 echo Set JAVA_HOME to a JDK 25+ installation and try again.
exit /b 1

rem -- Build the project unless --skip-build was given --
:check_build
if "%SKIP_BUILD%"=="1" goto :find_jar

set "MVN_CMD="
if exist "%REPO_ROOT%\mvnw.cmd" (
    set "MVN_CMD=%REPO_ROOT%\mvnw.cmd"
) else (
    for /f "delims=" %%M in ('where mvn.cmd 2^>nul') do (
        if not defined MVN_CMD set "MVN_CMD=%%M"
    )
)
if defined MVN_CMD goto :run_build
>&2 echo Error: Maven (mvn.cmd) was not found on PATH. Install Maven 3.9+ or use --skip-build.
exit /b 1

:run_build

set "MVN_GOALS=install -DskipTests"
if "%CLEAN_BUILD%"=="1" set "MVN_GOALS=clean install -DskipTests"

echo Building SeaRobots (mvn %MVN_GOALS%)...
pushd "%REPO_ROOT%"
call "%MVN_CMD%" %MVN_GOALS%
set "MVN_EXIT=%ERRORLEVEL%"
popd
if not "%MVN_EXIT%"=="0" (
    >&2 echo Error: Maven build failed with exit code %MVN_EXIT%.
    exit /b %MVN_EXIT%
)

rem -- Locate the shaded viewer JAR --
:find_jar
set "VIEWER_JAR="
if exist "%VIEWER_TARGET%" (
    for /f "delims=" %%F in ('dir /b /o-d "%VIEWER_TARGET%\searobots-viewer-*.jar" 2^>nul') do (
        call :consider_jar "%VIEWER_TARGET%\%%F" "%%F"
    )
)
if not defined VIEWER_JAR (
    >&2 echo Error: Could not find searobots-viewer-*.jar in "%VIEWER_TARGET%".
    >&2 echo Run without --skip-build to build the project first.
    exit /b 1
)

rem -- Locate ffmpeg if rendering video --
if "%HAS_FFMPEG%"=="1" goto :run
if "%HAS_FRAMES%"=="1" goto :run
if "%HAS_PLAN_ONLY%"=="1" goto :run

set "FOUND_FFMPEG="
if defined FFMPEG (
    set "FFMPEG_UNQUOTED=%FFMPEG:"=%"
    call :check_env_ffmpeg
)
if defined FOUND_FFMPEG goto :run

for /f "delims=" %%F in ('where ffmpeg.exe 2^>nul') do (
    if not defined FOUND_FFMPEG set "FOUND_FFMPEG=%%F"
)
if defined FOUND_FFMPEG goto :run

rem Check common Windows package manager install locations (e.g. winget before terminal restart)
if defined LOCALAPPDATA if exist "%LOCALAPPDATA%\Microsoft\WinGet\Links\ffmpeg.exe" (
    set "FOUND_FFMPEG=%LOCALAPPDATA%\Microsoft\WinGet\Links\ffmpeg.exe"
)
if not defined FOUND_FFMPEG if defined USERPROFILE if exist "%USERPROFILE%\scoop\shims\ffmpeg.exe" (
    set "FOUND_FFMPEG=%USERPROFILE%\scoop\shims\ffmpeg.exe"
)
if not defined FOUND_FFMPEG if defined ProgramData if exist "%ProgramData%\chocolatey\bin\ffmpeg.exe" (
    set "FOUND_FFMPEG=%ProgramData%\chocolatey\bin\ffmpeg.exe"
)
if not defined FOUND_FFMPEG if defined LOCALAPPDATA if exist "%LOCALAPPDATA%\Microsoft\WinGet\Packages" (
    for /d %%D in ("%LOCALAPPDATA%\Microsoft\WinGet\Packages\*FFmpeg*") do (
        if not defined FOUND_FFMPEG (
            for /f "delims=" %%F in ('dir /b /s "%%D\ffmpeg.exe" 2^>nul') do (
                if not defined FOUND_FFMPEG set "FOUND_FFMPEG=%%F"
            )
        )
    )
)

if not defined FOUND_FFMPEG goto :warn_no_ffmpeg
set "FFMPEG=%FOUND_FFMPEG%"
echo Found ffmpeg: "%FOUND_FFMPEG%"
goto :run

:warn_no_ffmpeg
>&2 echo Warning: ffmpeg was not found on PATH or in FFMPEG.
>&2 echo Showreel will write PNG frames instead of an MP4 video.
>&2 echo Install ffmpeg with: winget install Gyan.FFmpeg
>&2 echo Or pass --ffmpeg PATH or set the FFMPEG environment variable.
>&2 echo.

:run
if "%HAS_OUT%"=="0" set SHOWREEL_ARGS=--out "%DEFAULT_OUT%" %SHOWREEL_ARGS%
if not defined SHOWREEL_JVM_OPTS set "SHOWREEL_JVM_OPTS=-XX:+UseZGC -Xmx4g"

"%JAVA_EXE%" %SHOWREEL_JVM_OPTS% -cp "%VIEWER_JAR%" se.hirt.searobots.viewer.Showreel %SHOWREEL_ARGS%
exit /b %ERRORLEVEL%

:resolve_java_home
if not defined JH exit /b 0
if "%JH:~-1%"=="\" set "JH=%JH:~0,-1%"
if exist "%JH%\bin\java.exe" set "JAVA_EXE=%JH%\bin\java.exe"
exit /b 0

:check_env_ffmpeg
if not defined FFMPEG_UNQUOTED exit /b 0
if exist "%FFMPEG_UNQUOTED%" (
    set "FOUND_FFMPEG=%FFMPEG_UNQUOTED%"
) else (
    >&2 echo Warning: FFMPEG environment variable points to a missing file: "%FFMPEG_UNQUOTED%".
)
exit /b 0

:consider_jar
if defined VIEWER_JAR exit /b 0
set "CANDIDATE_NAME=%~2"
if /i not "%CANDIDATE_NAME:-sources.jar=%"=="%CANDIDATE_NAME%" exit /b 0
if /i not "%CANDIDATE_NAME:-javadoc.jar=%"=="%CANDIDATE_NAME%" exit /b 0
if /i not "%CANDIDATE_NAME:-tests.jar=%"=="%CANDIDATE_NAME%" exit /b 0
if /i not "%CANDIDATE_NAME:-shaded.jar=%"=="%CANDIDATE_NAME%" exit /b 0
set "VIEWER_JAR=%~1"
exit /b 0

:show_help
echo Usage: scripts\showreel.cmd [script options] [showreel options]
echo.
echo Builds SeaRobots and generates the showreel on Windows.
echo Works from Command Prompt (cmd.exe) and PowerShell.
echo.
echo Script options:
echo   --skip-build      reuse the existing viewer JAR without running Maven
echo   --clean           run a clean Maven build (mvn clean install -DskipTests)
echo   -h, --help        show this help and exit
echo.
echo Showreel options (forwarded to se.hirt.searobots.viewer.Showreel):
echo   --out FILE        video to write (default target\showreel\searobots-showreel.mp4)
echo   --replay FILE     film this .srl replay instead of scouting
echo   --seed HEX        simulate and film this seed instead of scouting
echo   --ships A:B       submarines to fight: claude, codex, gemini, default (default claude:codex)
echo   --scout N         seeds whose terrain to survey (default 64)
echo   --matches N       of those, how many to simulate (default 16)
echo   --master HEX      seed for the scouting draw (default 5eab07)
echo   --length SECONDS  target length (default 85)
echo   --size WxH        frame size (default 1920x960, a 2:1 cinemascope frame)
echo   --fps N           frame rate (default 60)
echo   --crf N           x264 quality, lower is better (default 20)
echo   --time HH:MM      time of day for the light (default 15:00)
echo   --ffmpeg PATH     ffmpeg executable
echo   --frames DIR      write PNG frames instead of a video
echo   --plan-only       scout and plan, print the plan, render nothing
echo.
echo Environment variables:
echo   JAVA_HOME          JDK 25+ directory (preferred over java.exe on PATH)
echo   FFMPEG             path to ffmpeg.exe (if not on PATH)
echo   SHOWREEL_JVM_OPTS  JVM flags (default: -XX:+UseZGC -Xmx4g)
echo.
echo Examples:
echo   scripts\showreel.cmd
echo   scripts\showreel.cmd --seed 574e4f1ec676d60a
echo   scripts\showreel.cmd --skip-build --plan-only --seed 574e4f1ec676d60a
echo   scripts\showreel.cmd --skip-build --out my-reel.mp4 --length 30
exit /b 0
