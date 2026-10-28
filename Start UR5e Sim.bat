@echo off
REM Start UR5e Sim.bat — double-click launcher (Windows).
REM Starts the simulator + 3D viewer and opens both in the browser.
REM Same as typing in a terminal in this folder:
REM     docker compose up -d ursim viewer
REM The terminal route (docs\SETUP.md) is the primary one; this is a convenience.
cd /d "%~dp0"

docker info >nul 2>&1
if errorlevel 1 (
  echo Docker Desktop is not running. Start it, wait for the whale icon to go steady, then double-click again.
  echo.
  pause
  exit /b 1
)

echo Starting URSim + viewer (first time: downloads ~3 GB, be patient) ...
docker compose up -d ursim viewer
if errorlevel 1 (
  echo.
  echo docker compose failed - see the message above.
  pause
  exit /b 1
)

echo Waiting for the simulator web UI ...
set /a tries=0
:wait
curl -s -o nul http://localhost:6080/vnc.html && goto ready
set /a tries+=1
if %tries% geq 60 goto ready
timeout /t 2 /nobreak >nul
goto wait

:ready
start "" "http://localhost:6080/vnc.html"
timeout /t 2 /nobreak >nul
start "" "http://localhost:7001/"

echo.
echo In the simulator tab: Connect -^> power button (bottom left) -^> ON -^> START.
echo Viewer: http://localhost:7001/   (shows 'connecting' until the robot is ON).
echo Then open a terminal here and run:  docker compose run --rm dev bash
echo.
pause
