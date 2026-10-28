#!/bin/bash
# Start UR5e Sim.command — double-click launcher (macOS).
# Starts the simulator + 3D viewer and opens both in the browser.
# Same as typing in a terminal in this folder:
#     docker compose up -d ursim viewer
# The terminal route (docs/SETUP.md) is the primary one; this is a convenience.
cd "$(dirname "$0")" || exit 1

if ! docker info >/dev/null 2>&1; then
  echo "Docker Desktop is not running. Start it (wait for the whale icon to go steady) and double-click again."
  echo
  read -r -p "Press Enter to close."
  exit 1
fi

echo "Starting URSim + viewer (first time: downloads ~3 GB, be patient) ..."
if ! docker compose up -d ursim viewer; then
  echo
  echo "docker compose failed - see the message above."
  read -r -p "Press Enter to close."
  exit 1
fi

echo "Waiting for the simulator web UI ..."
for _ in $(seq 1 60); do
  curl -s -o /dev/null http://localhost:6080/vnc.html && break
  sleep 2
done

open "http://localhost:6080/vnc.html"
sleep 2
open "http://localhost:7001/"

echo
echo "In the simulator tab: Connect -> power button (bottom left) -> ON -> START."
echo "Viewer: http://localhost:7001/   (shows 'connecting' until the robot is ON)."
echo "Then open a terminal here and run:  docker compose run --rm dev bash"
echo
read -r -p "Press Enter to close this window (the simulator keeps running)."
