#!/usr/bin/env bash
# Bring up PX4 SITL + Gazebo in Docker, streaming MAVLink on UDP 14550.
#
# Usage:
#   ./tools/sitl.sh          # start the simulator (Ctrl+C to stop)
#   ./tools/sitl.sh <args>   # extra args are passed through to the container
#
# Point a ground station at udp://127.0.0.1:14550 once you see
# "INFO [commander] Ready for takeoff!".
set -euo pipefail

IMAGE="jonasvautherin/px4-gazebo-headless:latest"

if ! docker info >/dev/null 2>&1; then
  echo "Docker is not running. Start Docker Desktop and try again." >&2
  exit 1
fi

# Pull once on first use (~2 GB); cached after that.
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "Pulling $IMAGE (first run, ~2 GB)…"
  docker pull "$IMAGE"
fi

echo "Starting PX4 SITL — MAVLink on udp://127.0.0.1:14550 (Ctrl+C to stop)"
# exec so Ctrl+C goes straight to the container; --rm cleans it up on exit.
exec docker run --rm -it \
  -p 14550:14550/udp \
  -p 14556:14556/udp \
  "$IMAGE" "$@"
