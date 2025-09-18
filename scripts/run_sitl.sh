#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEPS="$ROOT/_deps"
ARDU="$DEPS/ardupilot"
LOGDIR="$DEPS/ardupilot_logs"
mkdir -p "$LOGDIR"

# Ensure profile updates from ArduPilot installer are active
# shellcheck disable=SC1091
. ~/.profile || true

if [ ! -d "$ARDU" ]; then
  echo "ArduPilot not found. Run scripts/bootstrap.sh first." >&2
  exit 1
fi

pushd "$ARDU" >/dev/null

# Minimal SITL with Gazebo transport using default IRIS model (ArduCopter)
# This starts SITL and opens a MAVProxy prompt (UDP 14550) for GCS/bridges.
echo "[sitl] Launching ArduCopter SITL (UDP:14550) ..."
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map \
  --out=udp:127.0.0.1:14550 \
  --aircraft="$LOGDIR"

popd >/dev/null || true
