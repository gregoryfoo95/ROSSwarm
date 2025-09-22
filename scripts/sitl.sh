#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARDU="$ROOT/_deps/ardupilot"

if [[ ! -x "$ARDU/build/sitl/bin/arducopter" ]]; then
  echo "[SITL] ERROR: arducopter binary not found. Build it first:"
  echo "       cd $ARDU && ./waf configure --board sitl && ./waf copter"
  exit 1
fi

echo "[SITL] Starting (no Gazebo) on tcp:0.0.0.0:5760 …"
cd "$ARDU"
exec ./build/sitl/bin/arducopter \
  --model quad \
  --serial0=tcp:0.0.0.0:5760 \
  -I0
