#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARDU="$ROOT/_deps/ardupilot"
WORLD="$ROOT/worlds/iris_runway.sdf"
LOGDIR="$ROOT/_deps/ardupilot_logs"

echo "[iris-sim] Starting integrated ArduCopter SITL + Gazebo simulation..."
source "$ROOT/local_env.sh"

[[ -d "$ROOT/_deps/ardupilot_gazebo/build" ]] || { echo "ERROR: build ardupilot_gazebo first"; exit 1; }
[[ -f "$ARDU/build/sitl/bin/arducopter"     ]] || { echo "ERROR: build ArduCopter SITL first"; exit 1; }
[[ -f "$WORLD"                              ]] || { echo "ERROR: world not found: $WORLD"; exit 1; }
mkdir -p "$LOGDIR"

gz sim "$WORLD" --verbose -r &
GAZEBO_PID=$!
for i in {1..40}; do gz topic -l 2>/dev/null | grep -q "/world/" && break; sleep 0.5; done

cd "$ARDU"
# **Disable MAVProxy** and make SITL send directly to QGC
./Tools/autotest/sim_vehicle.py \
  --vehicle ArduCopter \
  --frame gazebo-iris \
  --no-mavproxy \
  -A "--serial0=udpclient:127.0.0.1:14550" \
  -L KSFO \
  -w \
  >"$LOGDIR/sim_vehicle.log" 2>&1 &
SITL_PID=$!

echo "[iris-sim] SITL (no MAVProxy) PID: $SITL_PID  | Gazebo PID: $GAZEBO_PID"
echo "[iris-sim] QGC will autoconnect on UDP 127.0.0.1:14550"
wait
