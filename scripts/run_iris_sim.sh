#!/usr/bin/env bash
set -eo pipefail

# Integrated ArduCopter SITL + Gazebo simulation launcher
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARDU="$ROOT/_deps/ardupilot"
WORLD="$ROOT/worlds/iris_runway.sdf"

echo "[iris-sim] Starting integrated ArduCopter SITL + Gazebo simulation..."

# Source the environment to ensure all paths are set
source "$ROOT/local_env.sh"

# Verify plugin paths
if [ ! -d "$ROOT/_deps/ardupilot_gazebo/build" ]; then
    echo "ERROR: ArduPilot Gazebo plugin not built. Run 'make install' first." >&2
    exit 1
fi

# Check if ArduPilot is built
if [ ! -f "$ARDU/build/sitl/bin/arducopter" ]; then
    echo "ERROR: ArduCopter SITL not built. Run 'make build' first." >&2
    exit 1
fi

# Check if world file exists
if [ ! -f "$WORLD" ]; then
    echo "ERROR: World file not found: $WORLD" >&2
    exit 1
fi

# Function to cleanup background processes
cleanup() {
    echo "[iris-sim] Cleaning up..."
    jobs -p | xargs -r kill
    exit 0
}
trap cleanup SIGINT SIGTERM

# Print environment info for debugging
echo "[iris-sim] Gazebo version: $GZ_VERSION"
echo "[iris-sim] Plugin path: $GZ_SIM_SYSTEM_PLUGIN_PATH"
echo "[iris-sim] Resource path: $GZ_SIM_RESOURCE_PATH"

# Start Gazebo with GUI in background
echo "[iris-sim] Starting Gazebo with Iris world and GUI..."
echo "[iris-sim] World file: $WORLD"
echo "[iris-sim] Display: $DISPLAY"
gz sim "$WORLD" --verbose --gui-config &
GAZEBO_PID=$!

# Wait for Gazebo to initialize and load the world
echo "[iris-sim] Waiting for Gazebo to initialize..."
sleep 10

# Start ArduCopter SITL with custom home location
echo "[iris-sim] Starting ArduCopter SITL..."
cd "$ARDU"
./Tools/autotest/sim_vehicle.py \
  --vehicle ArduCopter \
  --frame gazebo-iris \
  --out 127.0.0.1:14550 \
  -w \
  -L 3DRBerkeley

# Wait for background processes
wait