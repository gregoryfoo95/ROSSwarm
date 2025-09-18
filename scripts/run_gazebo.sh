#!/usr/bin/env bash
set -euo pipefail

# Start Gazebo with an empty world; you can later replace with
# ArduPilot-provided worlds or custom SDF that includes ardupilot bridge plugins.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD="$ROOT/worlds/empty_world.sdf"

# Check for available Gazebo commands in order of preference
if command -v gz >/dev/null 2>&1; then
    echo "[gazebo] Starting Gazebo with 'gz sim'..."
    gz sim "$WORLD"
elif command -v ign >/dev/null 2>&1; then
    echo "[gazebo] Starting Gazebo with 'ign gazebo'..."
    ign gazebo "$WORLD"
elif command -v gazebo >/dev/null 2>&1; then
    echo "[gazebo] Starting Gazebo with 'gazebo'..."
    gazebo "$WORLD"
else
    echo "ERROR: Gazebo not found. Please install Gazebo Fortress, Garden, or Classic." >&2
    echo "For Ubuntu 22.04, you can install Gazebo Fortress with:" >&2
    echo "  sudo apt update" >&2
    echo "  sudo apt install lsb-release wget gnupg" >&2
    echo "  sudo wget -qO /usr/share/keyrings/gazebo-archive-keyring.gpg https://packages.osrfoundation.org/gazebo.gpg" >&2
    echo "  echo \"deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu \$(. /etc/os-release && echo \$UBUNTU_CODENAME) stable\" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null" >&2
    echo "  sudo apt update" >&2
    echo "  sudo apt install gz-fortress" >&2
    exit 1
fi
