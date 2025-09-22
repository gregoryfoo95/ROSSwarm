#!/usr/bin/env bash
set -euo pipefail

# bootstrap: clone ArduPilot into _deps/, setup env, and (optionally) build SITL
# Usage: scripts/bootstrap.sh [--build-only]
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEPS="$ROOT/_deps"
ARDU="$DEPS/ardupilot"

mkdir -p "$DEPS"

# Ensure ROS and Gazebo env when interactive shells source .bashrc
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi
if ! grep -q "export GZ_VERSION=garden" ~/.bashrc; then
  echo "export GZ_VERSION=garden" >> ~/.bashrc
fi

# Clone ArduPilot if missing
if [ ! -d "$ARDU/.git" ]; then
  echo "[bootstrap] Cloning ArduPilot into $ARDU ..."
  git clone --depth=1 https://github.com/ArduPilot/ardupilot.git "$ARDU"
fi

# Install Python dependencies for ArduPilot
echo "[bootstrap] Installing Python dependencies ..."
python3 -m pip install empy==3.3.4 pexpect pyserial

# Note: ArduPilot system prerequisites should be installed separately with:
# cd _deps/ardupilot && Tools/environment_install/install-prereqs-ubuntu.sh -y

if [[ "${1:-}" == "--build-only" ]]; then
  echo "[bootstrap] Building ArduCopter SITL ..."
  pushd "$ARDU" >/dev/null
    ./waf configure --board sitl
    ./waf copter
  popd >/dev/null
fi

echo "[bootstrap] Done."