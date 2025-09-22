#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD="${1:-$ROOT/worlds/iris_runway.sdf}"

# Load your ROS/GZ environment here (only this script needs it)
# shellcheck disable=SC1091
source "$ROOT/local_env.sh"

if [[ ! -f "$WORLD" ]]; then
  echo "[GZ] ERROR: world not found: $WORLD"
  exit 1
fi

echo "[GZ] Starting gz sim with world: $WORLD"
exec gz sim "$WORLD" --verbose -r
