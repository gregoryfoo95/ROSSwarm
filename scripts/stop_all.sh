#!/usr/bin/env bash
set -euo pipefail

# Try to clean up stray sim processes
pkill -f sim_vehicle.py || true
pkill -f ArduCopter.elf || true
pkill -f gz || true
echo "Requested termination of SITL and Gazebo if running."
