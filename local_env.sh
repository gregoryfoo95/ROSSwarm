#!/usr/bin/env bash
# Local environment setup for ROSSwarm (safe under set -u)
# Gazebo version
export GZ_VERSION=${GZ_VERSION:-garden}

# ArduPilot Gazebo plugin + resources
AP_GZ_BUILD="$HOME/dev/ROSSwarm/_deps/ardupilot_gazebo/build"
AP_GZ_MODELS="$HOME/dev/ROSSwarm/_deps/ardupilot_gazebo/models"
AP_GZ_WORLDS="$HOME/dev/ROSSwarm/_deps/ardupilot_gazebo/worlds"

# Plugin search path
export GZ_SIM_SYSTEM_PLUGIN_PATH="${AP_GZ_BUILD}:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

# Resource search paths (models + worlds). Mirror to legacy var for safety.
export GZ_SIM_RESOURCE_PATH="${AP_GZ_MODELS}:${AP_GZ_WORLDS}:${GZ_SIM_RESOURCE_PATH:-}"
export IGN_GAZEBO_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}"
export GZ_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}"   # some tools look at this

# (Optional) classic variable for tools that still check it
export GAZEBO_MODEL_PATH="${AP_GZ_MODELS}:${GAZEBO_MODEL_PATH:-}"
