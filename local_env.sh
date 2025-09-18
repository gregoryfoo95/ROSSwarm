#!/bin/bash
# Local environment setup for ROSSwarm

# ROS 2 Humble setup
source /opt/ros/humble/setup.bash

# Gazebo setup
export GZ_VERSION=fortress
export IGN_GAZEBO_RESOURCE_PATH="/usr/share/ignition/ignition-gazebo6/worlds:$IGN_GAZEBO_RESOURCE_PATH"
export IGN_GAZEBO_MODEL_PATH="/usr/share/ignition/ignition-gazebo6/models:$IGN_GAZEBO_MODEL_PATH"

# Use Fuel server for models if local ones aren't available
export GZ_FUEL_CACHE_PATH="$HOME/.ignition/fuel"

# Add local Python packages to path
export PATH="$HOME/.local/bin:$PATH"

# Convenience aliases
alias gs='gz sim'
alias srcros='source /opt/ros/humble/setup.bash'

echo "ROSSwarm local environment loaded"
