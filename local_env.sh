#!/bin/bash
# Local environment setup for ROSSwarm

# ROS 2 Humble setup
source /opt/ros/humble/setup.bash

# Gazebo setup
export GZ_VERSION=garden

# ArduPilot Gazebo plugin paths
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/dev/ROSSwarm/_deps/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export GZ_SIM_RESOURCE_PATH="$HOME/dev/ROSSwarm/_deps/ardupilot_gazebo/models:$HOME/dev/ROSSwarm/_deps/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH"

# Add local Python packages to path
export PATH="$HOME/.local/bin:$PATH"

# Convenience aliases
alias gs='gz sim'
alias srcros='source /opt/ros/humble/setup.bash'

echo "ROSSwarm local environment loaded"
