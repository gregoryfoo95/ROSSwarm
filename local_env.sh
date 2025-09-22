#!/bin/bash
# ROSSwarm Environment Setup - Fresh Start for QGC Connection

# ROS 2 Humble setup
source /opt/ros/humble/setup.bash

# Gazebo Garden setup
export GZ_VERSION=garden

# ArduPilot Gazebo plugin paths
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/dev/ROSSwarm/_deps/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export GZ_SIM_RESOURCE_PATH="$HOME/dev/ROSSwarm/_deps/ardupilot_gazebo/models:$HOME/dev/ROSSwarm/_deps/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH"

# Add ArduPilot tools to PATH
export PATH="$HOME/dev/ROSSwarm/_deps/ardupilot/Tools/autotest:$PATH"
export PATH="$HOME/.local/bin:$PATH"

# Convenience aliases
alias qgc='./QGroundControl-x86_64.AppImage'

echo "✅ ROSSwarm environment loaded"
echo "📁 Project: $(pwd)"
echo "🚁 ArduPilot tools in PATH"
echo "🎮 QGC alias ready: 'qgc'"