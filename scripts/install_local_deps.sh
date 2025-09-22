#!/usr/bin/env bash
set -euo pipefail
set +H

# Local dependency installation script for ROSSwarm
# Installs ROS 2 Humble, Gazebo Fortress, and ArduPilot toolchain locally
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEPS="$ROOT/_deps"

echo "[install] Setting up local dependencies in $DEPS"
mkdir -p "$DEPS"

# Check if running on Ubuntu 22.04
if ! grep -q "Ubuntu 22.04" /etc/os-release 2>/dev/null; then
    echo "Warning: This script is designed for Ubuntu 22.04. Your system may not be compatible."
    read -p "Continue anyway? [y/N] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Update system packages
echo "[install] Updating system packages..."
sudo apt-get update

# Install basic development tools
echo "[install] Installing basic development tools..."
sudo apt-get install -y \
    locales curl wget gnupg2 lsb-release git build-essential cmake \
    python3 python3-pip python3-venv python3-dev \
    software-properties-common apt-transport-https ca-certificates \
    vim tmux bash-completion net-tools iputils-ping

# Set up locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Install ROS 2 Humble
echo "[install] Installing ROS 2 Humble..."
if ! dpkg-query -s ros-humble-desktop >/dev/null 2>&1; then
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update
    sudo apt-get install -y ros-humble-desktop
fi

# Install Gazebo Garden (required for ArduPilot plugin)
echo "[install] Installing Gazebo Garden..."
if ! dpkg-query -s gz-garden >/dev/null 2>&1; then
    echo "[install] Installing from official Gazebo repository..."
    sudo wget -qO /usr/share/keyrings/gazebo-archive-keyring.gpg https://packages.osrfoundation.org/gazebo.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    sudo apt-get update
    sudo apt-get install -y gz-garden
fi

# Verify Gazebo installation
if command -v gz >/dev/null 2>&1; then
    echo "[install] Gazebo commands available"
else
    echo "[install] Warning: Gazebo package installed but commands not found in PATH"
    echo "[install] You may need to source the environment or restart your terminal"
fi

# Install ArduPilot build prerequisites
echo "[install] Installing ArduPilot build prerequisites..."
sudo apt-get install -y \
    git python3-distutils python3-empy python3-future python3-lxml \
    python3-numpy python3-opencv python3-pip \
    libxml2-dev libxslt1-dev ccache genromfs \
    libtool libffi-dev gawk \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libfltk1.3-dev \
    screen xterm \
    pkg-config \
    rapidjson-dev \
    python3-catkin-pkg \
    cmake

# Install MAVROS2 for ROS2 Humble
echo "[install] Installing MAVROS2 for ROS2 Humble..."
sudo apt-get install -y \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-mavros-msgs

# Install MAVROS GeographicLib datasets
echo "[install] Installing GeographicLib datasets for MAVROS..."
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# Install Python packages
echo "[install] Installing Python packages..."
pip3 install --user --upgrade pip
pip3 install --user mavproxy

# Set up environment variables in local env file
echo "[install] Creating local environment setup..."
cat > "$ROOT/local_env.sh" << 'EOF'
#!/bin/bash
# Local environment setup for ROSSwarm

# ROS 2 Humble setup
source /opt/ros/humble/setup.bash

# Gazebo setup
export GZ_VERSION=garden

# Add local Python packages to path
export PATH="$HOME/.local/bin:$PATH"

# Convenience aliases
alias gs='gz sim'
alias srcros='source /opt/ros/humble/setup.bash'

echo "ROSSwarm local environment loaded"
EOF

chmod +x "$ROOT/local_env.sh"

# Run the existing bootstrap script to set up ArduPilot
echo "[install] Running ArduPilot bootstrap..."
"$ROOT/scripts/bootstrap.sh"

# Install ArduPilot Gazebo Plugin
echo "[install] Installing ArduPilot Gazebo Plugin..."
ARDUPILOT_GAZEBO="$DEPS/ardupilot_gazebo"
if [ ! -d "$ARDUPILOT_GAZEBO/.git" ]; then
    echo "[install] Cloning ArduPilot Gazebo Plugin..."
    git clone https://github.com/ArduPilot/ardupilot_gazebo.git "$ARDUPILOT_GAZEBO"
fi

# Build the plugin
echo "[install] Building ArduPilot Gazebo Plugin..."
pushd "$ARDUPILOT_GAZEBO" >/dev/null
mkdir -p build
cd build
# Ensure we use the correct Gazebo version and system Python for the build
export GZ_VERSION=garden
# Temporarily deactivate virtual environment for ROS build
if [ -n "${VIRTUAL_ENV:-}" ]; then
    SAVED_VIRTUAL_ENV="$VIRTUAL_ENV"
    unset VIRTUAL_ENV
    export PATH=$(echo $PATH | tr ':' '\n' | grep -v "$SAVED_VIRTUAL_ENV" | tr '\n' ':' | sed 's/:$//')
fi
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)
popd >/dev/null

echo ""
echo "========================================="
echo "Local dependencies installed successfully!"
echo "========================================="
echo ""
echo "To use the environment, run:"
echo "  source ./local_env.sh"
echo ""
echo "Or add this to your ~/.bashrc:"
echo "  source $ROOT/local_env.sh"
echo ""