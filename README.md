# ROSSwarm

A drone simulation environment combining **ROS 2 Humble**, **Gazebo Fortress**, and **ArduPilot SITL** for developing and testing autonomous drone swarms.

## What This Project Does

This project provides a complete, reproducible development environment for drone simulation that includes:

- **ArduPilot SITL (Software-in-the-Loop)**: Simulates ArduCopter flight controller firmware
- **Gazebo Fortress**: 3D physics simulation for realistic drone behavior
- **ROS 2 Humble**: Robotics middleware for communication and control
- **Local Installation**: Direct installation of dependencies on Ubuntu 22.04

The simulation allows you to test drone flight algorithms, swarm behaviors, and autonomous navigation without physical hardware.

## Prerequisites

- **Ubuntu 22.04** (recommended for best compatibility)
- **Git**
- **sudo access** (for installing dependencies)

## Getting Started

1. **Clone the repository:**
   ```bash
   git clone <your-repo-url>
   cd ROSSwarm
   ```

2. **Install dependencies:**
   ```bash
   make install
   ```

   This will install:
   - ROS 2 Humble
   - Gazebo Fortress
   - ArduPilot build prerequisites
   - Python dependencies

3. **Set up the environment:**
   ```bash
   source ./local_env.sh
   ```

   Or add to your `~/.bashrc` for automatic loading:
   ```bash
   echo "source $(pwd)/local_env.sh" >> ~/.bashrc
   ```

4. **Build ArduPilot SITL:**
   ```bash
   make build
   ```

## Running the Simulation

After installation and setup, you'll need two terminals:

### Terminal 1: Start ArduPilot SITL

```bash
make sitl
```

This command:
- Starts the ArduCopter SITL simulation
- Opens MAVProxy console for drone control
- Listens on UDP port 14550 for connections

### Terminal 2: Start Gazebo Simulation

```bash
make gazebo
```

This command:
- Launches Gazebo Fortress with an empty world
- Provides the 3D visualization environment
- Ready to load drone models and worlds

## Available Commands

Use `make help` to see all available commands:

- `make install` - Install local dependencies (ROS 2, Gazebo, ArduPilot)
- `make build` - Build ArduPilot SITL (ArduCopter)
- `make sitl` - Run ArduCopter SITL
- `make gazebo` - Run Gazebo world simulation
- `make stop` - Stop all running simulations
- `make clean` - Remove build artifacts and dependencies

## Project Structure

```
ROSSwarm/
├── scripts/                # Setup and run scripts
│   ├── install_local_deps.sh  # Install all dependencies locally
│   ├── bootstrap.sh        # Initial setup and ArduPilot installation
│   ├── run_sitl.sh        # ArduPilot SITL launcher
│   ├── run_gazebo.sh      # Gazebo launcher
│   └── stop_all.sh        # Stop all simulations
├── worlds/                 # Gazebo world files
├── _deps/                  # Dependencies and build artifacts (gitignored)
├── local_env.sh           # Local environment setup (created by install)
└── Makefile               # Build and run commands
```

## Next Steps

After getting the basic simulation running:

1. **Connect a Ground Control Station** (like QGroundControl) to `localhost:14550`
2. **Load custom worlds** in the `worlds/` directory
3. **Develop ROS 2 nodes** for autonomous flight control
4. **Add multiple drones** for swarm simulation
5. **Integrate sensors and cameras** for computer vision tasks