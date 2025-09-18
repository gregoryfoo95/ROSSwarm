# ROSSwarm

A drone simulation environment combining **ROS 2 Humble**, **Gazebo Garden**, and **ArduPilot SITL** for developing and testing autonomous drone swarms.

## What This Project Does

This project provides a complete, reproducible development environment for drone simulation that includes:

- **ArduPilot SITL (Software-in-the-Loop)**: Simulates ArduCopter flight controller firmware
- **Gazebo Garden**: 3D physics simulation for realistic drone behavior
- **ROS 2 Humble**: Robotics middleware for communication and control
- **Integrated Simulation**: Fully integrated ArduPilot + Gazebo simulation in Berkeley, CA
- **QGroundControl Support**: Connect to ground control station for flight planning and monitoring

The simulation allows you to test drone flight algorithms, swarm behaviors, and autonomous navigation without physical hardware.

## Prerequisites

- **Ubuntu 22.04 LTS** (required for ROS 2 Humble compatibility)

## Quick Start

1. **Clone the repository:**
   ```bash
   git clone <your-repo-url>
   cd ROSSwarm
   ```

2. **Install all dependencies:**
   ```bash
   make install
   ```
   This installs ROS 2 Humble, Gazebo Garden, ArduPilot, and all required dependencies.

3. **Build ArduPilot SITL:**
   ```bash
   make build
   ```

4. **Run the integrated simulation:**
   ```bash
   make iris-sim
   ```

That's it! You now have a fully working ArduPilot + Gazebo simulation running in Berkeley, CA.

## What Gets Installed

The `make install` command installs:

- **ROS 2 Humble** (full desktop installation)
- **Gazebo Garden** (latest 3D simulation environment)
- **ArduPilot** (latest development branch)
- **ArduPilot Gazebo Plugin** (for integration)
- **MAVProxy** (for drone communication)
- **Python dependencies** (required packages)
- **colcon** (ROS 2 build system)

## Running the Simulation

### Integrated Simulation (Recommended)

Run the complete integrated simulation with one command:

```bash
make iris-sim
```

This command:
- Starts Gazebo Garden with the Iris drone model
- Launches ArduCopter SITL with Berkeley, CA location
- Establishes communication between ArduPilot and Gazebo
- Makes the simulation available on `127.0.0.1:14550` for QGroundControl

### Connect QGroundControl

1. Download and install [QGroundControl](https://qgroundcontrol.com/)
2. Start QGroundControl
3. It should automatically connect to `localhost:14550`
4. You'll see the drone positioned in Berkeley, CA with full flight controls

### Alternative: Standalone ArduPilot (No Gazebo)

For QGroundControl testing without 3D visualization:

```bash
cd _deps/ardupilot
./Tools/autotest/sim_vehicle.py --vehicle ArduCopter --frame X -L 3DRBerkeley --out 127.0.0.1:14550
```

## Available Commands

Use `make help` to see all available commands:

- `make install` - Install local dependencies (ROS 2, Gazebo, ArduPilot)
- `make build` - Build ArduPilot SITL (ArduCopter)
- `make iris-sim` - Run integrated ArduCopter + Gazebo simulation
- `make stop` - Stop screen/tmux sessions
- `make clean` - Remove local _deps artifacts

## Project Structure

```
ROSSwarm/
├── scripts/                # Setup and run scripts
│   ├── install_local_deps.sh  # Install all dependencies locally
│   ├── bootstrap.sh        # ArduPilot setup and building
│   ├── run_iris_sim.sh     # Integrated simulation launcher
│   └── stop_all.sh         # Stop all simulations
├── worlds/                 # Gazebo world files
│   └── iris_runway.sdf     # Berkeley, CA world with Iris drone
├── _deps/                  # Dependencies and build artifacts (gitignored)
│   ├── ardupilot/          # ArduPilot source code
│   └── ardupilot_gazebo/   # ArduPilot Gazebo plugin
├── local_env.sh            # Local environment setup (auto-generated)
└── Makefile                # Build and run commands
```

## Configuration Details

### Location Settings
- **Default Location**: Berkeley, CA (37.872991°N, -122.302348°W)
- **Altitude**: 20.0m
- **Heading**: 260° (roughly west-southwest)

### Network Ports
- **MAVLink Output**: `127.0.0.1:14550` (for QGroundControl)
- **ArduPilot TCP**: `127.0.0.1:5760` (internal MAVProxy connection)
- **Gazebo Plugin**: `127.0.0.1:9002` (ArduPilot-Gazebo communication)

### Integration Fix Applied
This project includes a fix for the ArduPilot-Gazebo timing integration issue:
- **Lock-step mode disabled** in ArduPilot plugin configuration
- **Proper plugin paths** configured for Gazebo Garden
- **Berkeley coordinates** set in world file

## Troubleshooting

### Common Issues

1. **"make iris-sim" fails**: Make sure you ran `make install` and `make build` first
2. **QGroundControl won't connect**: Check that no other MAVLink applications are using port 14550
3. **Gazebo doesn't start**: Ensure you have a display (`$DISPLAY` is set) and GPU drivers installed
4. **ArduPilot compilation fails**: Make sure you have all build dependencies via `make install`

### Getting Help

- Check that your system meets the Ubuntu 22.04 LTS requirement
- Ensure all commands are run from the ROSSwarm root directory
- Try `make clean` followed by `make install` and `make build` to reset everything

## What's Working

✅ **Complete Ubuntu 22.04 LTS + ROS 2 Humble setup**
✅ **ArduPilot SITL installation and building**
✅ **Gazebo Garden installation and configuration**
✅ **Integrated ArduPilot + Gazebo simulation**
✅ **Berkeley, CA location mapping (no more Australia!)**
✅ **QGroundControl connectivity on port 14550**
✅ **ArduPilot Gazebo plugin integration (timing issues fixed)**

## Next Steps

After getting the basic simulation running:

1. **Test autonomous flight modes** in QGroundControl
2. **Develop ROS 2 nodes** for autonomous flight control
3. **Create custom worlds** in the `worlds/` directory
4. **Add multiple drones** for swarm simulation
5. **Integrate sensors and cameras** for computer vision tasks
6. **Implement mission planning** with waypoint navigation

## Development Notes

This is a **Week 0 setup verification** project that provides a solid foundation for drone simulation development. The integrated ArduPilot + Gazebo simulation represents a significant step toward **Week 1 goals** of single-vehicle end-to-end simulation.