# ROS2 Drone Control Package

This ROS2 package provides high-level flight control capabilities for ArduPilot drones through MAVROS2 integration.

## Features

- **Drone State Monitor**: Monitors vehicle state, position, battery, GPS status
- **Flight Commander**: Provides high-level flight commands (arm, takeoff, land, goto)
- **Test Flight**: Automated test sequences for validating drone functionality
- **MAVROS2 Integration**: Seamless communication with ArduPilot through MAVLink

## Package Structure

```
drone_control_pkg/
├── src/
│   ├── drone_state_monitor.py    # Vehicle state monitoring
│   ├── flight_commander.py       # Flight command interface
│   └── test_flight.py            # Automated test flights
├── launch/
│   └── drone_control.launch.py   # Launch MAVROS + control nodes
├── config/                       # Configuration files
├── package.xml                   # Package metadata
└── CMakeLists.txt                # Build configuration
```

## Quick Start

### 1. Build the Package
```bash
make ros2-build
```

### 2. Start ArduPilot Simulation
```bash
# Terminal 1: Start integrated Gazebo + ArduPilot simulation
make iris-sim
```

### 3. Launch ROS2 Control System
```bash
# Terminal 2: Start ROS2 nodes with MAVROS
make ros2-launch
```

### 4. Run Test Flight
```bash
# Terminal 3: Execute automated test flight
make test-flight
```

## Nodes

### drone_state_monitor
Monitors and publishes drone state information.

**Subscribed Topics:**
- `/mavros/state` - Vehicle connection and mode
- `/mavros/local_position/pose` - Local position and attitude
- `/mavros/local_position/velocity_local` - Velocity information
- `/mavros/global_position/global` - GPS position
- `/mavros/battery` - Battery status
- `/mavros/extended_state` - Landing state information

**Published Topics:**
- `/drone/status_summary` - Consolidated status information

### flight_commander
Provides high-level flight command interface.

**Subscribed Topics:**
- `/drone/command` - Flight commands (String messages)
- `/mavros/state` - Vehicle state for command validation
- `/mavros/local_position/pose` - Position feedback

**Published Topics:**
- `/drone/flight_status` - Current flight state
- `/mavros/setpoint_position/local` - Position setpoints

**Services Used:**
- `/mavros/cmd/arming` - Arm/disarm vehicle
- `/mavros/cmd/takeoff` - Takeoff command
- `/mavros/cmd/land` - Landing command
- `/mavros/set_mode` - Flight mode changes

### test_flight
Automated test flight sequences.

**Available Commands:**
- `arm` - Arm the vehicle
- `disarm` - Disarm the vehicle
- `takeoff` - Takeoff to default altitude (5m)
- `land` - Land at current position
- `set_guided` - Set GUIDED mode for autonomous control
- `set_stabilize` - Set STABILIZE mode for manual control
- `goto X Y Z` - Fly to position (e.g., "goto 5 5 10")

## Usage Examples

### Manual Command Control
```bash
# Send individual commands
ros2 topic pub /drone/command std_msgs/String "data: 'arm'"
ros2 topic pub /drone/command std_msgs/String "data: 'takeoff'"
ros2 topic pub /drone/command std_msgs/String "data: 'goto 10 10 5'"
ros2 topic pub /drone/command std_msgs/String "data: 'land'"
```

### Interactive Flight Control
```bash
# Run test flight in interactive mode
ros2 run drone_control_pkg test_flight.py --interactive
```

### Monitor Drone Status
```bash
# Watch status updates
ros2 topic echo /drone/status_summary
ros2 topic echo /drone/flight_status
```

## Configuration

### MAVROS Connection
The default MAVROS configuration connects to:
- **FCU URL**: `udp://:14550@127.0.0.1:14551`
- **Target System**: 1
- **Target Component**: 1

### Flight Parameters
- **Default Takeoff Altitude**: 5.0 meters
- **Position Hold Tolerance**: 0.5 meters
- **Landing Detection Threshold**: 0.5 meters

## Troubleshooting

### Common Issues

1. **MAVROS not connecting to ArduPilot**
   - Ensure ArduPilot simulation is running (`make iris-sim`)
   - Check that port 14550 is not in use by other applications
   - Verify MAVROS installation: `ros2 pkg list | grep mavros`

2. **Commands not working**
   - Check vehicle is connected: `ros2 topic echo /mavros/state`
   - Ensure vehicle is in GUIDED mode for autonomous commands
   - Verify GPS fix for takeoff commands

3. **Package build errors**
   - Install missing dependencies: `rosdep install --from-paths ros2_ws/src --ignore-src -r -y`
   - Source ROS2 environment: `source /opt/ros/humble/setup.bash`

### Useful Debug Commands

```bash
# Check MAVROS topics
ros2 topic list | grep mavros

# Monitor MAVLink connection
ros2 topic echo /mavros/state

# Check current position
ros2 topic echo /mavros/local_position/pose

# View available services
ros2 service list | grep mavros

# Test MAVROS arming
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "value: true"
```

## Integration with ArduPilot

This package works with ArduPilot SITL and hardware autopilots. Key integration points:

- **MAVLink Protocol**: Communication via MAVROS2 bridge
- **Flight Modes**: GUIDED mode for autonomous control, STABILIZE for manual
- **Coordinate Frames**: NED (North-East-Down) for local positioning
- **Safety Features**: GPS dependency for takeoff, battery monitoring

## Next Steps

1. **Add Sensor Integration**: Camera, LIDAR, obstacle avoidance
2. **Mission Planning**: Waypoint sequences, complex flight patterns
3. **Multi-Vehicle**: Extend for drone swarm coordination
4. **Computer Vision**: Object detection and tracking capabilities
5. **Formation Flight**: Coordinated multi-drone operations