SHELL := /bin/bash

.PHONY: help install build sitl gazebo iris-sim ros2-build ros2-launch test-flight stop clean

help:
	@echo "Targets:"
	@echo "  install      - install local dependencies (ROS 2, Gazebo, ArduPilot, MAVROS2)"
	@echo "  build        - build ArduPilot SITL (ArduCopter)"
	@echo "  iris-sim     - run integrated ArduCopter + Gazebo simulation (foreground)"
	@echo "  iris-sim-bg  - run integrated ArduCopter + Gazebo simulation (background)"
	@echo "  ros2-build   - build ROS2 drone control package"
	@echo "  ros2-launch  - launch ROS2 drone control system with MAVROS"
	@echo "  test-flight  - run automated test flight (requires simulation + ROS2)"
	@echo "  stop         - stop screen/tmux sessions"
	@echo "  clean        - remove local _deps artifacts"

install:
	@bash scripts/install_local_deps.sh

build:
	@bash scripts/bootstrap.sh --build-only

iris-sim:
	@bash scripts/run_iris_sim.sh

iris-sim-bg:
	@bash scripts/run_iris_sim.sh --background

ros2-build:
	@echo "Building ROS2 drone control package..."
	@source /opt/ros/humble/setup.bash && source ./local_env.sh && cd ros2_ws && colcon build --packages-select drone_control_pkg
	@echo "ROS2 package built successfully!"
	@echo "Run 'source ros2_ws/install/setup.bash' to use the package"

ros2-launch:
	@echo "Launching ROS2 drone control system..."
	@echo "Make sure ArduPilot simulation is running first (make iris-sim)"
	@source /opt/ros/humble/setup.bash && source ./local_env.sh && source ros2_ws/install/setup.bash && \
	ros2 launch drone_control_pkg drone_control.launch.py

test-flight:
	@echo "Running automated test flight..."
	@echo "Make sure both simulation and ROS2 system are running!"
	@source /opt/ros/humble/setup.bash && source ./local_env.sh && source ros2_ws/install/setup.bash && \
	ros2 run drone_control_pkg test_flight

stop:
	@bash scripts/stop_all.sh

clean:
	rm -rf _deps/build-ardupilot _deps/ardupilot _deps/ardupilot_logs _deps/ardupilot_gazebo ros2_ws/build ros2_ws/install ros2_ws/log
