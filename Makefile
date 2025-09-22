SHELL := /bin/bash

.PHONY: help install build sitl mavproxy gz stop clean 
help:
	@echo "Targets:"
	@echo "  install   - install local dependencies (ROS 2, Gazebo, ArduPilot)"
	@echo "  build     - build ArduPilot SITL (ArduCopter)"
	@echo "  iris-sim  - run integrated ArduCopter + Gazebo simulation"
	@echo "  stop      - stop screen/tmux sessions"
	@echo "  clean     - remove local _deps artifacts"

install:
	@bash scripts/install_local_deps.sh

build:
	@bash scripts/bootstrap.sh --build-only

sitl:
	@scripts/sitl.sh

mavproxy:
	@scripts/mavproxy.sh

gz:
	@scripts/gz.sh

stop:
	@bash scripts/stop_all.sh

clean:
	rm -rf _deps/build-ardupilot _deps/ardupilot _deps/ardupilot_logs _deps/ardupilot_gazebo
	