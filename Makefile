SHELL := /bin/bash

.PHONY: help install build sitl gazebo stop clean

help:
	@echo "Targets:"
	@echo "  install   - install local dependencies (ROS 2, Gazebo, ArduPilot)"
	@echo "  build     - build ArduPilot SITL (ArduCopter)"
	@echo "  sitl      - run ArduCopter SITL"
	@echo "  gazebo    - run Gazebo world"
	@echo "  stop      - stop screen/tmux sessions"
	@echo "  clean     - remove local _deps artifacts"

install:
	@bash scripts/install_local_deps.sh

build:
	@bash scripts/bootstrap.sh --build-only

sitl:
	@bash scripts/run_sitl.sh

gazebo:
	@bash scripts/run_gazebo.sh

stop:
	@bash scripts/stop_all.sh

clean:
	rm -rf _deps/build-ardupilot _deps/ardupilot _deps/ardupilot_logs
