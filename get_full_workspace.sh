#!/bin/bash
git clone --branch $1 --recurse-submodules https://github.com/Serafadam/depthai_ros_driver.git && cd depthai_ros_driver && source /opt/ros/$1/setup.sh && ./install_dependencies.sh && ./build_workspace.sh