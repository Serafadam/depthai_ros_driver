#!/bin/bash
file_path=$(realpath $0)
docker run -it --volume=/$(dirname $file_path):/workspace depthai_ros_driver zsh && cd /workspace
