#!/bin/bash
file_path=$(realpath $0)
docker run -it -v /dev/:/dev/ --privileged --volume=/$(dirname $file_path):/workspace --network host depthai_ros_driver zsh && cd /workspace
