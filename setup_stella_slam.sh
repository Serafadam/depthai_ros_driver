#!/bin/bash
mkdir -p stella_ws/src \
    && cd stella_ws/src \
    && git clone --recursive https://github.com/stella-cv/stella_vslam.git \
    && git clone https://github.com/stella-cv/stella_vslam_ros.git \
    && git clone --recursive https://github.com/stevenlovegrove/Pangolin.git \
    && git clone https://github.com/stella-cv/FBoW \
    && cd Pangolin && git checkout ad8b5f83 \
    && cd ../.. \
    && sudo apt update && rosdep update && rosdep install -y -r -q --from-paths src --ignore-src \
    && colcon build --symlink-install --packages-select pangolin --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_PANGOLIN_VIEWER=ON -DUSE_SOCKET_PUBLISHER=OFF -DINSTALL_PANGOLIN_VIEWER=ON \
    && . install/setup.sh && colcon build --symlink-install  --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_PANGOLIN_VIEWER=ON -DUSE_SOCKET_PUBLISHER=OFF -DINSTALL_PANGOLIN_VIEWER=ON -DCMAKE_CXX_FLAGS="-std=c++14" \
    && . install/setup.sh \
    && cd .. && curl -sL "https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow" -o src/depthai_ros_driver/models/orb_vocab.fbow \
    && echo "Finished"
