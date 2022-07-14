#!/bin/bash
colcon build --symlink-install  --cmake-args -DCMAKE_BUILD_TYPE=Release --cmake-args -DBUILD_TESTING=OFF --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --cmake-args -DCMAKE_POSITION_INDEPENDENT_CODE=ON --cmake-args -DBUILD_SHARED_LIBS=ON
