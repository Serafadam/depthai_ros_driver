#!/bin/bash
colcon build --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' '-DCMAKE_EXPORT_COMPILE_COMMANDS=On' '-DCMAKE_POSITION_INDEPENDENT_CODE=On' '-DBUILD_SHARED_LIBS=On' -Wall -Wextra -Wpedantic
