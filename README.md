# An example-based DepthAI ROS2 driver

Work presented here has been developed by Adam Serafin while at Inmotion Labs

Hi! This is a simple (for now) project that enables Luxonis' DepthAI camera to work with ROS2 based systems.
For now, it consists of a base mobilenet_camera example, which outputs detections based on a pretrained SSD model.
The code is loosely based on depthai examples, but it follows more object-oriented pattern.
Code was being tested on:
 - OAK-D lite camera on a x86 PC - ROS2 Galactic
 - OAK-D camera on a x86 PC - ROS2 Galactic
 - OAK-D camera on an ARM based system - Nvidia Jetson, using ROS2 Galactic built from source

![](docs/example.gif)

## Working with VSCode and docker.
When opening this repository directly in VSCode, you will have the option to develop inside a container. 
After reopening, execute Code task `setup all` which will download all the repositories needed by this package.
Then a build task will build `depthai_ros_driver` as well as `depthai-core` repository. For more on tasks, see `.vscode/tasks.json` file.

## Installation

When using the Remote Containers plugin, after the container builds, run `setup all` task from VSCode. This automatically pulls the `depthai-core` git submodule and uses rosdep to install all ROS dependencies.
If you want to install it outside of the container, check out `.vscode/tasks.json` for the exact commands used. You can build both `depthai-core` and `depthai-ros-driver` by using colcon in the base directory.
## Running
To run only the mobilenet_camera, run `ros2 run depthai_ros_driver mobilenet_camera`
To launch it together with rviz and a node that outputs the detections as TF frames and markers, `ros2 launch depthai_ros_driver mobilenet_camera.launch.py use_rviz:=True`
## Known issues
- mobilenet camera experiences lower frame rate than what is set.

## Roadmap
 - Update topics - [ ]
 - Make the Node Components - [ ]
 - Add more example launch files to choose from, such as rgb-only or stereo only cameras - [ ]
 - Reorganize the code - [ ]
 - Add example how to use Camera Calibration - [ ]
 - Visual Slam example - [ ]
 - Add docker images - [ ]
