cmake_minimum_required(VERSION 3.8)
project(depthai_ros_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(depthai REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

set(DEPENDENCIES
rclcpp
sensor_msgs
std_msgs
vision_msgs
depthai
cv_bridge
image_transport
OpenCV)
include_directories(include)

add_executable(mobilenet_camera src/mobilenet_camera.cpp src/mobilenet_camera_obj.cpp)
ament_target_dependencies(mobilenet_camera
                          ${DEPENDENCIES})
target_link_libraries(mobilenet_camera
                      depthai::opencv
                      depthai::core
                      ${OpenCV_LIBRARIES})
add_executable(rgb_camera src/rgb_camera.cpp src/rgb_camera_obj.cpp)
ament_target_dependencies(rgb_camera
                          ${DEPENDENCIES})
target_link_libraries(rgb_camera
                      depthai::opencv
                      depthai::core
                      ${OpenCV_LIBRARIES})
add_executable(seg_camera src/segmentation_camera.cpp src/segmentation_camera_obj.cpp)
ament_target_dependencies(seg_camera
                          ${DEPENDENCIES})
target_link_libraries(seg_camera
                      depthai::opencv
                      depthai::core
                      ${OpenCV_LIBRARIES})
add_executable(rgbd_camera src/rgbd_camera.cpp src/rgbd_camera_obj.cpp)
ament_target_dependencies(rgbd_camera
                          ${DEPENDENCIES})
target_link_libraries(rgbd_camera
                      depthai::opencv
                      depthai::core
                      ${OpenCV_LIBRARIES})
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch models config DESTINATION share/${PROJECT_NAME})

install(TARGETS
        mobilenet_camera
        rgb_camera
        seg_camera
        rgbd_camera
        DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/obj_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
