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
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
include_directories(include)

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/base_camera.cpp
  src/calibration.cpp
  src/params_rgb.cpp
  src/params_stereo.cpp
  src/rgbd_camera_obj.cpp
  src/mobilenet_camera_obj.cpp
  src/segmentation_camera_obj.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
target_link_libraries(
  ${PROJECT_NAME}
  depthai::opencv
  depthai::core
  ${OpenCV_LIBRARIES}
)
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::RGBDCamera")
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::MobilenetCamera")
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::SegmentationCamera")
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

install(
  TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(
  PROGRAMS
  scripts/obj_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
