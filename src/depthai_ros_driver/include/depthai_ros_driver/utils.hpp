// Copyright (c) [2022] [Adam Serafin]

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef DEPTHAI_ROS_DRIVER__UTILS_HPP_
#define DEPTHAI_ROS_DRIVER__UTILS_HPP_

#include <cv_bridge/cv_bridge.h>

#include <string>
#include <unordered_map>
#include <vector>

#include "depthai/depthai.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

namespace depthai_ros_driver
{
namespace utils
{
static std::unordered_map<std::string, dai::ColorCameraProperties::SensorResolution>
resolution_map = {
  {"1080", dai::ColorCameraProperties::SensorResolution::THE_1080_P},
  {"4k", dai::ColorCameraProperties::SensorResolution::THE_4_K},
  {"12MP", dai::ColorCameraProperties::SensorResolution::THE_12_MP},
};
const std::vector<std::string> default_label_map_ = {
  "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",
  "car", "cat", "chair", "cow", "diningtable", "dog", "horse",
  "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"};
sensor_msgs::msg::Image convert_img_to_ros(
  const cv::Mat & frame, const char * encoding, rclcpp::Time stamp);
}  // namespace utils
}  // namespace depthai_ros_driver
#endif  // DEPTHAI_ROS_DRIVER__UTILS_HPP_
