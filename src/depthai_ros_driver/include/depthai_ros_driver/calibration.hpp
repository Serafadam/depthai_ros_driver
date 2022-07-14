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
#ifndef DEPTHAI_ROS_DRIVER__CALIBRATION_HPP_
#define DEPTHAI_ROS_DRIVER__CALIBRATION_HPP_
#include <memory>
#include <string>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/Point2f.hpp"
#include "depthai/depthai.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
namespace depthai_ros_driver
{
namespace calibration
{
sensor_msgs::msg::CameraInfo get_calibration(
  std::unique_ptr<dai::Device> & device, const std::string & frame_id,
  dai::CameraBoardSocket socket, int width = 0, int height = 0,
  dai::Point2f top_left_pixel_id = {(0.0), (0.0)},
  dai::Point2f bottom_right_pixel_id = {(0.0), (0.0)});
}  // namespace calibration
}  // namespace depthai_ros_driver
#endif  // DEPTHAI_ROS_DRIVER__CALIBRATION_HPP_
