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
#ifndef DEPTHAI_ROS_DRIVER__BASE_CAMERA_HPP_
#define DEPTHAI_ROS_DRIVER__BASE_CAMERA_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"

namespace depthai_ros_driver
{
class BaseCamera : public rclcpp::Node
{
public:
  explicit BaseCamera(
    const std::string & name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(name, options)
  {
  }
  virtual ~BaseCamera() {}
  virtual void on_configure() {}
  std::unique_ptr<dai::Device> device_;
  std::unique_ptr<dai::Pipeline> pipeline_;

  void start_the_device()
  {
    bool cam_setup = false;
    while (!cam_setup) {
      try {
        device_ = std::make_unique<dai::Device>(*pipeline_, dai::UsbSpeed::SUPER);
        cam_setup = true;
      } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(this->get_logger(), "Camera not found! Please connect it");
      }
    }
    RCLCPP_INFO(this->get_logger(), "Camera connected!");
  }

  sensor_msgs::msg::CameraInfo get_calibration(
    dai::CameraBoardSocket socket, int width = 0, int height = 0,
    dai::Point2f top_left_pixel_id = {(0.0), (0.0)},
    dai::Point2f bottom_right_pixel_id = {(0.0), (0.0)})
  {
    dai::CalibrationHandler cal_data = device_->readCalibration();
    std::vector<std::vector<float>> intrinsics;
    sensor_msgs::msg::CameraInfo info;
    if (width == 0 || height == 0) {
      std::tie(intrinsics, width, height) = cal_data.getDefaultIntrinsics(socket);
    } else {
      intrinsics = cal_data.getCameraIntrinsics(socket, width, height);
    }
    info.height = height;
    info.width = width;
    for (size_t i = 0; i <= intrinsics.size(); ++i) {
      for (size_t j = 0; j <= intrinsics[i].size(); ++j) {
        info.k[j + i] = intrinsics[j][i];
      }
    }
    info.r[0] = info.r[4] = info.r[8] = 1;
    info.d = std::vector<double>(
      cal_data.getDistortionCoefficients(
        socket).begin(), cal_data.getDistortionCoefficients(socket).begin());

    std::vector<std::vector<float>> extrinsics = cal_data.getCameraExtrinsics(socket);
    info.distortion_model = "rational_polynomial";
    return info;
  }

private:
  virtual void timer_cb() {}
  virtual void declare_parameters() {}
  virtual void setup_publishers() {}
  virtual void setup_pipeline() {}
};
}  // namespace depthai_ros_driver

#endif  // DEPTHAI_ROS_DRIVER__BASE_CAMERA_HPP_
