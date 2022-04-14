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

#ifndef DEPTHAI_ROS_DRIVER__RGB_CAMERA_OBJ_HPP_
#define DEPTHAI_ROS_DRIVER__RGB_CAMERA_OBJ_HPP_

#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_ros_driver/base_camera.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
namespace depthai_ros_driver
{
class RGBCamera : public BaseCamera
{
public:
  explicit RGBCamera(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RGBCamera() {}
  void on_configure() override;

private:
  image_transport::Publisher image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;
  sensor_msgs::msg::CameraInfo rgb_info_;

  void timer_cb() override;
  void declare_parameters() override;
  void setup_pipeline() override;
  void setup_publishers() override;

  std::shared_ptr<dai::node::ColorCamera> video_;

  std::shared_ptr<dai::node::XLinkOut> xout_video_;
  std::shared_ptr<dai::DataOutputQueue> video_q_;

  rclcpp::TimerBase::SharedPtr image_timer_;
  std::string resolution_;
  int width_, height_;
  double fps_;
  std::string camera_frame_;
};

}  // namespace depthai_ros_driver

#endif  // DEPTHAI_ROS_DRIVER__RGB_CAMERA_OBJ_HPP_
