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
#ifndef DEPTHAI_ROS_DRIVER__SEGMENTATION_CAMERA_OBJ_HPP_
#define DEPTHAI_ROS_DRIVER__SEGMENTATION_CAMERA_OBJ_HPP_

#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_ros_driver/base_camera.hpp"
#include "depthai_ros_driver/visibility.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace depthai_ros_driver
{
class SegmentationCamera : public BaseCamera
{
public:
  DEPTHAI_ROS_DRIVER_PUBLIC
  explicit SegmentationCamera(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SegmentationCamera() {}
  void on_configure() override;

private:
  image_transport::CameraPublisher mask_pub_;
  sensor_msgs::msg::CameraInfo cropped_info_;
  void setup_publishers();
  void setup_pipeline();
  void seg_cb(const std::string & name, const std::shared_ptr<dai::ADatatype> & data);
  void filter_out_detections(std::vector<int> & det);
  cv::Mat decode_deeplab(cv::Mat mat);

  std::shared_ptr<dai::node::NeuralNetwork> nn_;
  std::shared_ptr<dai::node::XLinkOut> xout_nn_;

  std::shared_ptr<dai::DataOutputQueue> segmentation_nn_q_;

  const int classes_num_ = 21;
  std::string nn_path_;
  std::vector<std::string> label_map_;
  std::vector<std::string> default_label_map_;
  std::vector<int> label_map_indexes_;
};
}  // namespace depthai_ros_driver

#endif  //  DEPTHAI_ROS_DRIVER__SEGMENTATION_CAMERA_OBJ_HPP_
