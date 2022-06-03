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
#include <depthai/pipeline/datatype/ADatatype.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_ros_driver/base_camera.hpp"
#include "depthai_ros_driver/visibility.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace depthai_ros_driver {
class SegmentationCamera : public BaseCamera {
public:
  DEPTHAI_ROS_DRIVER_PUBLIC
  SegmentationCamera(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~SegmentationCamera() {}
  void on_configure() override;

private:
  image_transport::CameraPublisher depth_pub_;
  image_transport::CameraPublisher cropped_depth_pub_;
  image_transport::CameraPublisher masked_preview_pub_;
  image_transport::CameraPublisher preview_pub_;
  image_transport::CameraPublisher mask_pub_;
  sensor_msgs::msg::CameraInfo cropped_info_;
  void timer_cb();
  void setup_publishers() override;
  void setup_pipeline() override;
  void seg_cb(const std::string &name,
              const std::shared_ptr<dai::ADatatype> &data);
  void filter_out_detections(std::vector<int> &det);
  void square_crop(cv::Mat &frame);
  void resize_and_get_mask(cv::Mat &seg_colored_src, cv::Mat &depth_frame_src,
                           cv::Mat &mask);
  void colorize_and_mask_depthamap(cv::Mat &depth_src, cv::Mat &depth_colored,
                                   cv::Mat &mask, cv::Mat &depth_frame_masked);
  cv::Mat decode_deeplab(cv::Mat mat);

  std::shared_ptr<dai::node::NeuralNetwork> nn_;
  std::shared_ptr<dai::node::XLinkOut> xout_rgb_, xout_nn_, xout_depth_,
      xout_video_;

  std::shared_ptr<dai::DataOutputQueue> preview_q_, segmentation_nn_q_,
      depth_q_, video_q_;

  const int classes_num_ = 21;
  std::vector<int> label_map_indexes_;
  std::atomic<bool> sync_nn{true};
};
} // namespace depthai_ros_driver

#endif //  DEPTHAI_ROS_DRIVER__SEGMENTATION_CAMERA_OBJ_HPP_
