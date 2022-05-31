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

#include "depthai_ros_driver/rgbd_camera_obj.hpp"

#include <memory>
#include <string>

#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/cvstd_wrapper.hpp"
#include "opencv2/core/hal/interface.h"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include "rclcpp/logging.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace depthai_ros_driver {
RGBDCamera::RGBDCamera(const rclcpp::NodeOptions &options)
    : BaseCamera("camera", options) {
  on_configure();
}

void RGBDCamera::on_configure() {
  declare_basic_params();
  setup_pipeline();
  setup_publishers();
  // image_timer_ = this->create_wall_timer(
  //     std::chrono::milliseconds(10), std::bind(&RGBDCamera::timer_cb, this));
}

void RGBDCamera::setup_pipeline() {
  pipeline_ = std::make_unique<dai::Pipeline>();
  setup_rgb();
  setup_stereo();
  xout_video_ = pipeline_->create<dai::node::XLinkOut>();
  xout_depth_ = pipeline_->create<dai::node::XLinkOut>();
  xout_left_ = pipeline_->create<dai::node::XLinkOut>();
  xout_right_ = pipeline_->create<dai::node::XLinkOut>();

  xout_video_->setStreamName("video");
  xout_depth_->setStreamName("depth");
  xout_left_->setStreamName("left");
  xout_right_->setStreamName("right");
  xout_video_->input.setBlocking(false);
  xout_video_->input.setQueueSize(1);
  camrgb_->video.link(xout_video_->input);
  stereo_->depth.link(xout_depth_->input);
  monoleft_->out.link(xout_left_->input);
  monoright_->out.link(xout_right_->input);
  start_the_device();
  int max_q_size = 1;
  video_q_ = device_->getOutputQueue("video", max_q_size, false);
  depth_q_ = device_->getOutputQueue("depth", max_q_size, false);
  left_q_ = device_->getOutputQueue("left", max_q_size, false);
  right_q_ = device_->getOutputQueue("right", max_q_size, false);
  video_q_->addCallback(std::bind(
      &RGBDCamera::rgb_cb, this, std::placeholders::_1, std::placeholders::_2));
  depth_q_->addCallback(std::bind(&RGBDCamera::depth_cb, this,
                                  std::placeholders::_1,
                                  std::placeholders::_2));
}

void RGBDCamera::setup_publishers() {
  image_pub_ = image_transport::create_camera_publisher(this, "~/image_rect");
  depth_pub_ =
      image_transport::create_camera_publisher(this, "~/depth/image_rect");
  rgb_info_ = get_calibration(dai::CameraBoardSocket::RGB);
  depth_info_ = get_calibration(dai::CameraBoardSocket::RGB);
}
void RGBDCamera::depth_cb(const std::string &name,
                          const std::shared_ptr<dai::ADatatype> &data) {
  auto depth_in = std::dynamic_pointer_cast<dai::ImgFrame>(data);
  cv::Mat depth_frame = depth_in->getCvFrame();
  cv::resize(depth_frame, depth_frame, cv::Size(rgb_width_, rgb_height_));

  depth_info_.header.frame_id = camera_frame_;
  auto curr_time = this->get_clock()->now();
  depth_info_.header.stamp = curr_time;

  depth_pub_.publish(
      convert_img_to_ros(depth_frame, sensor_msgs::image_encodings::TYPE_16UC1,
                         curr_time),
      depth_info_);
}
void RGBDCamera::rgb_cb(const std::string &name,
                        const std::shared_ptr<dai::ADatatype> &data) {
  auto depth_in = std::dynamic_pointer_cast<dai::ImgFrame>(data);
  cv::Mat rgb_frame = depth_in->getCvFrame();
  cv::resize(rgb_frame, rgb_frame, cv::Size(rgb_width_, rgb_height_));
  rgb_info_.header.frame_id = camera_frame_;
  auto curr_time = this->get_clock()->now();
  rgb_info_.header.stamp = curr_time;

  image_pub_.publish(convert_img_to_ros(rgb_frame,
                                        sensor_msgs::image_encodings::BGR8,
                                        curr_time),
                     depth_info_);
}
void RGBDCamera::timer_cb() {}
} // namespace depthai_ros_driver
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ros_driver::RGBDCamera);
