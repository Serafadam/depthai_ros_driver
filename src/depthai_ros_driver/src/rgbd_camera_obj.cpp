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

namespace depthai_ros_driver
{
RGBDCamera::RGBDCamera(const rclcpp::NodeOptions & options)
: BaseCamera("camera", options)
{}

void RGBDCamera::on_configure()
{
  declare_parameters();
  setup_pipeline();
  setup_publishers();
  image_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / fps_)),
    std::bind(&RGBDCamera::timer_cb, this));
}

void RGBDCamera::declare_parameters()
{
  fps_ = this->declare_parameter<double>("fps", 15.0);
  camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera_link");
  width_ = this->declare_parameter<int>("width", 1280);
  height_ = this->declare_parameter<int>("height", 720);
  resolution_ = this->declare_parameter<std::string>("resolution", "1080");
  depth_filter_size_ = this->declare_parameter<int>("depth_filter_size", 7);
}
void RGBDCamera::setup_pipeline()
{
  pipeline_ = std::make_unique<dai::Pipeline>();
  video_ = pipeline_->create<dai::node::ColorCamera>();
  monoleft_ = pipeline_->create<dai::node::MonoCamera>();
  monoright_ = pipeline_->create<dai::node::MonoCamera>();
  stereo_ = pipeline_->create<dai::node::StereoDepth>();

  video_->setVideoSize(width_, height_);
  xout_video_ = pipeline_->create<dai::node::XLinkOut>();
  xout_video_->setStreamName("video");
  xout_depth_ = pipeline_->create<dai::node::XLinkOut>();
  xout_depth_->setStreamName("depth");

  monoleft_->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
  monoright_->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
  monoleft_->setBoardSocket(dai::CameraBoardSocket::LEFT);
  monoright_->setBoardSocket(dai::CameraBoardSocket::RIGHT);
  auto median = static_cast<dai::MedianFilter>(depth_filter_size_);
  stereo_->setLeftRightCheck(true);
  stereo_->setDepthAlign(dai::CameraBoardSocket::RGB);
  stereo_->initialConfig.setLeftRightCheckThreshold(5);
  stereo_->initialConfig.setMedianFilter(median);
  stereo_->initialConfig.setConfidenceThreshold(210);
  stereo_->initialConfig.setSubpixel(true);
  stereo_->setExtendedDisparity(false);
  stereo_->setRectifyEdgeFillColor(-1);

  monoleft_->out.link(stereo_->left);
  monoright_->out.link(stereo_->right);


  video_->setPreviewSize(300, 300);
  video_->setResolution(utils::resolution_map.at(resolution_));
  video_->setInterleaved(false);
  video_->setFps(fps_);
  video_->setPreviewKeepAspectRatio(false);
  xout_video_->input.setBlocking(false);
  xout_video_->input.setQueueSize(1);
  video_->video.link(xout_video_->input);
  stereo_->disparity.link(xout_depth_->input);
  start_the_device();
  int max_q_size = 4;
  video_q_ = device_->getOutputQueue("video", max_q_size, false);
  depth_q_ = device_->getOutputQueue("depth", max_q_size, false);
}
void RGBDCamera::setup_publishers()
{
  image_pub_ = image_transport::create_camera_publisher(this, "~/image_rect");
  depth_pub_ = image_transport::create_camera_publisher(this, "~/depth/image_rect");
  rgb_info_ = get_calibration(dai::CameraBoardSocket::RGB);
  depth_info_ = get_calibration(dai::CameraBoardSocket::RGB);
}
void RGBDCamera::timer_cb()
{
  auto video_in = video_q_->get<dai::ImgFrame>();
  auto depth_in = depth_q_->get<dai::ImgFrame>();
  cv::Mat video_frame = video_in->getCvFrame();
  cv::Mat depth_frame = depth_in->getCvFrame();
  cv::resize(depth_frame, depth_frame, cv::Size(1280,720));
  cv::resize(video_frame, video_frame, cv::Size(1280,720));
  cv::Mat depth_norm;
  cv::normalize(depth_frame, depth_norm, 65535, 0, cv::NORM_MINMAX, CV_16UC1);
  auto curr_time = this->get_clock()->now();
  rgb_info_.header.stamp = depth_info_.header.stamp = curr_time;
  rgb_info_.header.frame_id = depth_info_.header.frame_id = "camera_link";

  auto video_img = utils::convert_img_to_ros(
    video_frame, sensor_msgs::image_encodings::BGR8, curr_time);
  auto depth_img = utils::convert_img_to_ros(
    depth_norm, sensor_msgs::image_encodings::TYPE_16UC1, curr_time);

  image_pub_.publish(video_img, rgb_info_);
  depth_pub_.publish(depth_img, depth_info_);
}

}  // namespace depthai_ros_driver
