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

#include "depthai_ros_driver/rgb_camera_obj.hpp"

#include <memory>
#include <string>

namespace depthai_ros_driver
{
RGBCamera::RGBCamera(const rclcpp::NodeOptions & options)
: BaseCamera("camera", options)
{}

void RGBCamera::on_configure()
{
  declare_parameters();
  setup_pipeline();
  setup_publishers();
  image_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / fps_)),
    std::bind(&RGBCamera::timer_cb, this));
}

void RGBCamera::declare_parameters()
{
  fps_ = this->declare_parameter<double>("fps", 60.0);
  camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera_link");
  width_ = this->declare_parameter<int>("width", 1280);
  height_ = this->declare_parameter<int>("height", 720);
  resolution_ = this->declare_parameter<std::string>("resolution", "1080");
}
void RGBCamera::setup_pipeline()
{
  pipeline_ = std::make_unique<dai::Pipeline>();
  video_ = pipeline_->create<dai::node::ColorCamera>();
  video_->setBoardSocket(dai::CameraBoardSocket::RGB);
  xout_video_ = pipeline_->create<dai::node::XLinkOut>();
  video_->setVideoSize(width_, height_);

  xout_video_->setStreamName("video");
  video_->setPreviewSize(300, 300);
  video_->setResolution(utils::resolution_map.at(resolution_));
  video_->setInterleaved(false);
  video_->setFps(fps_);
  video_->setPreviewKeepAspectRatio(false);
  xout_video_->input.setBlocking(false);
  xout_video_->input.setQueueSize(1);
  video_->video.link(xout_video_->input);
  device_ = std::make_unique<dai::Device>(*pipeline_, dai::UsbSpeed::SUPER_PLUS);
  int max_q_size = 4;
  video_q_ = device_->getOutputQueue("video", max_q_size, false);
}
void RGBCamera::setup_publishers()
{
  image_pub_ = image_transport::create_publisher(this, "~/image_rect");
  cam_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", 10);
}
void RGBCamera::timer_cb()
{
  auto video_in = video_q_->get<dai::ImgFrame>();
  cv::Mat video_frame = video_in->getCvFrame();
  auto video_img = utils::convert_img_to_ros(
    video_frame, sensor_msgs::image_encodings::BGR8, this->get_clock()->now());
  auto calib = get_calibration(dai::CameraBoardSocket::RGB);
  cam_info_pub_->publish(calib);
  image_pub_.publish(video_img);
}

}  // namespace depthai_ros_driver
