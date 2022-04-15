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
  declare_basic_params();
  setup_pipeline();
  setup_publishers();
  image_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / fps_)),
    std::bind(&RGBCamera::timer_cb, this));
}

void RGBCamera::setup_pipeline()
{
  pipeline_ = std::make_unique<dai::Pipeline>();
  setup_rgb();
  xout_video_ = pipeline_->create<dai::node::XLinkOut>();

  xout_video_->setStreamName("video");
  xout_video_->input.setBlocking(false);
  xout_video_->input.setQueueSize(1);
  camrgb_->video.link(xout_video_->input);
  start_the_device();
  int max_q_size = 4;
  video_q_ = device_->getOutputQueue("video", max_q_size, false);
}
void RGBCamera::setup_publishers()
{
  image_pub_ = image_transport::create_camera_publisher(this, "~/image_rect");
  rgb_info_ = get_calibration(dai::CameraBoardSocket::RGB);
}
void RGBCamera::timer_cb()
{
  auto video_in = video_q_->get<dai::ImgFrame>();
  cv::Mat video_frame = video_in->getCvFrame();
  auto stamp = this->get_clock()->now();
  auto video_img = convert_img_to_ros(
    video_frame, sensor_msgs::image_encodings::BGR8, stamp);

  video_img.header.frame_id = rgb_info_.header.frame_id = camera_frame_;
  video_img.header.stamp = rgb_info_.header.stamp = stamp;
  image_pub_.publish(video_img, rgb_info_);
}

}  // namespace depthai_ros_driver
