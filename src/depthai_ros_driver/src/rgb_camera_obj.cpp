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

#include <depthai-shared/properties/VideoEncoderProperties.hpp>
#include <depthai/pipeline/datatype/ADatatype.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/node/XLinkOut.hpp>
#include <memory>
#include <rmw/qos_profiles.h>
#include <std_srvs/srv/detail/trigger__struct.hpp>
#include <string>

namespace depthai_ros_driver {
RGBCamera::RGBCamera(const rclcpp::NodeOptions &options)
    : BaseCamera("camera", options) {}

void RGBCamera::on_configure() {
  declare_basic_params();
  setup_pipeline();
  setup_publishers();
  record_ = false;
  trigger_recording_srv = this->create_service<std_srvs::srv::Trigger>(
      "~/trigger_recording",
      std::bind(&RGBCamera::trigger_cb, this, std::placeholders::_1,
                std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(), "RGBCam ready!");
}

void RGBCamera::setup_pipeline() {
  pipeline_ = std::make_unique<dai::Pipeline>();
  setup_rgb();
  xout_video_ = pipeline_->create<dai::node::XLinkOut>();
  xout_enc_ = pipeline_->create<dai::node::XLinkOut>();
  video_enc_ = pipeline_->create<dai::node::VideoEncoder>();
  video_enc_->setDefaultProfilePreset(
      30, dai::VideoEncoderProperties::Profile::H265_MAIN);
  xout_enc_->setStreamName("h265");
  xout_video_->setStreamName("video");
  xout_video_->input.setBlocking(false);
  xout_video_->input.setQueueSize(1);
  camrgb_->video.link(xout_video_->input);
  camrgb_->video.link(video_enc_->input);
  video_enc_->bitstream.link(xout_enc_->input);
  start_the_device();
  video_q_ = device_->getOutputQueue("video", max_q_size_, false);
  enc_q_ = device_->getOutputQueue("h265", max_q_size_, false);
  video_q_->addCallback(std::bind(
      &RGBCamera::rgb_cb, this, std::placeholders::_1, std::placeholders::_2));
  enc_q_->addCallback(std::bind(&RGBCamera::enc_cb, this, std::placeholders::_1,
                                std::placeholders::_2));
}
void RGBCamera::setup_publishers() {
  image_pub_ = image_transport::create_camera_publisher(this, "~/image_rect");
  rgb_info_ = get_calibration(dai::CameraBoardSocket::RGB);
}
void RGBCamera::rgb_cb(const std::string &name,
                       const std::shared_ptr<dai::ADatatype> &data) {
  auto rgb_in = std::dynamic_pointer_cast<dai::ImgFrame>(data);
  cv::Mat rgb_frame = rgb_in->getCvFrame();
  publish_img(rgb_frame, sensor_msgs::image_encodings::BGR8, rgb_info_,
              image_pub_);
}
void RGBCamera::enc_cb(const std::string &name,
                       const std::shared_ptr<dai::ADatatype> &data) {
  if (record_) {
    if (!started_recording_) {
      std::stringstream current_time;
      // getting current time from node
      auto now = std::chrono::system_clock::now();
      auto in_time_t = std::chrono::system_clock::to_time_t(now);
      current_time << std::put_time(std::localtime(&in_time_t),
                                    "%Y_%m_%d_%H_%M_%S");
      current_time << ".h265";
      video_file_ = std::ofstream(current_time.str(), std::ios::binary);
      started_recording_ = true;
    }
    RCLCPP_INFO(this->get_logger(), "Recording!");
    auto enc_in = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    video_file_.write((char *)(enc_in->getData().data()),
                      enc_in->getData().size());
  }
}
void RGBCamera::trigger_cb(const std_srvs::srv::Trigger::Request::SharedPtr req,
                           std_srvs::srv::Trigger::Response::SharedPtr res) {
  if (!record_) {
    RCLCPP_INFO(this->get_logger(), "Starting recording.");
  } else {
    RCLCPP_INFO(this->get_logger(), "Stopping recording.");
    started_recording_ = false;
  }
  record_ = !record_;
}
} // namespace depthai_ros_driver
