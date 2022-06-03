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

#include "depthai_ros_driver/mobilenet_camera_obj.hpp"

#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace depthai_ros_driver {
MobilenetCamera::MobilenetCamera(const rclcpp::NodeOptions &options)
    : BaseCamera("camera", options) {}

void MobilenetCamera::on_configure() {
  declare_basic_params();
  setup_pipeline();
  setup_publishers();

  RCLCPP_INFO(this->get_logger(), "MobilenetCamera ready!");
}

void MobilenetCamera::setup_pipeline() {
  pipeline_ = std::make_unique<dai::Pipeline>();
  setup_rgb();
  setup_stereo();
  nn_ = pipeline_->create<dai::node::MobileNetSpatialDetectionNetwork>();

  nn_->setConfidenceThreshold(0.5);
  nn_->setBlobPath(nn_path_);
  nn_->setNumInferenceThreads(2);
  nn_->input.setBlocking(false);
  nn_->setDepthLowerThreshold(100);
  nn_->setDepthUpperThreshold(5000);
  nn_->setBoundingBoxScaleFactor(0.3);

  xout_video_ = pipeline_->create<dai::node::XLinkOut>();
  xout_mono_left_ = pipeline_->create<dai::node::XLinkOut>();
  xout_mono_right_ = pipeline_->create<dai::node::XLinkOut>();
  xout_rgb_ = pipeline_->create<dai::node::XLinkOut>();
  xout_nn_ = pipeline_->create<dai::node::XLinkOut>();
  xout_bbdm_ = pipeline_->create<dai::node::XLinkOut>();
  xout_depth_ = pipeline_->create<dai::node::XLinkOut>();

  xout_rgb_->setStreamName("rgb");
  xout_nn_->setStreamName("nn");
  xout_bbdm_->setStreamName("bbdm");
  xout_depth_->setStreamName("depth");
  xout_video_->setStreamName("video");
  xout_mono_left_->setStreamName("mono_left");
  xout_mono_right_->setStreamName("mono_right");

  camrgb_->preview.link(nn_->input);
  if (sync_nn_) {
    nn_->passthrough.link(xout_rgb_->input);
  } else {
    camrgb_->preview.link(xout_rgb_->input);
  }

  nn_->out.link(xout_nn_->input);
  nn_->boundingBoxMapping.link(xout_bbdm_->input);

  stereo_->depth.link(nn_->inputDepth);
  nn_->passthroughDepth.link(xout_depth_->input);
  start_the_device();
  video_q_ = device_->getOutputQueue("video", max_q_size_, false);
  preview_q_ = device_->getOutputQueue("rgb", max_q_size_, false);
  detection_nn_q_ = device_->getOutputQueue("nn", max_q_size_, false);
  bbdm_q_ = device_->getOutputQueue("bbdm", max_q_size_, false);
  depth_q_ = device_->getOutputQueue("depth", max_q_size_, false);
  mono_left_q_ = device_->getOutputQueue("mono_left", max_q_size_, false);
  mono_right_q_ = device_->getOutputQueue("mono_right", max_q_size_, false);
}
void MobilenetCamera::setup_publishers() {
  preview_pub_ = image_transport::create_publisher(this, "~/preview");
  depth_pub_ = image_transport::create_publisher(this, "~/depth");
  mono_left_pub_ = image_transport::create_publisher(this, "~/mono_left");
  mono_right_pub_ = image_transport::create_publisher(this, "~/mono_right");
  image_pub_ = image_transport::create_publisher(this, "~/image_raw");
  det_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(
      "~/detections", 10);
}

void MobilenetCamera::timer_cb() {
  auto in_preview = preview_q_->get<dai::ImgFrame>();
  auto in_det = detection_nn_q_->get<dai::SpatialImgDetections>();
  auto depth = depth_q_->get<dai::ImgFrame>();
  auto video_in = video_q_->get<dai::ImgFrame>();
  auto mono_left = mono_left_q_->get<dai::ImgFrame>();
  auto mono_right = mono_right_q_->get<dai::ImgFrame>();

  cv::Mat preview_frame = in_preview->getCvFrame();
  cv::Mat depth_frame = depth->getFrame();
  cv::Mat video_frame = video_in->getCvFrame();
  cv::Mat mono_left_frame = mono_left->getCvFrame();
  cv::Mat mono_right_frame = mono_right->getCvFrame();

  cv::Mat depth_frame_color;
  cv::normalize(depth_frame, depth_frame_color, 255, 0, cv::NORM_INF, CV_8UC1);
  cv::equalizeHist(depth_frame_color, depth_frame_color);
  cv::applyColorMap(depth_frame_color, depth_frame_color, cv::COLORMAP_JET);

  auto detections = in_det->detections;
  vision_msgs::msg::Detection3DArray ros_det;
  ros_det.header.frame_id = rgb_frame_;
  ros_det.header.stamp = this->get_clock()->now();

  ros_det.detections.resize(detections.size());
  for (size_t i = 0; i < detections.size(); i++) {
    uint16_t label = detections[i].label;
    std::string label_name = std::to_string(label);
    if (label < label_map_.size()) {
      label_name = label_map_[label];
    }
    ros_det.detections[i].results.resize(1);
    ros_det.detections[i].results[0].hypothesis.class_id = label_name;
    ros_det.detections[i].results[0].hypothesis.score =
        detections[i].confidence;
    ros_det.detections[i].results[0].hypothesis.score =
        detections[i].confidence;
    ros_det.detections[i].results[0].pose.pose.position.x =
        detections[i].spatialCoordinates.z / 1000.0;
    ros_det.detections[i].results[0].pose.pose.position.y =
        -detections[i].spatialCoordinates.x / 1000.0;
    ros_det.detections[i].results[0].pose.pose.position.z =
        -detections[i].spatialCoordinates.y / 1000.0;
  }

  auto preview_img =
      convert_img_to_ros(preview_frame, sensor_msgs::image_encodings::BGR8,
                         rgb_frame_, this->get_clock()->now());
  preview_pub_.publish(preview_img);
  auto depth_img =
      convert_img_to_ros(depth_frame_color, sensor_msgs::image_encodings::BGR8,
                         rgb_frame_, this->get_clock()->now());
  depth_pub_.publish(depth_img);
  auto video_img =
      convert_img_to_ros(video_frame, sensor_msgs::image_encodings::BGR8,
                         rgb_frame_, this->get_clock()->now());
  image_pub_.publish(video_img);
  auto mono_left_img =
      convert_img_to_ros(mono_left_frame, sensor_msgs::image_encodings::MONO8,
                         rgb_frame_, this->get_clock()->now());
  mono_left_pub_.publish(mono_left_img);
  auto mono_right_img =
      convert_img_to_ros(mono_right_frame, sensor_msgs::image_encodings::MONO8,
                         rgb_frame_, this->get_clock()->now());
  mono_right_pub_.publish(mono_right_img);
  det_pub_->publish(ros_det);
}

} // namespace depthai_ros_driver
