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

namespace depthai_ros_driver
{
MobilenetCamera::MobilenetCamera(const rclcpp::NodeOptions & options)
: BaseCamera("camera", options)
{
}

void MobilenetCamera::on_configure()
{
  declare_parameters();
  setup_pipeline();
  setup_publishers();

  start_time_ = this->get_clock()->now();
  counter_ = 0;
  image_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / fps_)),
    std::bind(&MobilenetCamera::timer_cb, this));
}

void MobilenetCamera::declare_parameters()
{
  std::string default_nn_path = ament_index_cpp::get_package_share_directory("depthai_ros_driver") +
    "/models/mobilenet-ssd_openvino_2021.2_6shave.blob";
  fps_ = this->declare_parameter<double>("fps", 15.0);
  camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera_link");
  width_ = this->declare_parameter<int>("width", 1280);
  height_ = this->declare_parameter<int>("height", 720);
  label_map_ = this->declare_parameter<std::vector<std::string>>("label_map", default_label_map_);
  depth_filter_size_ = this->declare_parameter<int>("depth_filter_size", 7);
  nn_path_ = this->declare_parameter<std::string>("nn_path", default_nn_path);
  resolution_ = this->declare_parameter<std::string>("resolution", "1080");
}
void MobilenetCamera::setup_pipeline()
{
  pipeline_ = std::make_unique<dai::Pipeline>();
  camrgb_ = pipeline_->create<dai::node::ColorCamera>();
  nn_ = pipeline_->create<dai::node::MobileNetSpatialDetectionNetwork>();
  monoleft_ = pipeline_->create<dai::node::MonoCamera>();
  monoright_ = pipeline_->create<dai::node::MonoCamera>();
  stereo_ = pipeline_->create<dai::node::StereoDepth>();

  xout_video_ = pipeline_->create<dai::node::XLinkOut>();
  xout_mono_left_ = pipeline_->create<dai::node::XLinkOut>();
  xout_mono_right_ = pipeline_->create<dai::node::XLinkOut>();
  xout_rgb_ = pipeline_->create<dai::node::XLinkOut>();
  xout_nn_ = pipeline_->create<dai::node::XLinkOut>();
  xout_bbdm_ = pipeline_->create<dai::node::XLinkOut>();
  xout_depth_ = pipeline_->create<dai::node::XLinkOut>();
  camrgb_->setVideoSize(width_, height_);

  xout_rgb_->setStreamName("rgb");
  xout_nn_->setStreamName("nn");
  xout_bbdm_->setStreamName("bbdm");
  xout_depth_->setStreamName("depth");
  xout_video_->setStreamName("video");
  xout_mono_left_->setStreamName("mono_left");
  xout_mono_right_->setStreamName("mono_right");

  camrgb_->setPreviewSize(300, 300);
  camrgb_->setResolution(utils::resolution_map.at(resolution_));
  camrgb_->setInterleaved(false);
  camrgb_->setFps(fps_);
  camrgb_->setPreviewKeepAspectRatio(false);

  nn_->setConfidenceThreshold(0.5);
  nn_->setBlobPath(nn_path_);
  nn_->setNumInferenceThreads(2);
  nn_->input.setBlocking(false);
  nn_->setDepthLowerThreshold(100);
  nn_->setDepthUpperThreshold(5000);
  nn_->setBoundingBoxScaleFactor(0.3);
  monoleft_->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
  monoright_->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
  monoleft_->setBoardSocket(dai::CameraBoardSocket::LEFT);
  monoright_->setBoardSocket(dai::CameraBoardSocket::RIGHT);
  auto median = static_cast<dai::MedianFilter>(depth_filter_size_);
  stereo_->setLeftRightCheck(true);
  stereo_->initialConfig.setLeftRightCheckThreshold(4);
  stereo_->initialConfig.setMedianFilter(median);
  stereo_->initialConfig.setConfidenceThreshold(245);
  stereo_->setRectifyEdgeFillColor(-1);

  monoleft_->out.link(stereo_->left);
  monoright_->out.link(stereo_->right);
  camrgb_->video.link(xout_video_->input);
  monoleft_->out.link(xout_mono_left_->input);
  monoright_->out.link(xout_mono_right_->input);

  camrgb_->preview.link(nn_->input);
  if (sync_nn) {
    nn_->passthrough.link(xout_rgb_->input);
  } else {
    camrgb_->preview.link(xout_rgb_->input);
  }

  nn_->out.link(xout_nn_->input);
  nn_->boundingBoxMapping.link(xout_bbdm_->input);

  stereo_->depth.link(nn_->inputDepth);
  nn_->passthroughDepth.link(xout_depth_->input);

  device_ = std::make_unique<dai::Device>(*pipeline_, dai::UsbSpeed::SUPER);
  int max_q_size = 4;
  video_q_ = device_->getOutputQueue("video", max_q_size, false);
  preview_q_ = device_->getOutputQueue("rgb", max_q_size, false);
  detection_nn_q_ = device_->getOutputQueue("nn", max_q_size, false);
  bbdm_q_ = device_->getOutputQueue("bbdm", max_q_size, false);
  depth_q_ = device_->getOutputQueue("depth", max_q_size, false);
  mono_left_q_ = device_->getOutputQueue("mono_left", max_q_size, false);
  mono_right_q_ = device_->getOutputQueue("mono_right", max_q_size, false);
}
void MobilenetCamera::setup_publishers()
{
  preview_pub_ = image_transport::create_publisher(this, "~/preview");
  depth_pub_ = image_transport::create_publisher(this, "~/depth");
  mono_left_pub_ = image_transport::create_publisher(this, "~/mono_left");
  mono_right_pub_ = image_transport::create_publisher(this, "~/mono_right");
  image_pub_ = image_transport::create_publisher(this, "~/image_raw");
  det_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("~/detections", 10);
}

void MobilenetCamera::timer_cb()
{
  auto in_preview = preview_q_->get<dai::ImgFrame>();
  auto in_det = detection_nn_q_->get<dai::SpatialImgDetections>();
  auto depth = depth_q_->get<dai::ImgFrame>();
  auto video_in = video_q_->get<dai::ImgFrame>();
  auto mono_left = mono_left_q_->get<dai::ImgFrame>();
  auto mono_right = mono_right_q_->get<dai::ImgFrame>();

  counter_++;
  auto currentTime = this->get_clock()->now();

  cv::Mat frame = in_preview->getCvFrame();
  cv::Mat depthFrame = depth->getFrame();
  cv::Mat video_frame = video_in->getCvFrame();
  cv::Mat mono_left_frame = mono_left->getCvFrame();
  cv::Mat mono_right_frame = mono_right->getCvFrame();

  cv::Mat depthFrameColor;
  cv::normalize(depthFrame, depthFrameColor, 255, 0, cv::NORM_INF, CV_8UC1);
  cv::equalizeHist(depthFrameColor, depthFrameColor);
  cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_JET);

  detections = in_det->detections;
  vision_msgs::msg::Detection3DArray ros_det;
  ros_det.header.frame_id = camera_frame_;
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
    ros_det.detections[i].results[0].hypothesis.score = detections[i].confidence;
    ros_det.detections[i].results[0].hypothesis.score = detections[i].confidence;
    ros_det.detections[i].results[0].pose.pose.position.x =
      detections[i].spatialCoordinates.z / 1000.0;
    ros_det.detections[i].results[0].pose.pose.position.y =
      -detections[i].spatialCoordinates.x / 1000.0;
    ros_det.detections[i].results[0].pose.pose.position.z =
      -detections[i].spatialCoordinates.y / 1000.0;
  }

  auto preview_img =
    utils::convert_img_to_ros(frame, sensor_msgs::image_encodings::BGR8, this->get_clock()->now());
  preview_pub_.publish(preview_img);
  auto depth_img = utils::convert_img_to_ros(
    depthFrameColor, sensor_msgs::image_encodings::BGR8, this->get_clock()->now());
  depth_pub_.publish(depth_img);
  auto video_img = utils::convert_img_to_ros(
    video_frame, sensor_msgs::image_encodings::BGR8, this->get_clock()->now());
  image_pub_.publish(video_img);
  auto mono_left_img = utils::convert_img_to_ros(
    mono_left_frame, sensor_msgs::image_encodings::MONO8, this->get_clock()->now());
  mono_left_pub_.publish(mono_left_img);
  auto mono_right_img = utils::convert_img_to_ros(
    mono_right_frame, sensor_msgs::image_encodings::MONO8, this->get_clock()->now());
  mono_right_pub_.publish(mono_right_img);
  det_pub_->publish(ros_det);
}

}  // namespace depthai_ros_driver
