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

#include "depthai_ros_driver/segmentation_camera_obj.hpp"

#include <memory>
#include <string>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace depthai_ros_driver
{
SegmentationCamera::SegmentationCamera(const rclcpp::NodeOptions & options)
: BaseCamera("camera", options)
{
}

void SegmentationCamera::on_configure()
{
  declare_parameters();
  setup_pipeline();
  setup_publishers();

  start_time_ = this->get_clock()->now();
  counter_ = 0;
  image_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / fps_)),
    std::bind(&SegmentationCamera::timer_cb, this));
  RCLCPP_INFO(this->get_logger(), "SegmentationCamera ready!");
}

void SegmentationCamera::declare_parameters()
{
  std::string default_nn_path = ament_index_cpp::get_package_share_directory("depthai_ros_driver") +
    "/models/deeplab_v3_plus_mnv2_decoder_256_openvino_2021.4.blob";
  fps_ = this->declare_parameter<double>("fps", 15.0);
  camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera_link");
  width_ = this->declare_parameter<int>("width", 1920);
  height_ = this->declare_parameter<int>("height", 1080);
  // this->declare_parameter<std::vector<int>>("chosen_classes", std::vector<int>{1});
  depth_filter_size_ = this->declare_parameter<int>("depth_filter_size", 7);
  nn_path_ = this->declare_parameter<std::string>("nn_path", default_nn_path);
  resolution_ = this->declare_parameter<std::string>("resolution", "1080");
}
void SegmentationCamera::setup_pipeline()
{
  pipeline_ = std::make_unique<dai::Pipeline>();
  camrgb_ = pipeline_->create<dai::node::ColorCamera>();
  nn_ = pipeline_->create<dai::node::NeuralNetwork>();
  monoleft_ = pipeline_->create<dai::node::MonoCamera>();
  monoright_ = pipeline_->create<dai::node::MonoCamera>();
  stereo_ = pipeline_->create<dai::node::StereoDepth>();

  xout_rgb_ = pipeline_->create<dai::node::XLinkOut>();
  xout_nn_ = pipeline_->create<dai::node::XLinkOut>();
  xout_depth_ = pipeline_->create<dai::node::XLinkOut>();
  xout_video_ = pipeline_->create<dai::node::XLinkOut>();
  camrgb_->setVideoSize(width_, height_);

  xout_rgb_->setStreamName("rgb");
  xout_nn_->setStreamName("nn");
  xout_depth_->setStreamName("depth");
  xout_video_->setStreamName("video");


  camrgb_->setPreviewSize(256, 256);
  camrgb_->setResolution(utils::resolution_map.at(resolution_));
  camrgb_->setInterleaved(false);
  camrgb_->setFps(fps_);
  camrgb_->setPreviewKeepAspectRatio(false);

  nn_->setNumPoolFrames(4);
  nn_->setBlobPath(nn_path_);
  nn_->setNumInferenceThreads(2);
  nn_->input.setBlocking(false);
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

  camrgb_->preview.link(nn_->input);
  if (sync_nn) {
    nn_->passthrough.link(xout_rgb_->input);
  } else {
    camrgb_->preview.link(xout_rgb_->input);
  }

  nn_->out.link(xout_nn_->input);
  stereo_->disparity.link(xout_depth_->input);
  start_the_device();
  int max_q_size = 4;
  video_q_ = device_->getOutputQueue("video", max_q_size, false);
  preview_q_ = device_->getOutputQueue("rgb", max_q_size, false);
  segmentation_nn_q_ = device_->getOutputQueue("nn", max_q_size, false);
  depth_q_ = device_->getOutputQueue("depth", max_q_size, false);
}
void SegmentationCamera::setup_publishers()
{
  depth_pub_ = image_transport::create_publisher(this, "~/depth");
  image_pub_ = image_transport::create_publisher(this, "~/image_raw");
}

void SegmentationCamera::timer_cb()
{
  auto in_det = segmentation_nn_q_->get<dai::NNData>();
  auto depth = depth_q_->get<dai::ImgFrame>();
  auto video_in = video_q_->get<dai::ImgFrame>();
  auto preview = preview_q_->get<dai::ImgFrame>();

  counter_++;
  auto currentTime = this->get_clock()->now();

  cv::Mat depthFrame = depth->getFrame();
  cv::Mat video_frame = video_in->getCvFrame();
  cv::Mat preview_frame = preview->getCvFrame();
  std::vector<int> nn_frame = in_det->getFirstLayerInt32();
  cv::Mat nn_mat = cv::Mat(nn_frame);
  nn_mat = nn_mat.reshape(0, 256);
  auto colors = decode_deeplab(nn_mat);
  cv::Mat fin_img;
  cv::addWeighted(preview_frame, 1.0, colors, 0.4, 0.0, fin_img);
  cv::Mat depthFrameColor;
  cv::normalize(depthFrame, depthFrameColor, 255, 0, cv::NORM_INF, CV_8UC1);
  cv::equalizeHist(depthFrameColor, depthFrameColor);
  cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_JET);

  auto depth_img = utils::convert_img_to_ros(
    depthFrameColor, sensor_msgs::image_encodings::BGR8, this->get_clock()->now());
  depth_pub_.publish(depth_img);
  auto video_img = utils::convert_img_to_ros(
    fin_img, sensor_msgs::image_encodings::BGR8, this->get_clock()->now());
  image_pub_.publish(video_img);
}

cv::Mat SegmentationCamera::decode_deeplab(const cv::Mat & mat)
{
  cv::Mat out = mat.mul(255 / classes_num_);
  out.convertTo(out, CV_8UC1);
  cv::Mat colors = cv::Mat(256, 1, CV_8UC3);
  cv::applyColorMap(out, colors, cv::COLORMAP_JET);
  for (int row = 0; row < out.rows; ++row) {
    uchar * p = out.ptr(row);
    for (int col = 0; col < out.cols; ++col) {
      if (*p++ == 0) {
        colors.at<cv::Vec3b>(row, col)[0] = 0;
        colors.at<cv::Vec3b>(row, col)[1] = 0;
        colors.at<cv::Vec3b>(row, col)[2] = 0;
      }
    }
  }
  return colors;
}


}  // namespace depthai_ros_driver
