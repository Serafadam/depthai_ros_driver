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
#ifndef DEPTHAI_ROS_DRIVER__BASE_CAMERA_HPP_
#define DEPTHAI_ROS_DRIVER__BASE_CAMERA_HPP_

#include <string>
#include <memory>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"

namespace depthai_ros_driver
{
class BaseCamera : public rclcpp::Node
{
public:
  explicit BaseCamera(
    const std::string & name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(name, options)
  {
  }
  virtual ~BaseCamera() {}
  virtual void on_configure() {}

  std::unique_ptr<dai::Device> device_;
  std::unique_ptr<dai::Pipeline> pipeline_;

  void start_the_device()
  {
    bool cam_setup = false;
    while (!cam_setup) {
      try {
        device_ = std::make_unique<dai::Device>(*pipeline_, dai::UsbSpeed::SUPER_PLUS);
        cam_setup = true;
      } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(this->get_logger(), "Camera not found! Please connect it");
      }
    }
    RCLCPP_INFO(this->get_logger(), "Camera connected!");
  }

  virtual void setup_rgb(
    int preview_size = 256,
    bool set_interleaved = false,
    bool set_preview_keep_aspect_ratio = false)
  {
    camrgb_ = pipeline_->create<dai::node::ColorCamera>();
    camrgb_->setPreviewSize(preview_size, preview_size);
    camrgb_->setVideoSize(rgb_width_, rgb_height_);
    camrgb_->setResolution(rgb_resolution_map.at(rgb_resolution_));
    camrgb_->setInterleaved(set_interleaved);
    camrgb_->setFps(fps_);
    camrgb_->setPreviewKeepAspectRatio(set_preview_keep_aspect_ratio);
  }
  virtual void setup_stereo()
  {
    monoleft_ = pipeline_->create<dai::node::MonoCamera>();
    monoright_ = pipeline_->create<dai::node::MonoCamera>();
    stereo_ = pipeline_->create<dai::node::StereoDepth>();
    monoleft_->setResolution(mono_resolution_map.at(mono_resolution_));
    monoright_->setResolution(mono_resolution_map.at(mono_resolution_));
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
  }

  virtual void declare_basic_params()
  {

    fps_ = this->declare_parameter<double>("fps", 15.0);
    camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera_link");
    rgb_width_ = this->declare_parameter<int>("width", 1280);
    rgb_height_ = this->declare_parameter<int>("height", 720);
    label_map_ = this->declare_parameter<std::vector<std::string>>(
      "label_map",
      default_label_map_);
    depth_filter_size_ = this->declare_parameter<int>("depth_filter_size", 7);
    rgb_resolution_ = this->declare_parameter<std::string>("rgb_resolution", "1080");
    mono_resolution_ = this->declare_parameter<std::string>("mono_resolution", "400");
  }

  sensor_msgs::msg::Image convert_img_to_ros(
    const cv::Mat & frame, const char * encoding, rclcpp::Time stamp)
  {
    cv_bridge::CvImage img_bridge;
    sensor_msgs::msg::Image img_msg;
    std_msgs::msg::Header header;
    header.frame_id = "camera_link";
    img_bridge = cv_bridge::CvImage(header, encoding, frame);
    img_bridge.toImageMsg(img_msg);
    img_msg.header.stamp = stamp;
    return img_msg;
  }

  sensor_msgs::msg::CameraInfo get_calibration(
    dai::CameraBoardSocket socket, int width = 0, int height = 0,
    dai::Point2f top_left_pixel_id = {(0.0), (0.0)},
    dai::Point2f bottom_right_pixel_id = {(0.0), (0.0)})
  {
    dai::CalibrationHandler cal_data = device_->readCalibration();
    std::vector<std::vector<float>> intrinsics;
    sensor_msgs::msg::CameraInfo info;
    if (width == 0 || height == 0) {
      std::tie(intrinsics, width, height) = cal_data.getDefaultIntrinsics(socket);
    } else {
      intrinsics = cal_data.getCameraIntrinsics(
        socket, width, height, top_left_pixel_id,
        bottom_right_pixel_id);
    }
    info.height = height;
    info.width = width;
    std::copy(intrinsics[0].begin(), intrinsics[0].end(), info.k.begin());
    std::copy(intrinsics[1].begin(), intrinsics[1].end(), info.k.begin() + 3);

    auto dist = cal_data.getDistortionCoefficients(socket);

    for (const float d:dist) {
      info.d.push_back(static_cast<double>(d));
    }

    double tx = 0.0;
    double ty = 0.0;
    info.r[0] = info.r[4] = info.r[8] = 1;
    std::vector<std::vector<float>> rotation;
    std::copy(intrinsics[0].begin(), intrinsics[0].end(), info.p.begin());
    std::copy(intrinsics[1].begin(), intrinsics[1].end(), info.p.begin() + 4);
    if (socket == dai::CameraBoardSocket::LEFT) {
      rotation = cal_data.getStereoLeftRectificationRotation();
    } else if (socket == dai::CameraBoardSocket::RIGHT) {
      rotation = cal_data.getStereoRightRectificationRotation();
      std::vector<std::vector<float>> extrinsics = cal_data.getCameraExtrinsics(
        dai::CameraBoardSocket::RIGHT, dai::CameraBoardSocket::LEFT);
      tx = extrinsics[0][3] / 100.0;
    }

    for (size_t i = 0; i < rotation.size(); i++) {
      for (size_t j = 0; j < rotation[i].size(); j++) {
        info.r[i + j] = rotation[i][j];
      }
    }
    if (socket != dai::CameraBoardSocket::RGB) {
      std::copy(rotation[0].begin(), rotation[0].end(), info.r.begin());
      std::copy(rotation[1].begin(), rotation[1].end(), info.r.begin() + 3);
      std::copy(rotation[2].begin(), rotation[2].end(), info.r.begin() + 6);
    }
    info.p[3] = tx;
    info.p[7] = ty;
    info.p[11] = 0.0;
    info.distortion_model = "rational_polynomial";
    return info;
  }

  std::shared_ptr<dai::node::ColorCamera> camrgb_;
  std::shared_ptr<dai::node::MonoCamera> monoleft_;
  std::shared_ptr<dai::node::MonoCamera> monoright_;
  std::shared_ptr<dai::node::StereoDepth> stereo_;
  std::vector<std::string> label_map_;
  rclcpp::TimerBase::SharedPtr image_timer_;
  int depth_filter_size_;
  std::string nn_path_;
  std::string rgb_resolution_;
  std::string mono_resolution_;
  int counter_;
  int rgb_width_, rgb_height_;
  double fps_;
  std::string camera_frame_;
  const std::vector<std::string> default_label_map_ = {
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",
    "car", "cat", "chair", "cow", "diningtable", "dog", "horse",
    "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"};

private:
  virtual void timer_cb() = 0;
  virtual void setup_publishers() = 0;
  virtual void setup_pipeline() = 0;

  std::unordered_map<std::string, dai::ColorCameraProperties::SensorResolution>
  rgb_resolution_map = {
    {"1080", dai::ColorCameraProperties::SensorResolution::THE_1080_P},
    {"4k", dai::ColorCameraProperties::SensorResolution::THE_4_K},
    {"12MP", dai::ColorCameraProperties::SensorResolution::THE_12_MP},
  };

  std::unordered_map<std::string, dai::MonoCameraProperties::SensorResolution>
  mono_resolution_map = {
    {"400", dai::MonoCameraProperties::SensorResolution::THE_400_P},
    {"480", dai::MonoCameraProperties::SensorResolution::THE_480_P},
    {"720", dai::MonoCameraProperties::SensorResolution::THE_720_P},
    {"800", dai::MonoCameraProperties::SensorResolution::THE_800_P},
  };

};
}  // namespace depthai_ros_driver

#endif  // DEPTHAI_ROS_DRIVER__BASE_CAMERA_HPP_
