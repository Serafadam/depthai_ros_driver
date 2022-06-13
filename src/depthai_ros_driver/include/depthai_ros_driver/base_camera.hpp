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

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/properties/StereoDepthProperties.hpp"
#include "depthai-shared/properties/VideoEncoderProperties.hpp"
#include "depthai/depthai.hpp"
#include "depthai/device/DataQueue.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_event_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace depthai_ros_driver {
class BaseCamera : public rclcpp::Node {
public:
  explicit BaseCamera(
      const std::string &name,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : rclcpp::Node(name, options) {}
  virtual ~BaseCamera() {}
  virtual void on_configure() {}

  std::unique_ptr<dai::Device> device_;
  std::unique_ptr<dai::Pipeline> pipeline_;

  void start_the_device() {
    bool cam_setup = false;
    while (!cam_setup) {
      try {
        device_ = std::make_unique<dai::Device>(*pipeline_,
                                                dai::UsbSpeed::SUPER_PLUS);
        cam_setup = true;
      } catch (const std::runtime_error &e) {
        RCLCPP_ERROR(this->get_logger(), "Camera not found! Please connect it");
      }
    }
    RCLCPP_INFO(this->get_logger(), "Camera connected!");
  }

  virtual void setup_rgb() {
    camrgb_ = pipeline_->create<dai::node::ColorCamera>();
    RCLCPP_INFO(this->get_logger(), "Preview size %d", preview_size_);
    camrgb_->setPreviewSize(preview_size_, preview_size_);
    RCLCPP_INFO(this->get_logger(), "RGB width: %d, height: %d", rgb_width_,
                rgb_height_);
    camrgb_->setVideoSize(rgb_width_, rgb_height_);
    RCLCPP_INFO(this->get_logger(), "RGB resolution %s",
                rgb_resolution_.c_str());
    camrgb_->setResolution(rgb_resolution_map.at(rgb_resolution_));
    RCLCPP_INFO(this->get_logger(), "Interleaved: %d", set_interleaved_);
    camrgb_->setInterleaved(set_interleaved_);
    RCLCPP_INFO(this->get_logger(), "FPS: %f", fps_);
    camrgb_->setFps(fps_);
    RCLCPP_INFO(this->get_logger(), "Set ISP scale: %d", set_isp_);
    if (set_isp_) {
      camrgb_->setIspScale(2, 3);
    }
    RCLCPP_INFO(this->get_logger(), "Enable manual focus: %d", set_man_focus_);
    if (set_man_focus_) {
      RCLCPP_INFO(this->get_logger(), "Manual focus set: %d", man_focus_);
      camrgb_->initialControl.setManualFocus(man_focus_);
    }
    RCLCPP_INFO(this->get_logger(), "Keep preview aspect ratio: %d",
                set_preview_keep_aspect_ratio_);
    camrgb_->setPreviewKeepAspectRatio(set_preview_keep_aspect_ratio_);
  }
  virtual void setup_control_config_xin() {
    xin_control_ = pipeline_->create<dai::node::XLinkIn>();
    xin_config_ = pipeline_->create<dai::node::XLinkIn>();
    xin_control_->setStreamName(control_q_name_);
    xin_config_->setStreamName(config_q_name_);
    xin_control_->out.link(camrgb_->inputControl);
    xin_config_->out.link(camrgb_->inputConfig);
  }
  virtual void setup_stereo() {
    monoleft_ = pipeline_->create<dai::node::MonoCamera>();
    monoright_ = pipeline_->create<dai::node::MonoCamera>();
    stereo_ = pipeline_->create<dai::node::StereoDepth>();
    monoleft_->setResolution(mono_resolution_map.at(mono_resolution_));
    monoright_->setResolution(mono_resolution_map.at(mono_resolution_));
    monoleft_->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoright_->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    monoleft_->setFps(fps_);
    monoright_->setFps(fps_);
    auto median = static_cast<dai::MedianFilter>(depth_filter_size_);
    stereo_->setLeftRightCheck(lr_check_);
    stereo_->setDepthAlign(dai::CameraBoardSocket::RGB);
    stereo_->setDefaultProfilePreset(
        dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo_->initialConfig.setLeftRightCheckThreshold(lrc_threshold_);
    stereo_->initialConfig.setMedianFilter(median);
    stereo_->initialConfig.setConfidenceThreshold(stereo_conf_threshold_);
    stereo_->initialConfig.setSubpixel(subpixel_);
    stereo_->setExtendedDisparity(extended_disp_);
    stereo_->setRectifyEdgeFillColor(rectify_edge_fill_color_);
    auto config = stereo_->initialConfig.get();
    config.postProcessing.speckleFilter.enable = enable_speckle_filter_;
    config.postProcessing.speckleFilter.speckleRange = speckle_range_;
    config.postProcessing.temporalFilter.enable = enable_temporal_filter_;
    config.postProcessing.spatialFilter.enable = enable_spatial_filter_;
    config.postProcessing.spatialFilter.holeFillingRadius =
        hole_filling_radius_;
    config.postProcessing.spatialFilter.numIterations =
        spatial_filter_iterations_;
    config.postProcessing.thresholdFilter.minRange =
        threshold_filter_min_range_;
    config.postProcessing.thresholdFilter.maxRange =
        threshold_filter_max_range_;
    config.postProcessing.decimationFilter.decimationFactor =
        decimation_factor_;
    stereo_->initialConfig.set(config);

    monoleft_->out.link(stereo_->left);
    monoright_->out.link(stereo_->right);
  }
  void trig_rec_cb(const std_srvs::srv::Trigger::Request::SharedPtr req,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
    if (!record_) {
      RCLCPP_INFO(this->get_logger(), "Starting recording.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Stopping recording.");
      started_recording_ = false;
    }
    record_ = !record_;
  }
  void setup_recording() {
    video_enc_ = pipeline_->create<dai::node::VideoEncoder>();
    video_enc_->setDefaultProfilePreset(
        fps_, dai::VideoEncoderProperties::Profile::H264_MAIN);
    trigger_recording_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/trigger_recording",
        std::bind(&BaseCamera::trig_rec_cb, this, std::placeholders::_1,
                  std::placeholders::_2));
  }
  virtual void declare_rgb_params() {
    fps_ = this->declare_parameter<double>("fps", 30.0);
    rgb_frame_ =
        this->declare_parameter<std::string>("camera_frame", "camera_link");
    rgb_width_ = this->declare_parameter<int>("rgb_width", 1280);
    rgb_height_ = this->declare_parameter<int>("rgb_height", 720);
    label_map_ = this->declare_parameter<std::vector<std::string>>(
        "label_map", default_label_map_);
    depth_filter_size_ = this->declare_parameter<int>("depth_filter_size", 7);
    rgb_resolution_ =
        this->declare_parameter<std::string>("rgb_resolution", "1080");
    enable_rgb_pub_ = this->declare_parameter<bool>("enable_rgb", true);
    max_q_size_ = this->declare_parameter<int>("max_q_size", 4);
    set_isp_ = this->declare_parameter<bool>("set_isp", true);
    set_man_focus_ = this->declare_parameter<bool>("set_man_focus", true);
    man_focus_ = this->declare_parameter<int>("man_focus", 135);
    set_interleaved_ = this->declare_parameter<bool>("set_interleaved", false);
    set_preview_keep_aspect_ratio_ =
        this->declare_parameter<bool>("set_keep_preview_aspect_ratio", true);
    exposure_ = this->declare_parameter<int>("exposure", 1000);
  }

  virtual void declare_depth_params() {
    mono_resolution_ =
        this->declare_parameter<std::string>("mono_resolution", "400");
    preview_size_ = this->declare_parameter<int>("preview_size", 256);
    lr_check_ = this->declare_parameter<bool>("lr_check", true);
    lrc_threshold_ = this->declare_parameter<int>("lrc_threshold", 5);
    stereo_conf_threshold_ =
        this->declare_parameter<int>("stereo_conf_threshold", 255);
    subpixel_ = this->declare_parameter<bool>("subpixel", true);
    extended_disp_ = this->declare_parameter<bool>("extended_disparity", false);
    rectify_edge_fill_color_ =
        this->declare_parameter<int>("rectify_edge_fill_color", -1);
    enable_speckle_filter_ =
        this->declare_parameter<bool>("enable_speckle_filter", false);
    speckle_range_ = this->declare_parameter<int>("speckle_range", 50);
    enable_temporal_filter_ =
        this->declare_parameter<bool>("enable_temporal_filter", true);
    enable_spatial_filter_ =
        this->declare_parameter<bool>("enable_spatial_filter", true);
    hole_filling_radius_ =
        this->declare_parameter<int>("hole_filling_radius", 2);
    spatial_filter_iterations_ =
        this->declare_parameter<int>("spatial_filter_iterations", 1);
    threshold_filter_min_range_ =
        this->declare_parameter<int>("threshold_filter_min_range", 400);
    threshold_filter_max_range_ =
        this->declare_parameter<int>("threshold_filter_mix_range", 15000);
    decimation_factor_ = this->declare_parameter<int>("decimation_factor", 1);
    enable_depth_pub_ = this->declare_parameter<bool>("enable_depth", true);
    enable_lr_pub_ = this->declare_parameter<bool>("enable_lr", true);
    enable_recording_ =
        this->declare_parameter<bool>("enable_recording", false);
    align_depth_ = this->declare_parameter<bool>("align_depth", true);
  }

  virtual void declare_basic_params() {
    declare_rgb_params();
    declare_depth_params();
  }

  sensor_msgs::msg::Image convert_img_to_ros(const cv::Mat &frame,
                                             const char *encoding,
                                             const std::string &frame_id,
                                             rclcpp::Time stamp) {
    cv_bridge::CvImage img_bridge;
    sensor_msgs::msg::Image img_msg;
    std_msgs::msg::Header header;
    img_bridge = cv_bridge::CvImage(header, encoding, frame);
    img_bridge.toImageMsg(img_msg);
    img_msg.header.stamp = stamp;
    img_msg.header.frame_id = frame_id;
    return img_msg;
  }
  void publish_img(const cv::Mat &img, const char *encoding,
                   sensor_msgs::msg::CameraInfo &info,
                   const image_transport::CameraPublisher &pub) {
    auto curr_time = this->get_clock()->now();
    info.header.stamp = curr_time;
    pub.publish(
        convert_img_to_ros(img, encoding, info.header.frame_id, curr_time),
        info);
  }

  sensor_msgs::msg::CameraInfo
  get_calibration(dai::CameraBoardSocket socket, int width = 0, int height = 0,
                  dai::Point2f top_left_pixel_id = {(0.0), (0.0)},
                  dai::Point2f bottom_right_pixel_id = {(0.0), (0.0)}) {
    dai::CalibrationHandler cal_data = device_->readCalibration();
    std::vector<std::vector<float>> intrinsics;
    sensor_msgs::msg::CameraInfo info;
    if (width == 0 || height == 0) {
      std::tie(intrinsics, width, height) =
          cal_data.getDefaultIntrinsics(socket);
    } else {
      intrinsics = cal_data.getCameraIntrinsics(
          socket, width, height, top_left_pixel_id, bottom_right_pixel_id);
    }
    info.height = height;
    info.width = width;
    std::copy(intrinsics[0].begin(), intrinsics[0].end(), info.k.begin());
    std::copy(intrinsics[1].begin(), intrinsics[1].end(), info.k.begin() + 3);

    auto dist = cal_data.getDistortionCoefficients(socket);

    for (const float d : dist) {
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
    switch (socket) {
    case dai::CameraBoardSocket::RGB: {
      info.header.frame_id = rgb_frame_;
    }
    case dai::CameraBoardSocket::AUTO: {
      info.header.frame_id = rgb_frame_;
    }
    case dai::CameraBoardSocket::LEFT: {
      info.header.frame_id = left_frame_;
    }
    case dai::CameraBoardSocket::RIGHT: {
      info.header.frame_id = rgb_frame_;
    }
    }
    return info;
  }

  void setup_rgb_xout() {
    xout_rgb_ = pipeline_->create<dai::node::XLinkOut>();
    xout_rgb_->setStreamName(rgb_q_name_);
    camrgb_->video.link(xout_rgb_->input);
  }
  void setup_depth_xout() {
    xout_depth_ = pipeline_->create<dai::node::XLinkOut>();
    xout_depth_->setStreamName(depth_q_name_);
    stereo_->depth.link(xout_depth_->input);
  }

  void setup_lr_xout() {
    xout_left_ = pipeline_->create<dai::node::XLinkOut>();
    xout_left_->setStreamName(left_q_name_);
    monoleft_->out.link(xout_left_->input);
    xout_right_ = pipeline_->create<dai::node::XLinkOut>();
    xout_right_->setStreamName(right_q_name_);
    monoright_->out.link(xout_right_->input);
  }
  void setup_record_xout() {
    xout_enc_ = pipeline_->create<dai::node::XLinkOut>();
    xout_enc_->setStreamName(video_enc_q_name_);
    camrgb_->video.link(video_enc_->input);
    video_enc_->bitstream.link(xout_enc_->input);
  }
  void setup_all_xout_streams() {
    if (enable_rgb_pub_) {
      RCLCPP_INFO(this->get_logger(), "Enabling rgb pub.");
      setup_rgb_xout();
    }
    if (enable_depth_pub_) {
      RCLCPP_INFO(this->get_logger(), "Enabling depth pub.");
      setup_depth_xout();
    }
    if (enable_lr_pub_) {
      RCLCPP_INFO(this->get_logger(), "Enabling left & right pub.");
      setup_lr_xout();
    }
    if (enable_recording_) {
      RCLCPP_INFO(this->get_logger(), "Enabling recording.");
      setup_record_xout();
    }
  }
  void regular_queue_cb(const std::string &name,
                        const std::shared_ptr<dai::ADatatype> &data) {
    auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    cv::Mat cv_frame = frame->getCvFrame();
    if (name == rgb_q_name_) {
      publish_img(cv_frame, sensor_msgs::image_encodings::BGR8, rgb_info_,
                  rgb_pub_);
    } else if (name == depth_q_name_) {
      publish_img(cv_frame, sensor_msgs::image_encodings::TYPE_16UC1,
                  depth_info_, depth_pub_);
    } else if (name == left_q_name_) {
      publish_img(cv_frame, sensor_msgs::image_encodings::MONO8, left_info_,
                  left_pub_);
    } else if (name == right_q_name_) {
      publish_img(cv_frame, sensor_msgs::image_encodings::MONO8, right_info_,
                  right_pub_);
    }
  }

  void enable_rgb_q() {
    rgb_pub_ =
        image_transport::create_camera_publisher(this, "~/color/image_raw");
    rgb_info_ =
        get_calibration(dai::CameraBoardSocket::RGB, rgb_width_, rgb_height_);
    rgb_q_ = device_->getOutputQueue(rgb_q_name_, max_q_size_, false);
    rgb_q_->addCallback(std::bind(&BaseCamera::regular_queue_cb, this,
                                  std::placeholders::_1,
                                  std::placeholders::_2));
  }
  void enable_depth_q() {
    depth_pub_ =
        image_transport::create_camera_publisher(this, "~/depth/image_raw");
    if (align_depth_) {
      depth_info_ =
          get_calibration(dai::CameraBoardSocket::RGB, rgb_width_, rgb_height_);
    } else {
      depth_info_ = get_calibration(dai::CameraBoardSocket::RIGHT);
    }
    depth_q_ = device_->getOutputQueue(depth_q_name_, max_q_size_, false);
    depth_q_->addCallback(std::bind(&BaseCamera::regular_queue_cb, this,
                                    std::placeholders::_1,
                                    std::placeholders::_2));
  }
  void setup_lr_q() {
    left_pub_ =
        image_transport::create_camera_publisher(this, "~/left/image_raw");
    left_info_ = get_calibration(dai::CameraBoardSocket::LEFT);
    right_pub_ =
        image_transport::create_camera_publisher(this, "~/right/image_raw");
    right_info_ = get_calibration(dai::CameraBoardSocket::RIGHT);

    left_q_ = device_->getOutputQueue(left_q_name_, max_q_size_, false);
    left_q_->addCallback(std::bind(&BaseCamera::regular_queue_cb, this,
                                   std::placeholders::_1,
                                   std::placeholders::_2));

    right_q_ = device_->getOutputQueue(right_q_name_, max_q_size_, false);
    right_q_->addCallback(std::bind(&BaseCamera::regular_queue_cb, this,
                                    std::placeholders::_1,
                                    std::placeholders::_2));
  }
  void enc_cb(const std::string &name,
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
      auto enc_in = std::dynamic_pointer_cast<dai::ImgFrame>(data);
      video_file_.write((char *)(enc_in->getData().data()),
                        enc_in->getData().size());
    }
  }
  void setup_recording_q() {
    enc_q_ = device_->getOutputQueue(video_enc_q_name_, max_q_size_, false);
    enc_q_->addCallback(std::bind(&BaseCamera::enc_cb, this,
                                  std::placeholders::_1,
                                  std::placeholders::_2));
  }
  void setup_control_q() {
    control_q_ = device_->getInputQueue(control_q_name_);
  }
  rcl_interfaces::msg::SetParametersResult
  parameter_cb(const std::vector<rclcpp::Parameter> &params) {
    for (const auto &p : params) {
      if (p.get_name() == "exposure") {
        uint32_t exp = p.get_value<uint32_t>();
        dai::CameraControl ctrl;
        ctrl.setManualExposure(exp, 200);
        control_q_->send(ctrl);
      }
    }
  }
  void setup_config_q() { config_q_ = device_->getInputQueue(config_q_name_); }
  void setup_all_queues() {
    if (enable_rgb_pub_) {
      enable_rgb_q();
    }
    if (enable_depth_pub_) {
      enable_depth_q();
    }
    if (enable_lr_pub_) {
      setup_lr_q();
    }
    if (enable_recording_) {
      setup_recording_q();
    }
    setup_control_q();
    setup_config_q();
    param_cb_handle_ = this->add_on_set_parameters_callback(
        std::bind(&BaseCamera::parameter_cb, this, std::placeholders::_1));
  }
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_recording_srv_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  image_transport::CameraPublisher rgb_pub_, depth_pub_, left_pub_, right_pub_;
  sensor_msgs::msg::CameraInfo rgb_info_, depth_info_, left_info_, right_info_;
  std::shared_ptr<dai::node::ColorCamera> camrgb_;
  std::shared_ptr<dai::node::MonoCamera> monoleft_;
  std::shared_ptr<dai::node::MonoCamera> monoright_;
  std::shared_ptr<dai::node::StereoDepth> stereo_;
  std::shared_ptr<dai::node::XLinkOut> xout_rgb_, xout_depth_, xout_left_,
      xout_right_, xout_enc_;
  std::shared_ptr<dai::node::XLinkIn> xin_config_, xin_control_;
  std::shared_ptr<dai::node::VideoEncoder> video_enc_;
  std::shared_ptr<dai::DataOutputQueue> rgb_q_, depth_q_, left_q_, right_q_,
      enc_q_;
  std::shared_ptr<dai::DataInputQueue> config_q_, control_q_;
  std::vector<std::string> label_map_;
  int depth_filter_size_;
  std::string nn_path_;
  std::string rgb_resolution_;
  std::string mono_resolution_;
  int counter_;
  int rgb_width_, rgb_height_;
  int preview_size_;
  int max_q_size_;
  double fps_;
  bool set_interleaved_;
  bool set_preview_keep_aspect_ratio_;
  bool set_isp_;
  bool set_man_focus_;
  int man_focus_;
  bool lr_check_;
  int lrc_threshold_;
  int stereo_conf_threshold_;
  bool subpixel_;
  bool extended_disp_;
  bool enable_speckle_filter_;
  int rectify_edge_fill_color_;
  int speckle_range_;
  bool enable_temporal_filter_;
  bool enable_spatial_filter_;
  int hole_filling_radius_;
  int spatial_filter_iterations_;
  int threshold_filter_min_range_;
  int threshold_filter_max_range_;
  int decimation_factor_;
  int exposure_;
  std::ofstream video_file_;
  std::string rgb_frame_, left_frame_, right_frame_;
  const std::string rgb_q_name_ = "rgb";
  const std::string depth_q_name_ = "depth";
  const std::string left_q_name_ = "left";
  const std::string right_q_name_ = "right";
  const std::string video_enc_q_name_ = "h265";
  const std::string control_q_name_ = "control";
  const std::string config_q_name_ = "config";
  bool enable_rgb_pub_;
  bool enable_depth_pub_;
  bool enable_lr_pub_;
  bool enable_recording_;
  bool align_depth_;
  std::atomic<bool> record_, started_recording_;
  const std::vector<std::string> default_label_map_ = {
      "background", "aeroplane",   "bicycle", "bird",  "boat",
      "bottle",     "bus",         "car",     "cat",   "chair",
      "cow",        "diningtable", "dog",     "horse", "motorbike",
      "person",     "pottedplant", "sheep",   "sofa",  "train",
      "tvmonitor"};

private:
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
} // namespace depthai_ros_driver

#endif // DEPTHAI_ROS_DRIVER__BASE_CAMERA_HPP_
