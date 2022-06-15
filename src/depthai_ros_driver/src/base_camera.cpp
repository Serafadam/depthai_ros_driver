// Copyright (c) [2022] [Adam Serafin]

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of node software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and node permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
#include "cv_bridge/cv_bridge.h"

#include <cstdint>
#include <memory>

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai_ros_driver/base_camera.hpp"
#include "depthai_ros_driver/params_rgb.hpp"
#include "depthai_ros_driver/params_stereo.hpp"
#include "image_transport/camera_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace depthai_ros_driver {
BaseCamera::BaseCamera(
    const std::string &name,
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : rclcpp::Node(name, options) {}
void BaseCamera::create_pipeline() {
  pipeline_ = std::make_unique<dai::Pipeline>();
}
void BaseCamera::start_the_device() {
  bool cam_setup = false;
  while (!cam_setup) {
    try {
      device_ =
          std::make_unique<dai::Device>(*pipeline_, dai::UsbSpeed::SUPER_PLUS);
      cam_setup = true;
    } catch (const std::runtime_error &e) {
      RCLCPP_ERROR(this->get_logger(), "Camera not found! Please connect it");
    }
  }
  RCLCPP_INFO(this->get_logger(), "Camera connected!");
}
void BaseCamera::setup_control_config_xin() {
  xin_control_ = pipeline_->create<dai::node::XLinkIn>();
  xin_config_ = pipeline_->create<dai::node::XLinkIn>();
  xin_control_->setStreamName(control_q_name_);
  xin_config_->setStreamName(config_q_name_);
  xin_control_->out.link(camrgb_->inputControl);
  xin_config_->out.link(camrgb_->inputConfig);
}
void BaseCamera::setup_rgb() {
  RCLCPP_INFO(this->get_logger(), "Creating RGB cam object");
  camrgb_ = pipeline_->create<dai::node::ColorCamera>();
  auto pn = rgb_params_->get_param_names();
  rgb_params_->set_init_config(this->get_parameters(pn.name_vector));
  rgb_params_->setup_rgb(camrgb_, this->get_logger());
}
void BaseCamera::setup_stereo() {
  mono_left_ = pipeline_->create<dai::node::MonoCamera>();
  mono_right_ = pipeline_->create<dai::node::MonoCamera>();
  stereo_ = pipeline_->create<dai::node::StereoDepth>();

  mono_left_->setBoardSocket(dai::CameraBoardSocket::LEFT);
  mono_right_->setBoardSocket(dai::CameraBoardSocket::RIGHT);
  auto pn = stereo_params_->get_param_names();
  stereo_params_->set_init_config(this->get_parameters(pn.name_vector));
  stereo_params_->setup_stereo(stereo_, mono_left_, mono_right_,
                               this->get_logger());
  mono_left_->out.link(stereo_->left);
  mono_right_->out.link(stereo_->right);
}
void BaseCamera::trig_rec_cb(
    const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
    std_srvs::srv::Trigger::Response::SharedPtr /*res*/) {
  if (!record_) {
    RCLCPP_INFO(this->get_logger(), "Starting recording.");
  } else {
    RCLCPP_INFO(this->get_logger(), "Stopping recording.");
    started_recording_ = false;
  }
  record_ = !record_;
}
void BaseCamera::setup_recording() {
  video_enc_ = pipeline_->create<dai::node::VideoEncoder>();
  video_enc_->setDefaultProfilePreset(
      rgb_params_->get_init_config().rgb_fps,
      dai::VideoEncoderProperties::Profile::H264_MAIN);
  trigger_recording_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "~/trigger_recording",
      std::bind(&BaseCamera::trig_rec_cb, this, std::placeholders::_1,
                std::placeholders::_2));
}
void BaseCamera::declare_rgb_depth_params() {
  declare_common_params();
  rgb_params_ = std::make_unique<rgb_params::RGBParams>();
  stereo_params_ = std::make_unique<stereo_params::StereoParams>();
  declare_rgb_params();
  declare_depth_params();
}
void BaseCamera::setup_rgb_xout() {
  xout_rgb_ = pipeline_->create<dai::node::XLinkOut>();
  xout_rgb_->setStreamName(rgb_q_name_);
  camrgb_->video.link(xout_rgb_->input);
}
void BaseCamera::setup_depth_xout() {
  xout_depth_ = pipeline_->create<dai::node::XLinkOut>();
  xout_depth_->setStreamName(depth_q_name_);
  stereo_->depth.link(xout_depth_->input);
}

void BaseCamera::setup_lr_xout() {
  xout_left_ = pipeline_->create<dai::node::XLinkOut>();
  xout_left_->setStreamName(left_q_name_);
  mono_left_->out.link(xout_left_->input);
  xout_right_ = pipeline_->create<dai::node::XLinkOut>();
  xout_right_->setStreamName(right_q_name_);
  mono_right_->out.link(xout_right_->input);
}
void BaseCamera::setup_record_xout() {
  xout_enc_ = pipeline_->create<dai::node::XLinkOut>();
  xout_enc_->setStreamName(video_enc_q_name_);
  camrgb_->video.link(video_enc_->input);
  video_enc_->bitstream.link(xout_enc_->input);
}
void BaseCamera::setup_all_xout_streams() {
  if (base_config_.enable_rgb) {
    RCLCPP_INFO(this->get_logger(), "Enabling rgb pub.");
    setup_rgb_xout();
  }
  if (base_config_.enable_depth) {
    RCLCPP_INFO(this->get_logger(), "Enabling depth pub.");
    setup_depth_xout();
  }
  if (base_config_.enable_lr) {
    RCLCPP_INFO(this->get_logger(), "Enabling left & right pub.");
    setup_lr_xout();
  }
  if (base_config_.enable_recording) {
    RCLCPP_INFO(this->get_logger(), "Enabling recording.");
    setup_record_xout();
  }
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
                 const image_transport::CameraPublisher &pub,
                 rclcpp::Time stamp) {
  info.header.stamp = stamp;
  pub.publish(convert_img_to_ros(img, encoding, info.header.frame_id,
                                 info.header.stamp),
              info);
}

void BaseCamera::regular_queue_cb(const std::string &name,
                                  const std::shared_ptr<dai::ADatatype> &data) {
  auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(data);
  cv::Mat cv_frame = frame->getCvFrame();
  auto curr_time = this->get_clock()->now();
  if (name == rgb_q_name_) {
    publish_img(cv_frame, sensor_msgs::image_encodings::BGR8, rgb_info_,
                rgb_pub_, curr_time);
  } else if (name == depth_q_name_) {
    publish_img(cv_frame, sensor_msgs::image_encodings::TYPE_16UC1, depth_info_,
                depth_pub_, curr_time);
  } else if (name == left_q_name_) {
    publish_img(cv_frame, sensor_msgs::image_encodings::MONO8, left_info_,
                left_pub_, curr_time);
  } else if (name == right_q_name_) {
    publish_img(cv_frame, sensor_msgs::image_encodings::MONO8, right_info_,
                right_pub_, curr_time);
  }
}
void BaseCamera::enable_rgb_q() {
  rgb_pub_ =
      image_transport::create_camera_publisher(this, "~/color/image_raw");
  rgb_info_ = get_calibration(device_, dai::CameraBoardSocket::RGB,
                              rgb_params_->get_init_config().rgb_width,
                              rgb_params_->get_init_config().rgb_height);
  rgb_q_ = device_->getOutputQueue(rgb_q_name_, base_config_.max_q_size, false);
  rgb_q_->addCallback(std::bind(&BaseCamera::regular_queue_cb, this,
                                std::placeholders::_1, std::placeholders::_2));
}
void BaseCamera::enable_depth_q() {
  depth_pub_ =
      image_transport::create_camera_publisher(this, "~/depth/image_raw");
  if (stereo_params_->get_init_config().align_depth) {
    depth_info_ = get_calibration(device_, dai::CameraBoardSocket::RGB,
                                  rgb_params_->get_init_config().rgb_width,
                                  rgb_params_->get_init_config().rgb_height);
  } else {
    depth_info_ = get_calibration(device_, dai::CameraBoardSocket::RIGHT);
  }
  depth_q_ =
      device_->getOutputQueue(depth_q_name_, base_config_.max_q_size, false);
  depth_q_->addCallback(std::bind(&BaseCamera::regular_queue_cb, this,
                                  std::placeholders::_1,
                                  std::placeholders::_2));
}
void BaseCamera::setup_lr_q() {
  left_pub_ =
      image_transport::create_camera_publisher(this, "~/left/image_raw");
  left_info_ = get_calibration(device_, dai::CameraBoardSocket::LEFT);
  right_pub_ =
      image_transport::create_camera_publisher(this, "~/right/image_raw");
  right_info_ = get_calibration(device_, dai::CameraBoardSocket::RIGHT);

  left_q_ =
      device_->getOutputQueue(left_q_name_, base_config_.max_q_size, false);
  left_q_->addCallback(std::bind(&BaseCamera::regular_queue_cb, this,
                                 std::placeholders::_1, std::placeholders::_2));

  right_q_ =
      device_->getOutputQueue(right_q_name_, base_config_.max_q_size, false);
  right_q_->addCallback(std::bind(&BaseCamera::regular_queue_cb, this,
                                  std::placeholders::_1,
                                  std::placeholders::_2));
}
void BaseCamera::enc_cb(const std::string &name,
                        const std::shared_ptr<dai::ADatatype> &data) {
  if (record_) {
    if (!started_recording_) {
      std::stringstream current_time;
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

void BaseCamera::setup_recording_q() {
  enc_q_ = device_->getOutputQueue(video_enc_q_name_, base_config_.max_q_size,
                                   false);
  enc_q_->addCallback(std::bind(&BaseCamera::enc_cb, this,
                                std::placeholders::_1, std::placeholders::_2));
}
void BaseCamera::setup_control_q() {
  control_q_ = device_->getInputQueue(control_q_name_);
}
rcl_interfaces::msg::SetParametersResult
BaseCamera::parameter_cb(const std::vector<rclcpp::Parameter> &params) {
  rgb_params_->set_runtime_config(params);
  auto ctrl = rgb_params_->get_rgb_control();
  control_q_->send(ctrl);
}
void BaseCamera::setup_config_q() {
  config_q_ = device_->getInputQueue(config_q_name_);
}
void BaseCamera::setup_all_queues() {
  if (base_config_.enable_rgb) {
    enable_rgb_q();
  }
  if (base_config_.enable_depth) {
    enable_depth_q();
  }
  if (base_config_.enable_lr) {
    setup_lr_q();
  }
  if (base_config_.enable_recording) {
    setup_recording_q();
  }
  setup_control_q();
  setup_config_q();
  param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&BaseCamera::parameter_cb, this, std::placeholders::_1));
}
void BaseCamera::declare_depth_params() {
  auto pn = stereo_params_->get_param_names();
  this->declare_parameter<double>(pn.mono_fps, 60.0);
  this->declare_parameter<std::string>(pn.mono_resolution, "400");
  this->declare_parameter<bool>(base_param_names_.align_depth, true);
  this->declare_parameter<bool>(pn.lr_check, true);
  this->declare_parameter<int>(pn.lrc_threshold, 5);
  this->declare_parameter<int>(pn.depth_filter_size, 7);
  this->declare_parameter<int>(pn.stereo_conf_threshold, 255);
  this->declare_parameter<bool>(pn.subpixel, true);
  this->declare_parameter<bool>(pn.extended_disp, false);
  this->declare_parameter<int>(pn.rectify_edge_fill_color, -1);
  this->declare_parameter<bool>(pn.enable_speckle_filter, false);
  this->declare_parameter<int>(pn.speckle_range, 50);
  this->declare_parameter<bool>(pn.enable_temporal_filter, true);
  this->declare_parameter<bool>(pn.enable_spatial_filter, true);
  this->declare_parameter<int>(pn.hole_filling_radius, 2);
  this->declare_parameter<int>(pn.spatial_filter_iterations, 1);
  this->declare_parameter<int>(pn.threshold_filter_min_range, 400);
  this->declare_parameter<int>(pn.threshold_filter_max_range, 15000);
  this->declare_parameter<int>(pn.decimation_factor, 1);
}
void BaseCamera::declare_rgb_params() {
  auto pn = rgb_params_->get_param_names();
  this->declare_parameter<double>(pn.rgb_fps, 30.0);
  this->declare_parameter<int>(pn.rgb_width, 1280);
  this->declare_parameter<int>(pn.rgb_height, 720);
  this->declare_parameter<int>(pn.preview_size, 256);
  this->declare_parameter<std::string>(pn.rgb_resolution, "1080");
  this->declare_parameter<bool>(pn.set_isp, true);
  this->declare_parameter<bool>(pn.set_man_focus, true);
  this->declare_parameter<int>(pn.man_focus, 135);
  this->declare_parameter<bool>(pn.interleaved, false);
  this->declare_parameter<bool>(pn.keep_preview_aspect_ratio, true);
  this->declare_parameter<int>(pn.rgb_exposure, 1000);
  this->declare_parameter<int>(pn.rgb_iso, 100);
  this->declare_parameter<bool>(pn.set_man_exposure, false);
}
void BaseCamera::declare_common_params() {
  base_config_.enable_rgb =
      this->declare_parameter<bool>(base_param_names_.enable_rgb, true);
  base_config_.enable_depth =
      this->declare_parameter<bool>(base_param_names_.enable_depth, true);
  base_config_.enable_lr =
      this->declare_parameter<bool>(base_param_names_.enable_lr, true);
  base_config_.enable_recording =
      this->declare_parameter<bool>(base_param_names_.enable_recording, false);
  base_config_.max_q_size =
      this->declare_parameter<int>(base_param_names_.max_q_size, 4);
}

} // namespace depthai_ros_driver