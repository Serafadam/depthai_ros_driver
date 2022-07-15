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
#include "depthai_ros_driver/base_camera.hpp"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/UsbSpeed.hpp"
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/DeviceBase.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/SystemInformation.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/SystemLogger.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai/xlink/XLinkConnection.hpp"
#include "depthai_ros_driver/params_rgb.hpp"
#include "depthai_ros_driver/params_stereo.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "image_transport/camera_publisher.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace depthai_ros_driver
{
BaseCamera::BaseCamera(
  const std::string & name,
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: rclcpp::Node(name, options)
{
  RCLCPP_INFO(this->get_logger(), "Initialzing camera...");
  cam_running_ = false;
  pipeline_ = std::make_shared<dai::Pipeline>();
  start_cam_srv_ = this->create_service<Trigger>(
    "~/start_camera",
    std::bind(
      &BaseCamera::start_cb, this, std::placeholders::_1,
      std::placeholders::_2));
  shutdown_cam_srv_ = this->create_service<Trigger>(
    "~/shutdown_camera",
    std::bind(
      &BaseCamera::shutdown_cb, this, std::placeholders::_1,
      std::placeholders::_2));

  restart_cam_srv_ = this->create_service<Trigger>(
    "~/restart_camera",
    std::bind(
      &BaseCamera::restart_cb, this, std::placeholders::_1,
      std::placeholders::_2));
}

void BaseCamera::restart_cb(
  const Trigger::Request::SharedPtr /*req*/,
  Trigger::Response::SharedPtr res)
{
  restart_device();
  res->success = true;
}

void BaseCamera::start_cb(
  const Trigger::Request::SharedPtr /*req*/,
  Trigger::Response::SharedPtr res)
{
  if (cam_running_) {
    RCLCPP_INFO(this->get_logger(), "Camera already running.");
    return;
  }
  start_device();
  setup_all_queues();
  res->success = true;
  cam_running_ = true;
}
void BaseCamera::set_frame_ids()
{
  std::string frame_prefix = this->get_name();
  frame_prefix.append("_");
  RCLCPP_INFO(this->get_logger(), "Frame prefix: %s", frame_prefix.c_str());
  frame_ids_ = {
    {dai::CameraBoardSocket::RGB, frame_prefix + "color_frame"},
    {dai::CameraBoardSocket::AUTO, frame_prefix + "color_frame"},
    {dai::CameraBoardSocket::LEFT, frame_prefix + "left_frame"},
    {dai::CameraBoardSocket::RIGHT, frame_prefix + "right_frame"},
  };
}
void BaseCamera::shutdown_cb(
  const Trigger::Request::SharedPtr /*req*/,
  Trigger::Response::SharedPtr res)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down camera");
  if (!cam_running_) {
    RCLCPP_INFO(this->get_logger(), "Camera already shut down.");
    return;
  }

  shutdown_device();
  res->success = true;
  cam_running_ = false;
}

void BaseCamera::shutdown_device() {device_.reset();}

void BaseCamera::restart_device()
{
  shutdown_device();
  start_device();
  setup_all_queues();
}

void BaseCamera::create_pipeline()
{
  pipeline_ = std::make_shared<dai::Pipeline>();
}

void BaseCamera::start_device()
{
  bool cam_setup = false;
  dai::DeviceInfo info;
  bool device_found;
  if (!base_config_.camera_mxid.empty()) {
    info = dai::DeviceInfo(base_config_.camera_mxid);
  } else if (!base_config_.camera_ip.empty()) {
    info = dai::DeviceInfo(base_config_.camera_ip);
  }
  rclcpp::Rate r(1.0);
  while (!cam_setup) {
    try {
      if (base_config_.camera_mxid.empty() && base_config_.camera_ip.empty()) {
        device_ = std::make_shared<dai::Device>(*pipeline_, dai::UsbSpeed::SUPER_PLUS);
      } else {
        device_ = std::make_shared<dai::Device>(*pipeline_, info, dai::UsbSpeed::SUPER_PLUS);
      }
      cam_setup = true;
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(this->get_logger(), "Camera not found! Please connect it");
    }
    r.sleep();
  }
  cam_running_ = true;
  device_name_ = device_->getMxId();
  RCLCPP_INFO(this->get_logger(), "Camera %s connected!", device_name_.c_str());
}

void BaseCamera::setup_control_config_xin()
{
  xin_control_ = pipeline_->create<dai::node::XLinkIn>();
  xin_config_ = pipeline_->create<dai::node::XLinkIn>();
  xin_control_->setStreamName(control_q_name_);
  xin_config_->setStreamName(config_q_name_);
  xin_control_->out.link(camrgb_->inputControl);
  xin_config_->out.link(camrgb_->inputConfig);
}

void BaseCamera::setup_basic_devices()
{
  if (base_config_.enable_rgb) {
    setup_rgb();
  }
  if (base_config_.enable_depth || base_config_.enable_lr) {
    setup_stereo();
  }
  if (base_config_.enable_recording) {
    setup_recording();
  }
  if (base_config_.enable_imu) {
    setup_imu();
  }
  if (base_config_.enable_logging) {
    setup_logger();
    setup_logger_xout();
  }
}

void BaseCamera::setup_rgb()
{
  RCLCPP_INFO(this->get_logger(), "Creating RGB.");
  camrgb_ = pipeline_->create<dai::node::ColorCamera>();
  rgb_params_handler_->setup_rgb(camrgb_, this->get_logger());
  RCLCPP_INFO(this->get_logger(), "Created.");
}

void BaseCamera::setup_imu()
{
  RCLCPP_INFO(this->get_logger(), "Creating IMU.");
  imu_ = pipeline_->create<dai::node::IMU>();
  imu_->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 400);
  imu_->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
  imu_->enableIMUSensor(dai::IMUSensor::ROTATION_VECTOR, 400);
  imu_->setBatchReportThreshold(1);
  imu_->setMaxBatchReports(10);
}

void BaseCamera::setup_stereo()
{
  RCLCPP_INFO(this->get_logger(), "Creating depth.");
  mono_left_ = pipeline_->create<dai::node::MonoCamera>();
  mono_right_ = pipeline_->create<dai::node::MonoCamera>();
  stereo_ = pipeline_->create<dai::node::StereoDepth>();

  mono_left_->setBoardSocket(dai::CameraBoardSocket::LEFT);
  mono_right_->setBoardSocket(dai::CameraBoardSocket::RIGHT);
  stereo_params_handler_->setup_stereo(
    stereo_, mono_left_, mono_right_,
    this->get_logger());
  mono_left_->out.link(stereo_->left);
  mono_right_->out.link(stereo_->right);
  RCLCPP_INFO(this->get_logger(), "Created.");
}
void BaseCamera::setup_logger()
{
  RCLCPP_INFO(this->get_logger(), "Creating logger.");
  logger_ = pipeline_->create<dai::node::SystemLogger>();
  logger_->setRate(1.0);
  RCLCPP_INFO(this->get_logger(), "Created.");
}
void BaseCamera::trig_rec_cb(
  const Trigger::Request::SharedPtr /*req*/,
  Trigger::Response::SharedPtr /*res*/)
{
  if (!record_) {
    RCLCPP_INFO(this->get_logger(), "Starting recording.");
  } else {
    RCLCPP_INFO(this->get_logger(), "Stopping recording.");
    started_recording_ = false;
  }
  record_ = !record_;
}
void BaseCamera::setup_recording()
{
  video_enc_ = pipeline_->create<dai::node::VideoEncoder>();
  video_enc_->setDefaultProfilePreset(
    rgb_params_handler_->get_init_config().rgb_fps,
    dai::VideoEncoderProperties::Profile::H264_MAIN);
  trigger_recording_srv_ = this->create_service<Trigger>(
    "~/trigger_recording",
    std::bind(
      &BaseCamera::trig_rec_cb, this, std::placeholders::_1,
      std::placeholders::_2));
}
void BaseCamera::declare_rgb_depth_params()
{
  declare_common_params();
  rgb_params_handler_ = std::make_unique<rgb_params::RGBParamsHandler>();
  stereo_params_handler_ =
    std::make_unique<stereo_params::StereoParamsHandler>();
  rgb_params_handler_->declare_rgb_params(this);
  stereo_params_handler_->declare_depth_params(this);
  set_frame_ids();
}
void BaseCamera::setup_rgb_xout()
{
  xout_rgb_ = pipeline_->create<dai::node::XLinkOut>();
  xout_rgb_->setStreamName(rgb_q_name_);
  camrgb_->video.link(xout_rgb_->input);
}
void BaseCamera::setup_depth_xout()
{
  xout_depth_ = pipeline_->create<dai::node::XLinkOut>();
  xout_depth_->setStreamName(depth_q_name_);
  stereo_->depth.link(xout_depth_->input);
}

void BaseCamera::setup_lr_xout()
{
  xout_left_ = pipeline_->create<dai::node::XLinkOut>();
  xout_left_->setStreamName(left_q_name_);
  mono_left_->out.link(xout_left_->input);
  xout_right_ = pipeline_->create<dai::node::XLinkOut>();
  xout_right_->setStreamName(right_q_name_);
  mono_right_->out.link(xout_right_->input);
}
void BaseCamera::setup_record_xout()
{
  xout_enc_ = pipeline_->create<dai::node::XLinkOut>();
  xout_enc_->setStreamName(video_enc_q_name_);
  camrgb_->video.link(video_enc_->input);
  video_enc_->bitstream.link(xout_enc_->input);
}
void BaseCamera::setup_imu_xout()
{
  xout_imu_ = pipeline_->create<dai::node::XLinkOut>();
  xout_imu_->setStreamName(imu_q_name_);
  imu_->out.link(xout_imu_->input);
}
void BaseCamera::setup_logger_xout()
{
  xout_logger_ = pipeline_->create<dai::node::XLinkOut>();
  xout_logger_->setStreamName(logger_q_name_);
  logger_->out.link(xout_logger_->input);
}
void BaseCamera::setup_all_xout_streams()
{
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
  if (base_config_.enable_imu) {
    RCLCPP_INFO(this->get_logger(), "Enabling IMU");
    setup_imu_xout();
  }
}
sensor_msgs::msg::Image
BaseCamera::convert_img_to_ros(
  const cv::Mat & frame, const char * encoding,
  const std::string & frame_id,
  rclcpp::Time stamp)
{
  cv_bridge::CvImage img_bridge;
  sensor_msgs::msg::Image img_msg;
  std_msgs::msg::Header header;
  img_bridge = cv_bridge::CvImage(header, encoding, frame);
  img_bridge.toImageMsg(img_msg);
  img_msg.header.stamp = stamp;
  img_msg.header.frame_id = frame_id;
  return img_msg;
}
void BaseCamera::publish_img(
  const cv::Mat & img, const char * encoding,
  sensor_msgs::msg::CameraInfo & info,
  const image_transport::CameraPublisher & pub,
  rclcpp::Time stamp)
{
  info.header.stamp = stamp;
  pub.publish(
    convert_img_to_ros(
      img, encoding, info.header.frame_id,
      info.header.stamp),
    info);
}

void BaseCamera::regular_queue_cb(
  const std::string & name,
  const std::shared_ptr<dai::ADatatype> & data)
{
  auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(data);
  cv::Mat cv_frame = frame->getCvFrame();
  auto curr_time = this->get_clock()->now();
  if (name == rgb_q_name_) {
    publish_img(
      cv_frame, sensor_msgs::image_encodings::BGR8, rgb_info_,
      rgb_pub_, curr_time);
  } else if (name == depth_q_name_) {
    if (stereo_params_handler_->get_init_config().align_depth) {
      cv::resize(
        cv_frame, cv_frame,
        cv::Size(
          rgb_params_handler_->get_init_config().rgb_width,
          rgb_params_handler_->get_init_config().rgb_height));
    }
    publish_img(
      cv_frame, sensor_msgs::image_encodings::TYPE_16UC1, depth_info_,
      depth_pub_, curr_time);
  } else if (name == left_q_name_) {
    publish_img(
      cv_frame, sensor_msgs::image_encodings::MONO8, left_info_,
      left_pub_, curr_time);
  } else if (name == right_q_name_) {
    publish_img(
      cv_frame, sensor_msgs::image_encodings::MONO8, right_info_,
      right_pub_, curr_time);
  }
}
void BaseCamera::imu_cb(
  const std::string & name,
  const std::shared_ptr<dai::ADatatype> & data)
{
  auto imu_data = std::dynamic_pointer_cast<dai::IMUData>(data);
  auto packets = imu_data->packets;
  for (const auto & packet : packets) {
    auto accel = packet.acceleroMeter;
    auto gyro = packet.gyroscope;
    auto rot = packet.rotationVector;
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.linear_acceleration.x = accel.x;
    imu_msg.linear_acceleration.y = accel.y;
    imu_msg.linear_acceleration.z = accel.z;
    imu_msg.angular_velocity.x = gyro.x;
    imu_msg.angular_velocity.y = gyro.y;
    imu_msg.angular_velocity.z = gyro.z;
    imu_msg.header.frame_id = "camera_link";
    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.orientation.x = rot.i;
    imu_msg.orientation.y = rot.j;
    imu_msg.orientation.z = rot.k;
    imu_msg.orientation.w = rot.real;
    // this is based on covariances from BNO055, here we have BNO086, but I'm
    // not sure whether those will differ
    // Source:
    // https://github.com/Vijfendertig/rosserial_adafruit_bno055/blob/532b63db9b0e5e5e9217bd89905001fe979df3a4/src/imu_publisher/imu_publisher.cpp#L42.
    for (unsigned row = 0; row < 3; ++row) {
      for (unsigned col = 0; col < 3; ++col) {
        imu_msg.orientation_covariance[row * 3 + col] =
          (row == col ? 0.002 : 0.);   // +-2.5deg
        imu_msg.angular_velocity_covariance[row * 3 + col] =
          (row == col ? 0.003 : 0.);   // +-3deg/s
        imu_msg.linear_acceleration_covariance[row * 3 + col] =
          (row == col ? 0.60 : 0.);   // +-80mg
      }
    }
    imu_pub_->publish(imu_msg);
  }
}
void BaseCamera::logger_cb(
  const std::string & name,
  const std::shared_ptr<dai::ADatatype> & data)
{
  auto info = std::dynamic_pointer_cast<dai::SystemInformation>(data);
  std::stringstream msg;
  msg << "Ddr used / total - " <<
    info->ddrMemoryUsage.used / (1024.0f * 1024.0f) << "/" <<
    info->ddrMemoryUsage.total / (1024.0f * 1024.0f) << "MiB" <<
    "\n";
  msg << "Cmx used / total - " <<
    info->cmxMemoryUsage.used / (1024.0f * 1024.0f) << "/" <<
    info->cmxMemoryUsage.total / (1024.0f * 1024.0f) << "MiB" <<
    "\n";
  msg << "LeonCss heap used / total - " <<
    info->leonCssMemoryUsage.used / (1024.0f * 1024.0f) << "/" <<
    info->leonCssMemoryUsage.total / (1024.0f * 1024.0f) << "MiB" <<
    "\n";
  msg << "LeonMss heap used / total - " <<
    info->leonMssMemoryUsage.used / (1024.0f * 1024.0f) << "/" <<
    info->leonMssMemoryUsage.total / (1024.0f * 1024.0f) << "MiB" <<
    "\n";
  const auto & t = info->chipTemperature;
  msg << "Chip temperature - average: " << t.average << " css: " << t.css <<
    " mss: " << t.mss << " upa: " << t.upa << " dss: " << t.dss << "\n";
  msg << "Cpu usage - Leon CSS: " << info->leonCssCpuUsage.average * 100 <<
    "%% Leon MSS: " << info->leonMssCpuUsage.average * 100 << "%%" <<
    "\n";
  DiagnosticArray diag;
  diag.header.stamp = this->get_clock()->now();
  diag.status.resize(1);
  diag.status[0].level = diag.status[0].OK;
  diag.status[0].name = this->get_name() + device_name_;
  diag.status[0].hardware_id = device_name_;
  diag.status[0].message = msg.str();
  diag_pub_->publish(diag);
}
sensor_msgs::msg::CameraInfo
BaseCamera::get_calibration(
  const std::string & frame_id,
  dai::CameraBoardSocket socket, int width,
  int height, dai::Point2f top_left_pixel_id,
  dai::Point2f bottom_right_pixel_id)
{
  return calibration::get_calibration(
    device_, frame_id, socket, width, height,
    top_left_pixel_id, bottom_right_pixel_id);
}
void BaseCamera::enable_rgb_q()
{
  rgb_pub_ =
    image_transport::create_camera_publisher(this, "~/color/image_raw");
  rgb_info_ = get_calibration(
    frame_ids_.at(dai::CameraBoardSocket::RGB), dai::CameraBoardSocket::RGB,
    rgb_params_handler_->get_init_config().rgb_width,
    rgb_params_handler_->get_init_config().rgb_height);
  rgb_q_ = device_->getOutputQueue(rgb_q_name_, base_config_.max_q_size, false);
  rgb_q_->addCallback(
    std::bind(
      &BaseCamera::regular_queue_cb, this,
      std::placeholders::_1, std::placeholders::_2));
}
void BaseCamera::enable_depth_q()
{
  depth_pub_ =
    image_transport::create_camera_publisher(this, "~/depth/image_raw");
  if (stereo_params_handler_->get_init_config().align_depth) {
    depth_info_ = get_calibration(
      frame_ids_.at(dai::CameraBoardSocket::RGB), dai::CameraBoardSocket::RGB,
      rgb_params_handler_->get_init_config().rgb_width,
      rgb_params_handler_->get_init_config().rgb_height);
  } else {
    depth_info_ = get_calibration(
      frame_ids_.at(dai::CameraBoardSocket::RIGHT),
      dai::CameraBoardSocket::RIGHT);
  }
  depth_q_ =
    device_->getOutputQueue(depth_q_name_, base_config_.max_q_size, false);
  depth_q_->addCallback(
    std::bind(
      &BaseCamera::regular_queue_cb, this,
      std::placeholders::_1,
      std::placeholders::_2));
}
void BaseCamera::setup_lr_q()
{
  left_pub_ =
    image_transport::create_camera_publisher(this, "~/left/image_raw");
  left_info_ = get_calibration(
    frame_ids_.at(dai::CameraBoardSocket::LEFT),
    dai::CameraBoardSocket::LEFT);
  right_pub_ =
    image_transport::create_camera_publisher(this, "~/right/image_raw");
  right_info_ = get_calibration(
    frame_ids_.at(dai::CameraBoardSocket::RIGHT),
    dai::CameraBoardSocket::RIGHT);

  left_q_ =
    device_->getOutputQueue(left_q_name_, base_config_.max_q_size, false);
  left_q_->addCallback(
    std::bind(
      &BaseCamera::regular_queue_cb, this,
      std::placeholders::_1, std::placeholders::_2));

  right_q_ =
    device_->getOutputQueue(right_q_name_, base_config_.max_q_size, false);
  right_q_->addCallback(
    std::bind(
      &BaseCamera::regular_queue_cb, this,
      std::placeholders::_1,
      std::placeholders::_2));
}
void BaseCamera::setup_imu_q()
{
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("~/imu/data", 10);
  imu_q_ = device_->getOutputQueue(imu_q_name_, base_config_.max_q_size, false);
  imu_q_->addCallback(
    std::bind(
      &BaseCamera::imu_cb, this,
      std::placeholders::_1, std::placeholders::_2));
}
void BaseCamera::enc_cb(
  const std::string & name,
  const std::shared_ptr<dai::ADatatype> & data)
{
  if (record_) {
    if (!started_recording_) {
      std::stringstream current_time;
      auto now = std::chrono::system_clock::now();
      auto in_time_t = std::chrono::system_clock::to_time_t(now);
      current_time << std::put_time(
        std::localtime(&in_time_t),
        "%Y_%m_%d_%H_%M_%S");
      current_time << ".h265";
      video_file_ = std::ofstream(current_time.str(), std::ios::binary);
      started_recording_ = true;
    }
    auto enc_in = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    video_file_.write(
      (char *)(enc_in->getData().data()),
      enc_in->getData().size());
  }
}

void BaseCamera::setup_recording_q()
{
  enc_q_ = device_->getOutputQueue(
    video_enc_q_name_, base_config_.max_q_size,
    false);
  enc_q_->addCallback(
    std::bind(
      &BaseCamera::enc_cb, this,
      std::placeholders::_1, std::placeholders::_2));
}
void BaseCamera::setup_control_q()
{
  control_q_ = device_->getInputQueue(control_q_name_);
}
rcl_interfaces::msg::SetParametersResult
BaseCamera::parameter_cb(const std::vector<rclcpp::Parameter> & params)
{
  rgb_params_handler_->set_runtime_config(params);
  auto ctrl = rgb_params_handler_->get_rgb_control();
  control_q_->send(ctrl);
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;
  return res;
}
void BaseCamera::setup_config_q()
{
  config_q_ = device_->getInputQueue(config_q_name_);
}
void BaseCamera::setup_logger_q()
{
  diag_pub_ = this->create_publisher<DiagnosticArray>("~/diagnostics", 10);

  logger_q_ =
    device_->getOutputQueue(logger_q_name_, base_config_.max_q_size, false);
  logger_q_->addCallback(
    std::bind(
      &BaseCamera::logger_cb, this,
      std::placeholders::_1,
      std::placeholders::_2));
}
void BaseCamera::setup_all_queues()
{
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
  if (base_config_.enable_imu) {
    setup_imu_q();
  }
  if (base_config_.enable_logging) {
    setup_logger_q();
  }
  setup_control_q();
  setup_config_q();
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&BaseCamera::parameter_cb, this, std::placeholders::_1));
}
std::shared_ptr<dai::Pipeline> BaseCamera::get_pipeline() {return pipeline_;}
std::shared_ptr<dai::DataOutputQueue>
BaseCamera::get_output_q(
  const std::string & q_name, uint8_t max_q_size,
  bool blocking)
{
  return device_->getOutputQueue(q_name, max_q_size, blocking);
}
std::string BaseCamera::get_frame_id(const dai::CameraBoardSocket & socket)
{
  return frame_ids_.at(socket);
}
std::vector<std::string> BaseCamera::get_default_label_map()
{
  return default_label_map_;
}

void BaseCamera::link_nn(std::shared_ptr<dai::node::NeuralNetwork> nn)
{
  camrgb_->preview.link(nn->input);
}
void BaseCamera::link_spatial_detection(
  std::shared_ptr<dai::node::SpatialDetectionNetwork> nn)
{
  camrgb_->preview.link(nn->input);
  stereo_->depth.link(nn->inputDepth);
}
void BaseCamera::override_init_rgb_config(
  const rgb_params::RGBInitConfig & config)
{
  rgb_params_handler_->set_init_config(config);
}
void BaseCamera::override_runtime_rgb_config(
  const rgb_params::RGBRuntimeConfig & config)
{
  rgb_params_handler_->set_runtime_config(config);
}
void BaseCamera::override_init_stereo_config(
  const stereo_params::StereoInitConfig & config)
{
  stereo_params_handler_->set_init_config(config);
}
void BaseCamera::override_runtime_stereo_config(
  const stereo_params::StereoRuntimeConfig & config)
{
  stereo_params_handler_->set_runtime_config(config);
}
void BaseCamera::declare_common_params()
{
  base_config_.enable_rgb =
    this->declare_parameter<bool>(base_param_names_.enable_rgb, true);
  base_config_.enable_depth =
    this->declare_parameter<bool>(base_param_names_.enable_depth, true);
  base_config_.enable_lr =
    this->declare_parameter<bool>(base_param_names_.enable_lr, true);
  base_config_.enable_imu =
    this->declare_parameter<bool>(base_param_names_.enable_imu, false);
  base_config_.enable_recording =
    this->declare_parameter<bool>(base_param_names_.enable_recording, false);
  base_config_.enable_logging =
    this->declare_parameter<bool>(base_param_names_.enable_logging, false);
  base_config_.max_q_size =
    this->declare_parameter<int>(base_param_names_.max_q_size, 4);
  base_config_.camera_mxid =
    this->declare_parameter<std::string>(base_param_names_.camera_mxid, "");
  base_config_.camera_ip =
    this->declare_parameter<std::string>(base_param_names_.camera_ip, "");
}

} // namespace depthai_ros_driver
