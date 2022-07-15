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

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/properties/StereoDepthProperties.hpp"
#include "depthai-shared/properties/VideoEncoderProperties.hpp"
#include "depthai/depthai.hpp"
#include "depthai/device/DataQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/SystemLogger.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_ros_driver/calibration.hpp"
#include "depthai_ros_driver/params_rgb.hpp"
#include "depthai_ros_driver/params_stereo.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_event_handler.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace depthai_ros_driver
{
struct BaseCameraConfig
{
  int max_q_size;
  bool enable_rgb;
  bool enable_depth;
  bool enable_lr;
  bool enable_imu;
  bool enable_recording;
  bool enable_logging;
  std::string camera_mxid;
  std::string camera_ip;
};
struct BaseCameraParamNames
{
  const std::string max_q_size = "i_max_q_size";
  const std::string enable_rgb = "i_enable_rgb";
  const std::string enable_depth = "i_enable_depth";
  const std::string enable_lr = "i_enable_lr";
  const std::string enable_imu = "i_enable_imu";
  const std::string enable_recording = "i_enable_recording";
  const std::string enable_logging = "i_enable_logging";
  const std::string camera_mxid = "i_camera_mxid";
  const std::string camera_ip = "i_camera_ip";
};
class BaseCamera : public rclcpp::Node
{
  using Trigger = std_srvs::srv::Trigger;
  using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;

public:
  explicit BaseCamera(const std::string & name, const rclcpp::NodeOptions & options);
  virtual ~BaseCamera() {}
  virtual void on_configure() {}

  void create_pipeline();
  virtual void start_device();
  virtual void shutdown_device();
  virtual void restart_device();
  virtual void setup_control_config_xin();
  virtual void setup_basic_devices();
  virtual void setup_rgb();
  virtual void setup_imu();
  virtual void setup_stereo();
  virtual void setup_logger();
  void start_cb(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);
  void shutdown_cb(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);
  void restart_cb(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);
  void trig_rec_cb(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);
  void setup_recording();

  virtual void declare_rgb_depth_params();
  virtual void declare_common_params();

  virtual void setup_rgb_xout();
  virtual void setup_depth_xout();
  virtual void setup_lr_xout();
  virtual void setup_imu_xout();
  virtual void setup_logger_xout();
  virtual void setup_record_xout();
  virtual void setup_all_xout_streams();
  void regular_queue_cb(const std::string & name, const std::shared_ptr<dai::ADatatype> & data);
  void enable_rgb_q();
  void enable_depth_q();
  void setup_imu_q();
  void setup_lr_q();
  void setup_recording_q();
  void setup_control_q();
  void setup_logger_q();
  rcl_interfaces::msg::SetParametersResult parameter_cb(
    const std::vector<rclcpp::Parameter> & params);
  void setup_config_q();
  void setup_all_queues();
  void override_init_rgb_config(const rgb_params::RGBInitConfig & config);
  void override_runtime_rgb_config(const rgb_params::RGBRuntimeConfig & config);
  void override_init_stereo_config(const stereo_params::StereoInitConfig & config);
  void override_runtime_stereo_config(const stereo_params::StereoRuntimeConfig & config);
  void override_base_config(const BaseCameraConfig & config);

  std::shared_ptr<dai::Pipeline> get_pipeline();
  void link_nn(std::shared_ptr<dai::node::NeuralNetwork> nn);
  void link_spatial_detection(std::shared_ptr<dai::node::SpatialDetectionNetwork> nn);
  std::string get_frame_id(const dai::CameraBoardSocket & socket);
  std::shared_ptr<dai::DataOutputQueue> get_output_q(
    const std::string & q_name, uint8_t max_q_size = 4, bool blocking = false);
  std::vector<std::string> get_default_label_map();
  void publish_img(
    const cv::Mat & img, const char * encoding, sensor_msgs::msg::CameraInfo & info,
    const image_transport::CameraPublisher & pub, rclcpp::Time stamp);
  sensor_msgs::msg::CameraInfo get_calibration(
    const std::string & frame_id, dai::CameraBoardSocket socket, int width = 0, int height = 0,
    dai::Point2f top_left_pixel_id = {(0.0), (0.0)},
    dai::Point2f bottom_right_pixel_id = {(0.0), (0.0)});

private:
  sensor_msgs::msg::Image convert_img_to_ros(
    const cv::Mat & frame, const char * encoding, const std::string & frame_id, rclcpp::Time stamp);

  void enc_cb(const std::string & name, const std::shared_ptr<dai::ADatatype> & data);
  void imu_cb(const std::string & name, const std::shared_ptr<dai::ADatatype> & data);
  void logger_cb(const std::string & name, const std::shared_ptr<dai::ADatatype> & data);

  void set_frame_ids();
  rclcpp::Service<Trigger>::SharedPtr trigger_recording_srv_, restart_cam_srv_, start_cam_srv_,
    shutdown_cam_srv_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  image_transport::CameraPublisher rgb_pub_, depth_pub_, left_pub_, right_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diag_pub_;
  sensor_msgs::msg::CameraInfo rgb_info_, depth_info_, left_info_, right_info_;
  std::unique_ptr<rgb_params::RGBParamsHandler> rgb_params_handler_;
  std::unique_ptr<stereo_params::StereoParamsHandler> stereo_params_handler_;
  BaseCameraConfig base_config_;
  BaseCameraParamNames base_param_names_;
  std::shared_ptr<dai::Pipeline> pipeline_;
  std::shared_ptr<dai::Device> device_;
  std::shared_ptr<dai::node::ColorCamera> camrgb_;
  std::shared_ptr<dai::node::MonoCamera> mono_left_;
  std::shared_ptr<dai::node::MonoCamera> mono_right_;
  std::shared_ptr<dai::node::StereoDepth> stereo_;
  std::shared_ptr<dai::node::SystemLogger> logger_;
  std::shared_ptr<dai::node::IMU> imu_;
  std::shared_ptr<dai::node::XLinkOut> xout_rgb_, xout_depth_, xout_left_, xout_right_, xout_enc_,
    xout_imu_, xout_logger_;
  std::shared_ptr<dai::node::XLinkIn> xin_config_, xin_control_;
  std::shared_ptr<dai::node::VideoEncoder> video_enc_;
  std::shared_ptr<dai::DataOutputQueue> rgb_q_, depth_q_, left_q_, right_q_, enc_q_, imu_q_,
    logger_q_;
  std::shared_ptr<dai::DataInputQueue> config_q_, control_q_;
  std::ofstream video_file_;
  std::string rgb_frame_, left_frame_, right_frame_;
  std::string device_name_;
  const std::string rgb_q_name_ = "rgb";
  const std::string depth_q_name_ = "depth";
  const std::string left_q_name_ = "left";
  const std::string right_q_name_ = "right";
  const std::string imu_q_name_ = "imu";
  const std::string video_enc_q_name_ = "h265";
  const std::string control_q_name_ = "control";
  const std::string config_q_name_ = "config";
  const std::string logger_q_name_ = "logger";
  std::atomic<bool> record_, started_recording_, cam_running_;
  const std::vector<std::string> default_label_map_ = {
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",
    "car", "cat", "chair", "cow", "diningtable", "dog", "horse",
    "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"};
  std::unordered_map<dai::CameraBoardSocket, std::string> frame_ids_;
};
}  // namespace depthai_ros_driver

#endif  // DEPTHAI_ROS_DRIVER__BASE_CAMERA_HPP_
