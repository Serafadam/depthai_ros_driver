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

#include <depthai-shared/common/CameraBoardSocket.hpp>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai_ros_driver/params_rgb.hpp"

namespace depthai_ros_driver
{
MobilenetCamera::MobilenetCamera(const rclcpp::NodeOptions & options)
: BaseCamera("camera", options)
{
  on_configure();
}

void MobilenetCamera::on_configure()
{
  declare_rgb_depth_params();
  std::string default_nn_path = ament_index_cpp::get_package_share_directory("depthai_ros_driver") +
                                "/models/mobilenet-ssd_openvino_2021.2_6shave.blob";
  nn_path_ = this->declare_parameter<std::string>("nn_path", default_nn_path);
  setup_publishers();
  setup_pipeline();

  RCLCPP_INFO(this->get_logger(), "MobilenetCamera ready!");
}

void MobilenetCamera::setup_pipeline()
{
  label_map_ = get_default_label_map();
  rgb_params::RGBInitConfig config;
  config.preview_size = 300;
  override_init_rgb_config(config);
  setup_basic_devices();
  setup_all_xout_streams();
  setup_control_config_xin();
  auto pipeline = get_pipeline();
  nn_ = pipeline->create<dai::node::MobileNetSpatialDetectionNetwork>();
  nn_->setConfidenceThreshold(0.5);
  nn_->setBlobPath(nn_path_);
  nn_->setNumInferenceThreads(2);
  nn_->input.setBlocking(false);
  nn_->setDepthLowerThreshold(100);
  nn_->setDepthUpperThreshold(5000);
  nn_->setBoundingBoxScaleFactor(0.3);

  xout_nn_ = pipeline->create<dai::node::XLinkOut>();

  xout_nn_->setStreamName("nn");
  link_spatial_detection(nn_);
  nn_->out.link(xout_nn_->input);

  start_device();
  setup_all_queues();
  detection_nn_q_ = get_output_q("nn", 4, false);
  detection_nn_q_->addCallback(
    std::bind(&MobilenetCamera::det_cb, this, std::placeholders::_1, std::placeholders::_2));
}
void MobilenetCamera::setup_publishers()
{
  det_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("~/detections", 10);
}

void MobilenetCamera::det_cb(const std::string & name, const std::shared_ptr<dai::ADatatype> & data)
{
  auto in_det = std::dynamic_pointer_cast<dai::SpatialImgDetections>(data);
  auto detections = in_det->detections;
  vision_msgs::msg::Detection3DArray ros_det;
  ros_det.header.frame_id = get_frame_id(dai::CameraBoardSocket::RGB);
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
      detections[i].spatialCoordinates.x / 1000.0;
    ros_det.detections[i].results[0].pose.pose.position.y =
      -detections[i].spatialCoordinates.x / 1000.0;
    ros_det.detections[i].results[0].pose.pose.position.z =
      -detections[i].spatialCoordinates.z / 1000.0;
  }

  det_pub_->publish(ros_det);
}

}  // namespace depthai_ros_driver
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ros_driver::MobilenetCamera);
