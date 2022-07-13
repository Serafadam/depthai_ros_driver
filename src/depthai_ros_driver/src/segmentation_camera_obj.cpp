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

#include "depthai/pipeline/node/XLinkOut.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/Point2f.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace depthai_ros_driver {
SegmentationCamera::SegmentationCamera(const rclcpp::NodeOptions &options)
    : BaseCamera("camera", options) {
      on_configure();
    }

void SegmentationCamera::on_configure() {
  declare_rgb_depth_params();
  std::string default_nn_path =
      ament_index_cpp::get_package_share_directory("depthai_ros_driver") +
      "/models/deeplab_v3_plus_mnv2_decoder_256_openvino_2021.4.blob";
  nn_path_ = this->declare_parameter<std::string>("nn_path", default_nn_path);
  default_label_map_ = get_default_label_map();
  label_map_ = default_label_map_;
  std::for_each(
      label_map_.begin(), label_map_.end(), [this](const std::string &l) {
        auto it =
            std::find(default_label_map_.begin(), default_label_map_.end(), l);
        if (it != default_label_map_.end()) {
          label_map_indexes_.emplace_back(it - default_label_map_.begin());
        }
      });

  setup_publishers();
  setup_pipeline();

  RCLCPP_INFO(this->get_logger(), "SegmentationCamera ready!");
}

void SegmentationCamera::setup_pipeline() {
  setup_basic_devices();
  setup_all_xout_streams();
  setup_control_config_xin();
  auto pipeline = get_pipeline();

  nn_ = pipeline->create<dai::node::NeuralNetwork>();
  xout_nn_ = pipeline->create<dai::node::XLinkOut>();
  xout_nn_->setStreamName("nn");

  nn_->setNumPoolFrames(4);
  nn_->setBlobPath(nn_path_);
  nn_->setNumInferenceThreads(2);
  nn_->input.setBlocking(false);
  link_nn(nn_);

  nn_->out.link(xout_nn_->input);
  start_device();
    cropped_info_ = get_calibration(get_frame_id(dai::CameraBoardSocket::RGB),
                      dai::CameraBoardSocket::RGB, 256, 256);
  setup_all_queues();
  segmentation_nn_q_ = get_output_q("nn");
  segmentation_nn_q_->addCallback(std::bind(&SegmentationCamera::seg_cb, this,
                                            std::placeholders::_1,
                                            std::placeholders::_2));
}
void SegmentationCamera::setup_publishers() {
  mask_pub_ = image_transport::create_camera_publisher(this, "~/mask");
}
void SegmentationCamera::seg_cb(const std::string &name,
                                const std::shared_ptr<dai::ADatatype> &data) {
  auto in_det = std::dynamic_pointer_cast<dai::NNData>(data);
  std::vector<std::int32_t> nn_frame = in_det->getFirstLayerInt32();
  filter_out_detections(nn_frame);
  cv::Mat nn_mat = cv::Mat(nn_frame);
  nn_mat = nn_mat.reshape(0, 256);
  cv::Mat seg_colored = decode_deeplab(nn_mat);
  publish_img(seg_colored, sensor_msgs::image_encodings::BGR8, cropped_info_,
              mask_pub_, this->get_clock()->now());
}

void SegmentationCamera::filter_out_detections(std::vector<int> &det) {
  std::for_each(det.begin(), det.end(), [this](int &x) {
    auto it =
        std::find(label_map_indexes_.begin(), label_map_indexes_.end(), x);
    if (it == label_map_indexes_.end()) {
      x = 0;
    }
  });
}

cv::Mat SegmentationCamera::decode_deeplab(cv::Mat mat) {
  cv::Mat out = mat.mul(255 / classes_num_);
  out.convertTo(out, CV_8UC1);
  cv::Mat colors = cv::Mat(256, 1, CV_8UC3);
  cv::applyColorMap(out, colors, cv::COLORMAP_JET);
  for (int row = 0; row < out.rows; ++row) {
    uchar *p = out.ptr(row);
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

} // namespace depthai_ros_driver
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ros_driver::SegmentationCamera);
