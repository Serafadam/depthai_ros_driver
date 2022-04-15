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

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace depthai_ros_driver
{
SegmentationCamera::SegmentationCamera(const rclcpp::NodeOptions & options)
: BaseCamera("camera", options)
{
}

void SegmentationCamera::on_configure()
{
  declare_basic_params();
  setup_pipeline();
  setup_publishers();

  image_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / fps_)),
    std::bind(&SegmentationCamera::timer_cb, this));
  RCLCPP_INFO(this->get_logger(), "SegmentationCamera ready!");
}

void SegmentationCamera::setup_pipeline()
{
  pipeline_ = std::make_unique<dai::Pipeline>();
  setup_rgb();
  setup_stereo();
  nn_ = pipeline_->create<dai::node::NeuralNetwork>();

  nn_->setNumPoolFrames(4);
  nn_->setBlobPath(nn_path_);
  nn_->setNumInferenceThreads(2);
  nn_->input.setBlocking(false);

  xout_rgb_ = pipeline_->create<dai::node::XLinkOut>();
  xout_nn_ = pipeline_->create<dai::node::XLinkOut>();
  xout_depth_ = pipeline_->create<dai::node::XLinkOut>();
  xout_video_ = pipeline_->create<dai::node::XLinkOut>();
  camrgb_->setVideoSize(rgb_width_, rgb_height_);

  xout_rgb_->setStreamName("rgb");
  xout_nn_->setStreamName("nn");
  xout_depth_->setStreamName("depth");
  xout_video_->setStreamName("video");

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
  image_pub_ = image_transport::create_publisher(this, "~/image_rect");
  mask_pub_ = image_transport::create_publisher(this, "~/mask");
  cropped_depth_pub_ = image_transport::create_camera_publisher(this, "~/depth/image_rect_raw");
  cropped_depth_info_ = get_calibration(dai::CameraBoardSocket::RGB, 400, 400);
}

void SegmentationCamera::timer_cb()
{
  auto in_det = segmentation_nn_q_->get<dai::NNData>();
  auto depth = depth_q_->get<dai::ImgFrame>();
  auto video_in = video_q_->get<dai::ImgFrame>();
  auto preview = preview_q_->get<dai::ImgFrame>();

  cv::Mat depth_frame = depth->getFrame();
  cv::Mat video_frame = video_in->getCvFrame();
  cv::Mat preview_frame = preview->getCvFrame();
  std::vector<int> nn_frame = in_det->getFirstLayerInt32();
  filter_out_detections(nn_frame);

  cv::Mat nn_mat = cv::Mat(nn_frame);
  nn_mat = nn_mat.reshape(0, 256);

  cv::Mat seg_colored = decode_deeplab(nn_mat);

  cv::Mat overlaid_preview, mask, depth_frame_masked, depth_frame_colored;
  cv::addWeighted(preview_frame, 1.0, seg_colored, 0.4, 0.0, overlaid_preview);

  resize_and_get_mask(seg_colored, depth_frame, mask);
  colorize_and_mask_depthamap(depth_frame, depth_frame_colored, mask, depth_frame_masked);

  depth_pub_.publish(
    convert_img_to_ros(
      depth_frame_colored, sensor_msgs::image_encodings::BGR8, this->get_clock()->now()));
  auto stamp = this->get_clock()->now();
  cv::Mat depth_frame_masked_gray;
  cv::cvtColor(depth_frame_masked, depth_frame_masked_gray, CV_BGR2GRAY);
  depth_frame_masked_gray.convertTo(depth_frame_masked_gray, CV_32FC1);
  cropped_depth_info_.header.stamp = stamp;
  cropped_depth_info_.header.frame_id = "camera_link";
  cropped_depth_pub_.publish(
    convert_img_to_ros(
      depth_frame_masked_gray, sensor_msgs::image_encodings::TYPE_32FC1, stamp),
    cropped_depth_info_);

  image_pub_.publish(
    convert_img_to_ros(
      overlaid_preview, sensor_msgs::image_encodings::BGR8, this->get_clock()->now()));

  mask_pub_.publish(
    convert_img_to_ros(
      mask, sensor_msgs::image_encodings::BGR8, this->get_clock()->now()));

}

void SegmentationCamera::filter_out_detections(std::vector<int> & det)
{
  std::for_each(
    det.begin(), det.end(), [this](int & x) {
      auto it = std::find(
        label_map_indexes_.begin(),
        label_map_indexes_.end(), x);
      if (it == label_map_indexes_.end()) {
        x = 0;
      }
    });
}

void SegmentationCamera::colorize_and_mask_depthamap(
  cv::Mat & depth_src, cv::Mat & depth_colored,
  cv::Mat & mask, cv::Mat & depth_frame_masked)
{
  auto disp_mult = 255 / stereo_->initialConfig.getMaxDisparity();
  depth_src = depth_src.mul(disp_mult);
  cv::normalize(depth_src, depth_colored, 255, 0, cv::NORM_INF, CV_8UC1);
  cv::equalizeHist(depth_colored, depth_colored);
  cv::applyColorMap(depth_colored, depth_colored, cv::COLORMAP_JET);
  depth_colored.copyTo(depth_frame_masked, mask);
}

void SegmentationCamera::resize_and_get_mask(
  cv::Mat & seg_colored_src, cv::Mat & depth_frame_src, cv::Mat & mask)
{
  square_crop(depth_frame_src);
  cv::resize(depth_frame_src, depth_frame_src, cv::Size(400, 400));
  cv::resize(
    seg_colored_src, seg_colored_src,
    cv::Size(depth_frame_src.cols, depth_frame_src.rows));
  cv::threshold(seg_colored_src, mask, 21, 255, cv::THRESH_BINARY);
}

void SegmentationCamera::square_crop(cv::Mat & frame)
{
  int d = frame.rows - frame.cols / 2;
  frame = frame(cv::Range(0, frame.rows), cv::Range(d, frame.cols - d));
}

cv::Mat SegmentationCamera::decode_deeplab(cv::Mat mat)
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
