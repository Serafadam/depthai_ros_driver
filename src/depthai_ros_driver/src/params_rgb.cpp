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

#include "depthai_ros_driver/params_rgb.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace depthai_ros_driver
{
namespace rgb_params
{
RGBParamsHandler::RGBParamsHandler() {}

rcl_interfaces::msg::ParameterDescriptor RGBParamsHandler::get_ranged_int_descriptor(
  int min, int max)
{
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.integer_range.resize(1);
  desc.integer_range.at(0).from_value = min;
  desc.integer_range.at(0).to_value = max;
  return desc;
}
void RGBParamsHandler::declare_rgb_params(rclcpp::Node * node)
{
  init_config_.rgb_fps =
    node->declare_parameter<double>(param_names_.rgb_fps, init_config_.rgb_fps);
  init_config_.rgb_width =
    node->declare_parameter<uint16_t>(param_names_.rgb_width, init_config_.rgb_width);
  init_config_.rgb_height =
    node->declare_parameter<uint16_t>(param_names_.rgb_height, init_config_.rgb_height);
  init_config_.preview_size =
    node->declare_parameter<uint16_t>(param_names_.preview_size, init_config_.preview_size);
  init_config_.rgb_resolution =
    node->declare_parameter<std::string>(param_names_.rgb_resolution, init_config_.rgb_resolution);
  init_config_.set_isp = node->declare_parameter<bool>(param_names_.set_isp, init_config_.set_isp);
  init_config_.interleaved =
    node->declare_parameter<bool>(param_names_.interleaved, init_config_.interleaved);
  init_config_.keep_preview_aspect_ratio = node->declare_parameter<bool>(
    param_names_.keep_preview_aspect_ratio, init_config_.keep_preview_aspect_ratio);
  runtime_config_.man_focus = node->declare_parameter<int>(
    param_names_.man_focus, runtime_config_.man_focus, get_ranged_int_descriptor(0, 255));
  runtime_config_.rgb_exposure = node->declare_parameter<int>(
    param_names_.rgb_exposure, runtime_config_.rgb_exposure, get_ranged_int_descriptor(10, 30000));
  runtime_config_.rgb_iso = node->declare_parameter<int>(
    param_names_.rgb_iso, runtime_config_.rgb_iso, get_ranged_int_descriptor(100, 1600));
  runtime_config_.set_man_focus =
    node->declare_parameter<bool>(param_names_.set_man_focus, runtime_config_.set_man_focus);
  runtime_config_.set_man_exposure =
    node->declare_parameter<bool>(param_names_.set_man_exposure, runtime_config_.set_man_exposure);
  runtime_config_.set_man_whitebalance = node->declare_parameter<bool>(
    param_names_.set_man_whitebalance, runtime_config_.set_man_whitebalance);
  runtime_config_.whitebalance = node->declare_parameter<uint16_t>(
    param_names_.whitebalance, runtime_config_.whitebalance,
    get_ranged_int_descriptor(1000, 12000));
}
void RGBParamsHandler::set_init_config(const std::vector<rclcpp::Parameter> & params)
{
  for (const auto & p : params) {
    if (p.get_name() == param_names_.rgb_fps) {
      init_config_.rgb_fps = p.get_value<double>();
    } else if (p.get_name() == param_names_.preview_size) {
      init_config_.preview_size = p.get_value<uint16_t>();
    } else if (p.get_name() == param_names_.rgb_width) {
      init_config_.rgb_width = p.get_value<uint16_t>();
    } else if (p.get_name() == param_names_.rgb_width) {
      init_config_.rgb_width = p.get_value<uint16_t>();
    } else if (p.get_name() == param_names_.rgb_height) {
      init_config_.rgb_height = p.get_value<uint16_t>();
    } else if (p.get_name() == param_names_.rgb_resolution) {
      init_config_.rgb_resolution = p.get_value<std::string>();
    } else if (p.get_name() == param_names_.set_isp) {
      init_config_.set_isp = p.get_value<bool>();
    } else if (p.get_name() == param_names_.interleaved) {
      init_config_.interleaved = p.get_value<bool>();
    } else if (p.get_name() == param_names_.keep_preview_aspect_ratio) {
      init_config_.keep_preview_aspect_ratio = p.get_value<bool>();
    }
  }
}
void RGBParamsHandler::set_init_config(const RGBInitConfig & config) {init_config_ = config;}
void RGBParamsHandler::set_runtime_config(const RGBRuntimeConfig & config)
{
  runtime_config_ = config;
}
void RGBParamsHandler::set_runtime_config(const std::vector<rclcpp::Parameter> & params)
{
  for (const auto & p : params) {
    if (p.get_name() == param_names_.set_man_exposure) {
      runtime_config_.set_man_exposure = p.get_value<bool>();
    } else if (p.get_name() == param_names_.rgb_exposure) {
      runtime_config_.rgb_exposure = p.get_value<uint16_t>();
    } else if (p.get_name() == param_names_.rgb_iso) {
      runtime_config_.rgb_iso = p.get_value<uint16_t>();
    } else if (p.get_name() == param_names_.set_man_focus) {
      runtime_config_.set_man_focus = p.get_value<bool>();
    } else if (p.get_name() == param_names_.man_focus) {
      runtime_config_.man_focus = p.get_value<uint16_t>();
    } else if (p.get_name() == param_names_.set_man_whitebalance) {
      runtime_config_.set_man_whitebalance = p.get_value<bool>();
    } else if (p.get_name() == param_names_.whitebalance) {
      runtime_config_.whitebalance = p.get_value<uint16_t>();
    }
  }
}
dai::CameraControl RGBParamsHandler::get_rgb_control()
{
  dai::CameraControl ctrl;
  if (runtime_config_.set_man_exposure) {
    ctrl.setManualExposure(runtime_config_.rgb_exposure, runtime_config_.rgb_iso);
  } else {
    ctrl.setAutoExposureEnable();
  }
  if (runtime_config_.set_man_focus) {
    ctrl.setManualFocus(runtime_config_.man_focus);
  } else {
    ctrl.setAutoFocusMode(dai::CameraControl::AutoFocusMode::CONTINUOUS_PICTURE);
  }
  if (runtime_config_.set_man_whitebalance) {
    ctrl.setManualWhiteBalance(runtime_config_.whitebalance);
  } else {
    ctrl.setAutoWhiteBalanceMode(dai::CameraControl::AutoWhiteBalanceMode::AUTO);
  }
  return ctrl;
}
void RGBParamsHandler::setup_rgb(
  std::shared_ptr<dai::node::ColorCamera> & camrgb, const rclcpp::Logger & logger)
{
  RCLCPP_INFO(logger, "Preview size %d", init_config_.preview_size);
  camrgb->setPreviewSize(init_config_.preview_size, init_config_.preview_size);
  RCLCPP_INFO(logger, "RGB width: %d, height: %d", init_config_.rgb_width, init_config_.rgb_height);
  camrgb->setVideoSize(init_config_.rgb_width, init_config_.rgb_height);

  RCLCPP_INFO(logger, "RGB resolution %s", init_config_.rgb_resolution.c_str());
  camrgb->setResolution(rgb_resolution_map_.at(init_config_.rgb_resolution));
  RCLCPP_INFO(logger, "Interleaved: %d", init_config_.interleaved);
  camrgb->setInterleaved(init_config_.interleaved);
  RCLCPP_INFO(logger, "RGB FPS: %f", init_config_.rgb_fps);
  camrgb->setFps(init_config_.rgb_fps);
  RCLCPP_INFO(logger, "Set ISP scale: %d", init_config_.set_isp);
  if (init_config_.set_isp) {
    camrgb->setIspScale(2, 3);
  }
  RCLCPP_INFO(logger, "Enable manual focus: %d", runtime_config_.set_man_focus);
  if (runtime_config_.set_man_focus) {
    RCLCPP_INFO(logger, "Manual focus set: %d", runtime_config_.man_focus);
    camrgb->initialControl.setManualFocus(runtime_config_.man_focus);
  }
  RCLCPP_INFO(logger, "Keep preview aspect ratio: %d", init_config_.keep_preview_aspect_ratio);
  camrgb->setPreviewKeepAspectRatio(init_config_.keep_preview_aspect_ratio);
}
RGBParamNames RGBParamsHandler::get_param_names() {return param_names_;}
RGBInitConfig RGBParamsHandler::get_init_config() {return init_config_;}
RGBRuntimeConfig RGBParamsHandler::get_runtime_config() {return runtime_config_;}
}  // namespace rgb_params
}  // namespace depthai_ros_driver
