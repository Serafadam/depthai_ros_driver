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

#include "depthai_ros_driver/params_stereo.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "depthai/pipeline/datatype/CameraControl.hpp"

namespace depthai_ros_driver
{
namespace stereo_params
{
StereoParamsHandler::StereoParamsHandler() {}
void StereoParamsHandler::declare_depth_params(rclcpp::Node * node)
{
  init_config_.mono_fps =
    node->declare_parameter<double>(param_names_.mono_fps, init_config_.mono_fps);
  init_config_.mono_resolution = node->declare_parameter<std::string>(
    param_names_.mono_resolution, init_config_.mono_resolution);
  init_config_.lr_check =
    node->declare_parameter<bool>(param_names_.lr_check, init_config_.lr_check);
  init_config_.lrc_threshold =
    node->declare_parameter<uint8_t>(param_names_.lrc_threshold, init_config_.lrc_threshold);
  init_config_.depth_filter_size = node->declare_parameter<uint8_t>(
    param_names_.depth_filter_size, init_config_.depth_filter_size);
  init_config_.stereo_conf_threshold = node->declare_parameter<uint8_t>(
    param_names_.stereo_conf_threshold, init_config_.stereo_conf_threshold);
  init_config_.subpixel =
    node->declare_parameter<bool>(param_names_.subpixel, init_config_.subpixel);
  init_config_.extended_disp =
    node->declare_parameter<bool>(param_names_.extended_disp, init_config_.extended_disp);
  init_config_.rectify_edge_fill_color = node->declare_parameter<uint8_t>(
    param_names_.rectify_edge_fill_color, init_config_.rectify_edge_fill_color);
  init_config_.enable_speckle_filter = node->declare_parameter<bool>(
    param_names_.enable_speckle_filter, init_config_.enable_speckle_filter);
  init_config_.speckle_range =
    node->declare_parameter<uint16_t>(param_names_.speckle_range, init_config_.speckle_range);
  init_config_.enable_temporal_filter = node->declare_parameter<bool>(
    param_names_.enable_temporal_filter, init_config_.enable_temporal_filter);
  init_config_.enable_spatial_filter = node->declare_parameter<bool>(
    param_names_.enable_spatial_filter, init_config_.enable_spatial_filter);
  init_config_.hole_filling_radius = node->declare_parameter<uint8_t>(
    param_names_.hole_filling_radius, init_config_.hole_filling_radius);
  init_config_.spatial_filter_iterations = node->declare_parameter<uint8_t>(
    param_names_.spatial_filter_iterations, init_config_.spatial_filter_iterations);
  init_config_.threshold_filter_min_range = node->declare_parameter<uint16_t>(
    param_names_.threshold_filter_min_range, init_config_.threshold_filter_min_range);
  init_config_.threshold_filter_max_range = node->declare_parameter<uint16_t>(
    param_names_.threshold_filter_max_range, init_config_.threshold_filter_max_range);
  init_config_.decimation_factor = node->declare_parameter<uint8_t>(
    param_names_.decimation_factor, init_config_.decimation_factor);
  init_config_.align_depth =
    node->declare_parameter<bool>(param_names_.align_depth, init_config_.align_depth);
}
void StereoParamsHandler::set_runtime_config(const std::vector<rclcpp::Parameter> & params) {}
dai::CameraControl StereoParamsHandler::get_depth_control()
{
  dai::CameraControl ctrl;
  return ctrl;
}
void StereoParamsHandler::setup_stereo(
  std::shared_ptr<dai::node::StereoDepth> & stereo,
  std::shared_ptr<dai::node::MonoCamera> & mono_left,
  std::shared_ptr<dai::node::MonoCamera> & mono_right, const rclcpp::Logger & logger)
{
  mono_left->setResolution(mono_resolution_map.at(init_config_.mono_resolution));
  mono_right->setResolution(mono_resolution_map.at(init_config_.mono_resolution));
  mono_left->setFps(init_config_.mono_fps);
  mono_right->setFps(init_config_.mono_fps);
  stereo->setLeftRightCheck(init_config_.lr_check);
  if (init_config_.align_depth) {
    stereo->setDepthAlign(dai::CameraBoardSocket::RGB);
  }
  stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
  stereo->initialConfig.setLeftRightCheckThreshold(init_config_.lrc_threshold);
  auto median = static_cast<dai::MedianFilter>(init_config_.depth_filter_size);
  stereo->initialConfig.setMedianFilter(median);
  stereo->initialConfig.setConfidenceThreshold(init_config_.stereo_conf_threshold);
  stereo->initialConfig.setSubpixel(init_config_.subpixel);
  stereo->setExtendedDisparity(init_config_.extended_disp);
  stereo->setRectifyEdgeFillColor(init_config_.rectify_edge_fill_color);
  auto config = stereo->initialConfig.get();
  config.postProcessing.speckleFilter.enable = init_config_.enable_speckle_filter;
  config.postProcessing.speckleFilter.speckleRange = init_config_.speckle_range;
  config.postProcessing.temporalFilter.enable = init_config_.enable_temporal_filter;
  config.postProcessing.spatialFilter.enable = init_config_.enable_spatial_filter;
  config.postProcessing.spatialFilter.holeFillingRadius = init_config_.hole_filling_radius;
  config.postProcessing.spatialFilter.numIterations = init_config_.spatial_filter_iterations;
  config.postProcessing.thresholdFilter.minRange = init_config_.threshold_filter_min_range;
  config.postProcessing.thresholdFilter.maxRange = init_config_.threshold_filter_max_range;
  config.postProcessing.decimationFilter.decimationFactor = init_config_.decimation_factor;
  stereo->initialConfig.set(config);
}
void StereoParamsHandler::set_init_config(const StereoInitConfig & config)
{
  init_config_ = config;
}
void StereoParamsHandler::set_runtime_config(const StereoRuntimeConfig & config)
{
  runtime_config_ = config;
}
StereoParamNames StereoParamsHandler::get_param_names() {return param_names_;}
StereoInitConfig StereoParamsHandler::get_init_config() {return init_config_;}
StereoRuntimeConfig StereoParamsHandler::get_runtime_config() {return runtime_config_;}

}  // namespace stereo_params
}  // namespace depthai_ros_driver
