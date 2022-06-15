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
#ifndef DEPTHAI_ROS_DRIVER__PARAMS_STEREO_HPP
#define DEPTHAI_ROS_DRIVER__PARAMS_STEREO_HPP
#include <cstdint>
#include <memory>

#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/rclcpp.hpp"
namespace depthai_ros_driver {
namespace stereo_params {
struct StereoInitConfig {
  double mono_fps;
  std::string mono_resolution;
  bool align_depth;
  bool lr_check;
  int lrc_threshold;
  int depth_filter_size;
  int stereo_conf_threshold;
  bool subpixel;
  bool extended_disp;
  int rectify_edge_fill_color;
  bool enable_speckle_filter;
  uint32_t speckle_range;
  bool enable_temporal_filter;
  bool enable_spatial_filter;
  uint8_t hole_filling_radius;
  int spatial_filter_iterations;
  uint32_t threshold_filter_min_range;
  uint32_t threshold_filter_max_range;
  uint32_t decimation_factor;
};

struct StereoRuntimeConfig {
  int mono_exposure;
  int mono_iso;
  bool set_man_exposure;
};
struct StereoParamNames {
  const std::string mono_fps = "h_mono_fps";
  const std::string mono_resolution = "h_mono_resolution";
  const std::string align_depth = "h_align_depth";
  const std::string lr_check = "h_lr_check";
  const std::string lrc_threshold = "h_lrc_threshold";
  const std::string depth_filter_size = "h_depth_filter_size";
  const std::string stereo_conf_threshold = "h_stereo_depth_threshold";
  const std::string subpixel = "h_subpixel";
  const std::string extended_disp = "h_extended_disp";
  const std::string rectify_edge_fill_color = "h_rectify_edge_fill_color";
  const std::string enable_speckle_filter = "h_enable_speckle_filter";
  const std::string speckle_range = "h_speckle_range";
  const std::string enable_temporal_filter = "h_enable_temporal_filter";
  const std::string enable_spatial_filter = "h_enable_spatial_filter";
  const std::string hole_filling_radius = "h_hole_filling_radius";
  const std::string spatial_filter_iterations = "h_spatial_filter_iterations";
  const std::string threshold_filter_min_range = "h_threshold_filter_min_range";
  const std::string threshold_filter_max_range = "h_threshold_filter_max_range";
  const std::string decimation_factor = "h_decimation_factor";
  const std::vector<std::string> name_vector = {mono_fps,
                                                mono_resolution,
                                                lr_check,
                                                lrc_threshold,
                                                depth_filter_size,
                                                stereo_conf_threshold,
                                                subpixel,
                                                extended_disp,
                                                rectify_edge_fill_color,
                                                enable_speckle_filter,
                                                speckle_range,
                                                enable_temporal_filter,
                                                enable_spatial_filter,
                                                hole_filling_radius,
                                                spatial_filter_iterations,
                                                threshold_filter_min_range,
                                                threshold_filter_max_range,
                                                decimation_factor};
};

class StereoParams {
public:
  StereoParams() {}
  void set_init_config(const std::vector<rclcpp::Parameter> &params) {
    for (const auto &p : params) {
      if (p.get_name() == param_names_.mono_fps) {
        init_config_.mono_fps = p.get_value<double>();
      } else if (p.get_name() == param_names_.mono_resolution) {
        init_config_.mono_resolution = p.get_value<std::string>();
      } else if (p.get_name() == param_names_.align_depth) {
        init_config_.align_depth = p.get_value<bool>();
      } else if (p.get_name() == param_names_.lr_check) {
        init_config_.lr_check = p.get_value<bool>();
      } else if (p.get_name() == param_names_.lrc_threshold) {
        init_config_.lrc_threshold = p.get_value<int>();
      } else if (p.get_name() == param_names_.depth_filter_size) {
        init_config_.depth_filter_size = p.get_value<int>();
      } else if (p.get_name() == param_names_.stereo_conf_threshold) {
        init_config_.stereo_conf_threshold = p.get_value<int>();
      } else if (p.get_name() == param_names_.subpixel) {
        init_config_.subpixel = p.get_value<bool>();
      } else if (p.get_name() == param_names_.extended_disp) {
        init_config_.extended_disp = p.get_value<bool>();
      } else if (p.get_name() == param_names_.rectify_edge_fill_color) {
        init_config_.rectify_edge_fill_color = p.get_value<int>();
      } else if (p.get_name() == param_names_.enable_speckle_filter) {
        init_config_.enable_speckle_filter = p.get_value<bool>();
      } else if (p.get_name() == param_names_.speckle_range) {
        init_config_.speckle_range = p.get_value<uint32_t>();
      } else if (p.get_name() == param_names_.enable_temporal_filter) {
        init_config_.enable_temporal_filter = p.get_value<bool>();
      } else if (p.get_name() == param_names_.enable_spatial_filter) {
        init_config_.enable_spatial_filter = p.get_value<bool>();
      } else if (p.get_name() == param_names_.hole_filling_radius) {
        init_config_.hole_filling_radius = p.get_value<uint8_t>();
      } else if (p.get_name() == param_names_.spatial_filter_iterations) {
        init_config_.spatial_filter_iterations = p.get_value<int>();
      } else if (p.get_name() == param_names_.threshold_filter_min_range) {
        init_config_.threshold_filter_min_range = p.get_value<uint32_t>();
      } else if (p.get_name() == param_names_.threshold_filter_max_range) {
        init_config_.threshold_filter_max_range = p.get_value<uint32_t>();
      } else if (p.get_name() == param_names_.decimation_factor) {
        init_config_.decimation_factor = p.get_value<uint32_t>();
      }
    }
  }
  void set_runtime_config(const std::vector<rclcpp::Parameter> &params) {}
  dai::CameraControl get_depth_control() {
    dai::CameraControl ctrl;
    return ctrl;
  }

  virtual void setup_stereo(std::shared_ptr<dai::node::StereoDepth> &stereo,
                            std::shared_ptr<dai::node::MonoCamera> &mono_left,
                            std::shared_ptr<dai::node::MonoCamera> &mono_right,
                            const rclcpp::Logger &logger) {
    mono_left->setResolution(
        mono_resolution_map.at(init_config_.mono_resolution));
    mono_right->setResolution(
        mono_resolution_map.at(init_config_.mono_resolution));
    mono_left->setFps(init_config_.mono_fps);
    mono_right->setFps(init_config_.mono_fps);
    stereo->setLeftRightCheck(init_config_.lr_check);
    if (init_config_.align_depth) {
      stereo->setDepthAlign(dai::CameraBoardSocket::RGB);
    }
    stereo->setDefaultProfilePreset(
        dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo->initialConfig.setLeftRightCheckThreshold(
        init_config_.lrc_threshold);
    auto median =
        static_cast<dai::MedianFilter>(init_config_.depth_filter_size);
    stereo->initialConfig.setMedianFilter(median);
    stereo->initialConfig.setConfidenceThreshold(
        init_config_.stereo_conf_threshold);
    stereo->initialConfig.setSubpixel(init_config_.subpixel);
    stereo->setExtendedDisparity(init_config_.extended_disp);
    stereo->setRectifyEdgeFillColor(init_config_.rectify_edge_fill_color);
    auto config = stereo->initialConfig.get();
    config.postProcessing.speckleFilter.enable =
        init_config_.enable_speckle_filter;
    config.postProcessing.speckleFilter.speckleRange =
        init_config_.speckle_range;
    config.postProcessing.temporalFilter.enable =
        init_config_.enable_temporal_filter;
    config.postProcessing.spatialFilter.enable =
        init_config_.enable_spatial_filter;
    config.postProcessing.spatialFilter.holeFillingRadius =
        init_config_.hole_filling_radius;
    config.postProcessing.spatialFilter.numIterations =
        init_config_.spatial_filter_iterations;
    config.postProcessing.thresholdFilter.minRange =
        init_config_.threshold_filter_min_range;
    config.postProcessing.thresholdFilter.maxRange =
        init_config_.threshold_filter_max_range;
    config.postProcessing.decimationFilter.decimationFactor =
        init_config_.decimation_factor;
    stereo->initialConfig.set(config);
  }
  StereoParamNames get_param_names() { return param_names_; }
  StereoInitConfig get_init_config() { return init_config_; }
  StereoRuntimeConfig get_runtime_config() { return runtime_config_; }

private:
  std::unordered_map<std::string, dai::MonoCameraProperties::SensorResolution>
      mono_resolution_map = {
          {"400", dai::MonoCameraProperties::SensorResolution::THE_400_P},
          {"480", dai::MonoCameraProperties::SensorResolution::THE_480_P},
          {"720", dai::MonoCameraProperties::SensorResolution::THE_720_P},
          {"800", dai::MonoCameraProperties::SensorResolution::THE_800_P},
  };

  StereoInitConfig init_config_;
  StereoRuntimeConfig runtime_config_;
  StereoParamNames param_names_;
};
} // namespace stereo_params
} // namespace depthai_ros_driver

#endif // DEPTHAI_ROS_DRIVER__PARAMS_STEREO_HPP
