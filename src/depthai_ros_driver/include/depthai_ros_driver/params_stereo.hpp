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
#include <cmath>
#include <cstdint>
#include <memory>

#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/rclcpp.hpp"
namespace depthai_ros_driver {
namespace stereo_params {
struct StereoInitConfig {
  double mono_fps = 60.0;
  std::string mono_resolution = "400";
  bool align_depth = true;
  bool lr_check = true;
  uint8_t lrc_threshold = 5;
  uint8_t depth_filter_size = 7;
  uint8_t stereo_conf_threshold = 255;
  bool subpixel = true;
  bool extended_disp = false;
  int8_t rectify_edge_fill_color = -1;
  bool enable_speckle_filter = false;
  uint16_t speckle_range = 50;
  bool enable_temporal_filter = true;
  bool enable_spatial_filter = true;
  uint8_t hole_filling_radius = 2;
  uint8_t spatial_filter_iterations = 1;
  uint16_t threshold_filter_min_range = 400;
  uint16_t threshold_filter_max_range = 15000;
  uint8_t decimation_factor = 1;
};

struct StereoRuntimeConfig {
  int mono_exposure = 1000;
  int mono_iso = 100;
  bool set_man_exposure = false;
};
struct StereoParamNames {
  const std::string mono_fps = "i_mono_fps";
  const std::string mono_resolution = "i_mono_resolution";
  const std::string align_depth = "i_align_depth";
  const std::string lr_check = "i_lr_check";
  const std::string lrc_threshold = "i_lrc_threshold";
  const std::string depth_filter_size = "i_depth_filter_size";
  const std::string stereo_conf_threshold = "i_stereo_depth_threshold";
  const std::string subpixel = "i_subpixel";
  const std::string extended_disp = "i_extended_disp";
  const std::string rectify_edge_fill_color = "i_rectify_edge_fill_color";
  const std::string enable_speckle_filter = "i_enable_speckle_filter";
  const std::string speckle_range = "i_speckle_range";
  const std::string enable_temporal_filter = "i_enable_temporal_filter";
  const std::string enable_spatial_filter = "i_enable_spatial_filter";
  const std::string hole_filling_radius = "i_hole_filling_radius";
  const std::string spatial_filter_iterations = "i_spatial_filter_iterations";
  const std::string threshold_filter_min_range = "i_threshold_filter_min_range";
  const std::string threshold_filter_max_range = "i_threshold_filter_max_range";
  const std::string decimation_factor = "i_decimation_factor";
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

class StereoParamsHandler {
public:
  StereoParamsHandler();
  void declare_depth_params(rclcpp::Node *node);
  void set_runtime_config(const std::vector<rclcpp::Parameter> &params);
  dai::CameraControl get_depth_control();
  virtual void setup_stereo(std::shared_ptr<dai::node::StereoDepth> &stereo,
                            std::shared_ptr<dai::node::MonoCamera> &mono_left,
                            std::shared_ptr<dai::node::MonoCamera> &mono_right,
                            const rclcpp::Logger &logger);
    void set_init_config(const StereoInitConfig &config);
    void set_runtime_config(const StereoRuntimeConfig &config);
    StereoParamNames get_param_names();
  StereoInitConfig get_init_config();
  StereoRuntimeConfig get_runtime_config();

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
