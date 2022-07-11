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
#ifndef DEPTHAI_ROS_DRIVER__PARAMS_RGB_HPP
#define DEPTHAI_ROS_DRIVER__PARAMS_RGB_HPP
#include <cstdint>
#include <memory>
#include <sstream>

#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/rclcpp.hpp"
namespace depthai_ros_driver {
namespace rgb_params {
struct RGBInitConfig {
  double rgb_fps = 30.0;
  uint16_t preview_size = 256;
  uint16_t rgb_width = 1920;
  uint16_t rgb_height = 1080;
  std::string rgb_resolution = "1080";
  uint16_t max_q_size = 4;
  bool set_isp = false;
  bool interleaved = false;
  bool keep_preview_aspect_ratio = true;
};
struct RGBRuntimeConfig {
  uint16_t rgb_exposure = 1000;
  uint16_t rgb_iso = 100;
  uint16_t man_focus = 1;
  uint16_t whitebalance = 3300;
  bool set_man_focus = true;
  bool set_man_exposure = false;
  bool set_man_whitebalance = false;
};
struct RGBParamNames {
  const std::string rgb_fps = "i_rgb_fps";
  const std::string preview_size = "i_preview_size";
  const std::string rgb_width = "i_rgb_width";
  const std::string rgb_height = "i_rgb_height";
  const std::string rgb_resolution = "i_rgb_resolution";
  const std::string set_isp = "i_set_isp";
  const std::string interleaved = "i_inverleaved";
  const std::string keep_preview_aspect_ratio = "i_keep_preview_aspect_ratio";
  const std::string set_man_focus = "r_set_man_focus";
  const std::string man_focus = "r_man_focus";
  const std::string set_man_exposure = "r_set_man_exposure";
  const std::string rgb_exposure = "r_rgb_exposure";
  const std::string rgb_iso = "r_rgb_iso";
  const std::string set_man_whitebalance = "r_set_man_whitebalance";
  const std::string whitebalance = "r_whitebalance";
  const std::vector<std::string> name_vector = {
      rgb_fps,          preview_size,
      rgb_width,        rgb_height,
      rgb_resolution,   set_isp,
      set_man_focus,    man_focus,
      interleaved,      keep_preview_aspect_ratio,
      set_man_exposure, rgb_exposure,
      rgb_iso,          set_man_whitebalance,
      whitebalance};
};

class RGBParamsHandler {
public:
  RGBParamsHandler();
  rcl_interfaces::msg::ParameterDescriptor get_ranged_int_descriptor(int min,
                                                                     int max);
  void declare_rgb_params(rclcpp::Node *node);
  dai::CameraControl get_rgb_control();
  virtual void setup_rgb(std::shared_ptr<dai::node::ColorCamera> &camrgb,
                         const rclcpp::Logger &logger);
  RGBParamNames get_param_names();
  RGBInitConfig get_init_config();
  RGBRuntimeConfig get_runtime_config();
  void set_init_config(const std::vector<rclcpp::Parameter> &params);
    void set_init_config(const RGBInitConfig &config);
    void set_runtime_config(const RGBRuntimeConfig &config);
  void set_runtime_config(const std::vector<rclcpp::Parameter> &params);
private:
  std::unordered_map<std::string, dai::ColorCameraProperties::SensorResolution>
      rgb_resolution_map_ = {
          {"1080", dai::ColorCameraProperties::SensorResolution::THE_1080_P},
          {"4k", dai::ColorCameraProperties::SensorResolution::THE_4_K},
          {"12MP", dai::ColorCameraProperties::SensorResolution::THE_12_MP},
  };
  RGBInitConfig init_config_;
  RGBRuntimeConfig runtime_config_;
  RGBParamNames param_names_;
};
} // namespace rgb_params
} // namespace depthai_ros_driver

#endif // DEPTHAI_ROS_DRIVER__PARAMS_RGB_HPP
