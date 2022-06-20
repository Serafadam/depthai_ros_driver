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
#include <memory>
#include <rclcpp/logger.hpp>
#include <sstream>

#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/rclcpp.hpp"
namespace depthai_ros_driver {
namespace rgb_params {
struct RGBInitConfig {
  double rgb_fps;
  int preview_size;
  int rgb_width;
  int rgb_height;
  std::string rgb_resolution;
  int max_q_size;
  bool set_isp;
  bool set_man_focus;
  int man_focus;
  bool interleaved;
  bool keep_preview_aspect_ratio;
};
struct RGBRuntimeConfig {
  int rgb_exposure;
  int rgb_iso;
  bool set_man_exposure;
};
struct RGBParamNames {
  const std::string rgb_fps = "h_rgb_fps";
  const std::string preview_size = "h_preview_size";
  const std::string rgb_width = "h_rgb_width";
  const std::string rgb_height = "h_rgb_height";
  const std::string rgb_resolution = "h_rgb_resolution";
  const std::string set_isp = "h_set_isp";
  const std::string set_man_focus = "h_set_man_focus";
  const std::string man_focus = "h_man_focus";
  const std::string interleaved = "h_inverleaved";
  const std::string keep_preview_aspect_ratio = "h_keep_preview_aspect_ratio";
  const std::string set_man_exposure = "s_set_man_exposure";
  const std::string rgb_exposure = "s_rgb_exposure";
  const std::string rgb_iso = "s_rgb_iso";
  const std::vector<std::string> name_vector = {
      rgb_fps,          preview_size,
      rgb_width,        rgb_height,
      rgb_resolution,   set_isp,
      set_man_focus,    man_focus,
      interleaved,      keep_preview_aspect_ratio,
      set_man_exposure, rgb_exposure,
      rgb_iso};
};
class RGBParams {
public:
  RGBParams() {}
  void set_init_config(const std::vector<rclcpp::Parameter> &params) {
    for (const auto &p : params) {
      if (p.get_name() == param_names_.rgb_fps) {
        init_config_.rgb_fps = p.get_value<double>();
      } else if (p.get_name() == param_names_.preview_size) {
        init_config_.preview_size = p.get_value<int>();
      } else if (p.get_name() == param_names_.rgb_width) {
        init_config_.rgb_width = p.get_value<int>();
      } else if (p.get_name() == param_names_.rgb_width) {
        init_config_.rgb_width = p.get_value<int>();
      } else if (p.get_name() == param_names_.rgb_height) {
        init_config_.rgb_height = p.get_value<int>();
      } else if (p.get_name() == param_names_.rgb_resolution) {
        init_config_.rgb_resolution = p.get_value<std::string>();
      } else if (p.get_name() == param_names_.set_isp) {
        init_config_.set_isp = p.get_value<bool>();
      } else if (p.get_name() == param_names_.set_man_focus) {
        init_config_.set_man_focus = p.get_value<bool>();
      } else if (p.get_name() == param_names_.interleaved) {
        init_config_.interleaved = p.get_value<bool>();
      } else if (p.get_name() == param_names_.keep_preview_aspect_ratio) {
        init_config_.keep_preview_aspect_ratio = p.get_value<bool>();
      }
    }
  }
  void set_runtime_config(const std::vector<rclcpp::Parameter> &params) {
    for (const auto &p : params) {
      if (p.get_name() == param_names_.set_man_exposure) {
        runtime_config_.set_man_exposure = p.get_value<bool>();
      } else if (p.get_name() == param_names_.rgb_exposure) {
        runtime_config_.rgb_exposure = p.get_value<int>();
      } else if (p.get_name() == param_names_.rgb_iso) {
        runtime_config_.rgb_iso = p.get_value<int>();
      }
    }
  }
  dai::CameraControl get_rgb_control() {
    dai::CameraControl ctrl;
    if (runtime_config_.set_man_exposure) {
      ctrl.setManualExposure(runtime_config_.rgb_exposure,
                             runtime_config_.rgb_iso);
    } else {
      ctrl.setAutoExposureEnable();
    }
    return ctrl;
  }

  virtual void setup_rgb(std::shared_ptr<dai::node::ColorCamera> &camrgb,
                         const rclcpp::Logger &logger) {
    int preview_size = init_config_.preview_size;
    RCLCPP_INFO(logger, "Preview size %d", preview_size);
    camrgb->setPreviewSize(preview_size, preview_size);
    int rgb_width = init_config_.rgb_width;
    int rgb_height = init_config_.rgb_height;
    RCLCPP_INFO(logger, "RGB width: %d, height: %d", rgb_width, rgb_height);
    camrgb->setVideoSize(rgb_width, rgb_height);
    std::string rgb_resolution = init_config_.rgb_resolution;
    RCLCPP_INFO(logger, "RGB resolution %s", rgb_resolution.c_str());
    camrgb->setResolution(rgb_resolution_map_.at(rgb_resolution));
    bool set_interleaved = init_config_.interleaved;
    RCLCPP_INFO(logger, "Interleaved: %d", set_interleaved);
    camrgb->setInterleaved(set_interleaved);
    double rgb_fps = init_config_.rgb_fps;
    RCLCPP_INFO(logger, "RGB FPS: %f", rgb_fps);
    camrgb->setFps(rgb_fps);
    bool set_isp = init_config_.set_isp;
    RCLCPP_INFO(logger, "Set ISP scale: %d", set_isp);
    if (set_isp) {
      camrgb->setIspScale(2, 3);
    }
    bool set_man_focus = init_config_.set_man_focus;
    RCLCPP_INFO(logger, "Enable manual focus: %d", set_man_focus);
    if (set_man_focus) {
      int man_focus = init_config_.man_focus;
      RCLCPP_INFO(logger, "Manual focus set: %d", man_focus);
      camrgb->initialControl.setManualFocus(man_focus);
    }
    bool set_preview_keep_aspect_ratio = init_config_.keep_preview_aspect_ratio;
    RCLCPP_INFO(logger, "Keep preview aspect ratio: %d",
                set_preview_keep_aspect_ratio);
    camrgb->setPreviewKeepAspectRatio(set_preview_keep_aspect_ratio);
  }
  RGBParamNames get_param_names() { return param_names_; }
  RGBInitConfig get_init_config() { return init_config_; }
  RGBRuntimeConfig get_runtime_config() { return runtime_config_; };

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
