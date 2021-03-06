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

#include "depthai_ros_driver/rgbd_camera_obj.hpp"

#include <memory>
#include <string>

#include "rclcpp/logging.hpp"

namespace depthai_ros_driver
{
RGBDCamera::RGBDCamera(const rclcpp::NodeOptions & options)
: BaseCamera("camera", options)
{
  on_configure();
}

void RGBDCamera::on_configure()
{
  declare_rgb_depth_params();
  setup_basic_devices();
  setup_all_xout_streams();
  setup_control_config_xin();
  start_device();
  setup_all_queues();
  RCLCPP_INFO(this->get_logger(), "RGBD Camera ready.");
}

}  // namespace depthai_ros_driver
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ros_driver::RGBDCamera);
