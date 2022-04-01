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

#include "depthai_ros_driver/utils.hpp"

namespace depthai_ros_driver
{
namespace utils
{
sensor_msgs::msg::Image convert_img_to_ros(
  const cv::Mat & frame, const char * encoding, rclcpp::Time stamp)
{
  cv_bridge::CvImage img_bridge;
  sensor_msgs::msg::Image img_msg;
  std_msgs::msg::Header header;
  header.frame_id = "camera_link";
  img_bridge = cv_bridge::CvImage(header, encoding, frame);
  img_bridge.toImageMsg(img_msg);
  img_msg.header.stamp = stamp;
  return img_msg;
}
}  // namespace utils
}  // namespace depthai_ros_driver
