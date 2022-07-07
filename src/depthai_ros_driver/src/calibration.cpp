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

#include "depthai_ros_driver/calibration.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace depthai_ros_driver {
sensor_msgs::msg::CameraInfo
get_calibration(std::unique_ptr<dai::Device> &device,
                const std::string &frame_id, dai::CameraBoardSocket socket,
                int width, int height, dai::Point2f top_left_pixel_id,
                dai::Point2f bottom_right_pixel_id) {
  dai::CalibrationHandler cal_data = device->readCalibration();
  std::vector<std::vector<float>> intrinsics;
  sensor_msgs::msg::CameraInfo info;
  if (width == 0 || height == 0) {
    std::tie(intrinsics, width, height) = cal_data.getDefaultIntrinsics(socket);
  } else {
    intrinsics = cal_data.getCameraIntrinsics(
        socket, width, height, top_left_pixel_id, bottom_right_pixel_id);
  }
  info.height = height;
  info.width = width;
  std::copy(intrinsics[0].begin(), intrinsics[0].end(), info.k.begin());
  std::copy(intrinsics[1].begin(), intrinsics[1].end(), info.k.begin() + 3);

  auto dist = cal_data.getDistortionCoefficients(socket);

  for (const float d : dist) {
    info.d.push_back(static_cast<double>(d));
  }

  double tx = 0.0;
  double ty = 0.0;
  info.r[0] = info.r[4] = info.r[8] = 1;
  std::vector<std::vector<float>> rotation;
  std::copy(intrinsics[0].begin(), intrinsics[0].end(), info.p.begin());
  std::copy(intrinsics[1].begin(), intrinsics[1].end(), info.p.begin() + 4);
  if (socket == dai::CameraBoardSocket::LEFT) {
    rotation = cal_data.getStereoLeftRectificationRotation();
  } else if (socket == dai::CameraBoardSocket::RIGHT) {
    rotation = cal_data.getStereoRightRectificationRotation();
    std::vector<std::vector<float>> extrinsics = cal_data.getCameraExtrinsics(
        dai::CameraBoardSocket::RIGHT, dai::CameraBoardSocket::LEFT);
    tx = extrinsics[0][3] / 100.0;
  }

  for (size_t i = 0; i < rotation.size(); i++) {
    for (size_t j = 0; j < rotation[i].size(); j++) {
      info.r[i + j] = rotation[i][j];
    }
  }
  if (socket != dai::CameraBoardSocket::RGB) {
    std::copy(rotation[0].begin(), rotation[0].end(), info.r.begin());
    std::copy(rotation[1].begin(), rotation[1].end(), info.r.begin() + 3);
    std::copy(rotation[2].begin(), rotation[2].end(), info.r.begin() + 6);
  }
  info.p[3] = tx;
  info.p[7] = ty;
  info.p[11] = 0.0;
  info.distortion_model = "rational_polynomial";
  info.header.frame_id = frame_id;
  return info;
}
} // namespace depthai_ros_driver
