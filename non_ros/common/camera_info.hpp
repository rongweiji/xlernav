#pragma once

#include <sensor_msgs/msg/camera_info.hpp>

#include <string>

namespace xlernav {

bool LoadCameraInfoYaml(const std::string & path, sensor_msgs::msg::CameraInfo & info);

}  // namespace xlernav
