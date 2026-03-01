#include "camera_info.hpp"

#include <cctype>
#include <fstream>
#include <sstream>
#include <vector>

namespace {

std::string Trim(const std::string & input)
{
  const auto start = input.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return "";
  }
  const auto end = input.find_last_not_of(" \t\r\n");
  return input.substr(start, end - start + 1);
}

bool StartsWith(const std::string & text, const std::string & prefix)
{
  return text.size() >= prefix.size() && text.compare(0, prefix.size(), prefix) == 0;
}

}  // namespace

namespace xlernav {

bool LoadCameraInfoYaml(const std::string & path, sensor_msgs::msg::CameraInfo & info)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    return false;
  }

  std::string line;
  int width = -1;
  int height = -1;
  std::vector<double> data;
  bool in_camera_matrix = false;
  bool in_data = false;

  while (std::getline(file, line)) {
    const std::string trimmed = Trim(line);
    if (trimmed.empty()) {
      continue;
    }

    if (StartsWith(trimmed, "image_width:")) {
      const auto value = Trim(trimmed.substr(std::string("image_width:").size()));
      try {
        width = std::stoi(value);
      } catch (...) {
      }
      continue;
    }
    if (StartsWith(trimmed, "image_height:")) {
      const auto value = Trim(trimmed.substr(std::string("image_height:").size()));
      try {
        height = std::stoi(value);
      } catch (...) {
      }
      continue;
    }

    if (trimmed == "camera_matrix:") {
      in_camera_matrix = true;
      in_data = false;
      continue;
    }

    if (in_camera_matrix && StartsWith(trimmed, "data:")) {
      in_data = true;
      continue;
    }

    if (in_data) {
      if (trimmed[0] == '-') {
        const auto value = Trim(trimmed.substr(1));
        try {
          data.push_back(std::stod(value));
        } catch (...) {
        }
      } else if (std::isdigit(trimmed[0]) || trimmed[0] == '+' || trimmed[0] == '.') {
        try {
          data.push_back(std::stod(trimmed));
        } catch (...) {
        }
      } else {
        in_data = false;
      }
    }
  }

  if (data.size() < 9) {
    return false;
  }

  info.k = {data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]};
  info.width = width > 0 ? static_cast<uint32_t>(width) : info.width;
  info.height = height > 0 ? static_cast<uint32_t>(height) : info.height;
  return true;
}

}  // namespace xlernav
