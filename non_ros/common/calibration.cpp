#include "calibration.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>

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

bool ParseInt(const std::string & value, int & out)
{
  try {
    out = std::stoi(value);
    return true;
  } catch (...) {
    return false;
  }
}

bool ParseDouble(const std::string & value, double & out)
{
  try {
    out = std::stod(value);
    return true;
  } catch (...) {
    return false;
  }
}

}  // namespace

namespace xlernav {

bool LoadCalibrationYaml(const std::string & path, CalibrationData & calib)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    return false;
  }

  enum class Section { kNone, kCameraMatrix, kDistCoeffs, kRectification, kProjection };
  Section section = Section::kNone;
  bool in_data = false;

  std::vector<double> k_data;
  std::vector<double> d_data;
  std::vector<double> r_data;
  std::vector<double> p_data;

  std::string line;
  while (std::getline(file, line)) {
    const std::string trimmed = Trim(line);
    if (trimmed.empty()) {
      continue;
    }

    if (StartsWith(trimmed, "image_width:")) {
      ParseInt(Trim(trimmed.substr(std::string("image_width:").size())), calib.width);
      continue;
    }
    if (StartsWith(trimmed, "image_height:")) {
      ParseInt(Trim(trimmed.substr(std::string("image_height:").size())), calib.height);
      continue;
    }
    if (StartsWith(trimmed, "distortion_model:")) {
      calib.distortion_model = Trim(trimmed.substr(std::string("distortion_model:").size()));
      if (!calib.distortion_model.empty() && calib.distortion_model.front() == '"' &&
          calib.distortion_model.back() == '"') {
        calib.distortion_model = calib.distortion_model.substr(1, calib.distortion_model.size() - 2);
      }
      continue;
    }

    if (trimmed == "camera_matrix:") {
      section = Section::kCameraMatrix;
      in_data = false;
      continue;
    }
    if (trimmed == "distortion_coefficients:") {
      section = Section::kDistCoeffs;
      in_data = false;
      continue;
    }
    if (trimmed == "rectification_matrix:") {
      section = Section::kRectification;
      in_data = false;
      continue;
    }
    if (trimmed == "projection_matrix:") {
      section = Section::kProjection;
      in_data = false;
      continue;
    }

    if (StartsWith(trimmed, "data:")) {
      in_data = true;
      if (section == Section::kCameraMatrix) {
        k_data.clear();
      } else if (section == Section::kDistCoeffs) {
        d_data.clear();
      } else if (section == Section::kRectification) {
        r_data.clear();
      } else if (section == Section::kProjection) {
        p_data.clear();
      }
      continue;
    }

    if (!in_data) {
      continue;
    }

    if (trimmed[0] == '-') {
      const std::string value = Trim(trimmed.substr(1));
      double parsed = 0.0;
      if (ParseDouble(value, parsed)) {
        if (section == Section::kCameraMatrix) {
          k_data.push_back(parsed);
        } else if (section == Section::kDistCoeffs) {
          d_data.push_back(parsed);
        } else if (section == Section::kRectification) {
          r_data.push_back(parsed);
        } else if (section == Section::kProjection) {
          p_data.push_back(parsed);
        }
      }
    } else if (std::isdigit(trimmed[0]) || trimmed[0] == '+' || trimmed[0] == '.') {
      double parsed = 0.0;
      if (ParseDouble(trimmed, parsed)) {
        if (section == Section::kCameraMatrix) {
          k_data.push_back(parsed);
        } else if (section == Section::kDistCoeffs) {
          d_data.push_back(parsed);
        } else if (section == Section::kRectification) {
          r_data.push_back(parsed);
        } else if (section == Section::kProjection) {
          p_data.push_back(parsed);
        }
      }
    } else {
      in_data = false;
    }
  }

  if (k_data.size() >= 9) {
    calib.k = k_data;
  }
  if (!d_data.empty()) {
    calib.d = d_data;
  }
  if (r_data.size() >= 9) {
    calib.r = r_data;
  }
  if (p_data.size() >= 12) {
    calib.p = p_data;
  }

  if (calib.distortion_model.empty()) {
    calib.distortion_model = "plumb_bob";
  }

  return calib.k.size() >= 9;
}

bool BuildUndistortMaps(
  const CalibrationData & calib,
  double balance,
  bool use_projection,
  cv::Mat & map1,
  cv::Mat & map2,
  cv::Mat & new_k)
{
  if (calib.k.size() < 9) {
    return false;
  }

  const int width = calib.width > 0 ? calib.width : 640;
  const int height = calib.height > 0 ? calib.height : 480;

  cv::Mat k = (cv::Mat_<double>(3, 3) <<
    calib.k[0], calib.k[1], calib.k[2],
    calib.k[3], calib.k[4], calib.k[5],
    calib.k[6], calib.k[7], calib.k[8]);

  cv::Mat d = cv::Mat::zeros(4, 1, CV_64F);
  if (!calib.d.empty()) {
    const size_t count = std::min<size_t>(4, calib.d.size());
    for (size_t i = 0; i < count; ++i) {
      d.at<double>(static_cast<int>(i), 0) = calib.d[i];
    }
  }

  cv::Mat r = cv::Mat::eye(3, 3, CV_64F);
  if (calib.r.size() >= 9) {
    r = (cv::Mat_<double>(3, 3) <<
      calib.r[0], calib.r[1], calib.r[2],
      calib.r[3], calib.r[4], calib.r[5],
      calib.r[6], calib.r[7], calib.r[8]);
  }

  const cv::Size size(width, height);
  const double clamped_balance = std::clamp(balance, 0.0, 1.0);

  const bool has_projection = calib.p.size() >= 12;
  const bool use_projection_matrix = use_projection && has_projection;
  if (use_projection_matrix) {
    new_k = (cv::Mat_<double>(3, 3) <<
      calib.p[0], calib.p[1], calib.p[2],
      calib.p[4], calib.p[5], calib.p[6],
      calib.p[8], calib.p[9], calib.p[10]);
  }

  if (calib.distortion_model == "equidistant") {
    if (!use_projection_matrix) {
      cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
        k, d, size, r, new_k, clamped_balance);
    }
    cv::fisheye::initUndistortRectifyMap(
      k, d, r, new_k, size, CV_16SC2, map1, map2);
  } else {
    if (!use_projection_matrix) {
      new_k = cv::getOptimalNewCameraMatrix(k, d, size, clamped_balance);
    }
    cv::initUndistortRectifyMap(k, d, r, new_k, size, CV_16SC2, map1, map2);
  }

  return !map1.empty();
}

}  // namespace xlernav
