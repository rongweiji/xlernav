#pragma once

#include <opencv2/core.hpp>

#include <string>
#include <vector>

struct CalibrationData {
  int width = 0;
  int height = 0;
  std::string distortion_model;
  std::vector<double> k;
  std::vector<double> d;
  std::vector<double> r;
  std::vector<double> p;
};

bool LoadCalibrationYaml(const std::string & path, CalibrationData & calib);

bool BuildUndistortMaps(
  const CalibrationData & calib,
  double balance,
  bool use_projection,
  cv::Mat & map1,
  cv::Mat & map2,
  cv::Mat & new_k);
