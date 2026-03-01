#pragma once

#include <depth_anything_v3/tensorrt_depth_anything.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <opencv2/opencv.hpp>

#include <string>

namespace xlernav {

class DepthEstimator {
public:
  explicit DepthEstimator(const std::string & engine_path);

  bool Infer(const cv::Mat & bgr, const sensor_msgs::msg::CameraInfo & camera_info);
  const cv::Mat & Depth() const;

private:
  depth_anything_v3::TensorRTDepthAnything engine_;
  bool prepared_;
  int width_;
  int height_;
  cv::Mat depth_;
};

}  // namespace xlernav
