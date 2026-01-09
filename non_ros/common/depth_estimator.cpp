#include "depth_estimator.hpp"

#include <tensorrt_common/tensorrt_common.hpp>

#include <vector>

namespace xlernav {

DepthEstimator::DepthEstimator(const std::string & engine_path)
  : engine_(engine_path, "fp16", tensorrt_common::BuildConfig(), false, std::string(), {1, 1, 1}, (1 << 30)),
    prepared_(false),
    width_(0),
    height_(0)
{
}

bool DepthEstimator::Infer(const cv::Mat & bgr, const sensor_msgs::msg::CameraInfo & camera_info)
{
  if (bgr.empty()) {
    return false;
  }

  if (!prepared_ || bgr.cols != width_ || bgr.rows != height_) {
    engine_.initPreprocessBuffer(bgr.cols, bgr.rows);
    width_ = bgr.cols;
    height_ = bgr.rows;
    prepared_ = true;
  }

  std::vector<cv::Mat> images{bgr};
  if (!engine_.doInference(images, camera_info, 0, false)) {
    return false;
  }

  depth_ = engine_.getDepthImage();
  return true;
}

const cv::Mat & DepthEstimator::Depth() const
{
  return depth_;
}

}  // namespace xlernav
