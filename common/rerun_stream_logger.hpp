#pragma once

#include <Eigen/Core>

#include <opencv2/core.hpp>

#include <cstdint>
#include <string>
#include <vector>

#ifdef XLERNAV_WITH_RERUN
#include <rerun.hpp>
#endif

namespace xlernav {

struct RerunOptions {
  bool spawn = true;
  std::string save_path;
  std::string recording_id = "xlernav";
  std::size_t log_every_n = 3;
  bool log_images = true;
  float depth_meter = 1.0f;
};

class RerunStreamLogger {
public:
  RerunStreamLogger(
    const RerunOptions & options,
    float fx,
    float fy,
    int width,
    int height,
    std::string * warning);

  bool enabled() const { return enabled_; }

  void set_frame_time(double timestamp_s, std::int64_t frame_index);

  void log_camera_pose(const Eigen::Matrix4f & twc);

  void log_trajectory_point(const Eigen::Vector3f & point);

  void log_rgb(const cv::Mat & bgr);

  void log_depth(const cv::Mat & depth_meters);

  void log_points(
    const std::string & entity_path,
    const std::vector<Eigen::Vector3f> & points,
    const std::vector<Eigen::Vector3f> * colors = nullptr);

private:
#ifdef XLERNAV_WITH_RERUN
  static std::vector<rerun::components::Position3D> to_positions(
    const std::vector<Eigen::Vector3f> & points);

  static std::vector<rerun::Color> to_colors(
    const std::vector<Eigen::Vector3f> & colors);

  RerunOptions options_;
  rerun::RecordingStream rec_;
  std::vector<rerun::components::Position3D> trajectory_;
#endif

  bool enabled_ = false;
};

}  // namespace xlernav
