#include "rerun_stream_logger.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace xlernav {

RerunStreamLogger::RerunStreamLogger(
  const RerunOptions & options,
  float fx,
  float fy,
  int width,
  int height,
  std::string * warning)
#ifdef XLERNAV_WITH_RERUN
: options_(options),
  rec_(options.recording_id.empty() ? "xlernav" : options.recording_id)
#endif
{
#ifdef XLERNAV_WITH_RERUN
  enabled_ = true;

  bool want_spawn = options_.spawn;
  const bool want_save = !options_.save_path.empty();

  if (want_spawn) {
    const auto err = rec_.spawn();
    if (err.is_err()) {
      want_spawn = false;
      if (warning) {
        *warning = "Failed to spawn Rerun viewer from PATH; continuing without live viewer.";
      }
    }
  }

  if (want_save && want_spawn) {
    const auto err = rec_.set_sinks(
      rerun::FileSink{options_.save_path},
      rerun::GrpcSink{});
    if (err.is_err()) {
      enabled_ = false;
      if (warning) {
        *warning = "Failed to configure Rerun file+grpc sinks.";
      }
      return;
    }
  } else if (want_save) {
    const auto err = rec_.save(options_.save_path);
    if (err.is_err()) {
      enabled_ = false;
      if (warning) {
        *warning = "Failed to open Rerun save path: " + options_.save_path;
      }
      return;
    }
  }

  rec_.log_static(
    "world/camera",
    rerun::Pinhole::from_focal_length_and_resolution(
      {fx, fy},
      {static_cast<float>(width), static_cast<float>(height)})
      .with_camera_xyz(rerun::components::ViewCoordinates::RDF));
#else
  (void)options;
  (void)fx;
  (void)fy;
  (void)width;
  (void)height;
  if (warning) {
    *warning = "Binary was built without XLERNAV_WITH_RERUN.";
  }
  enabled_ = false;
#endif
}

void RerunStreamLogger::set_frame_time(double timestamp_s, std::int64_t frame_index)
{
#ifdef XLERNAV_WITH_RERUN
  if (!enabled_) {
    return;
  }
  rec_.set_time_timestamp_secs_since_epoch("capture_time", timestamp_s);
  rec_.set_time_sequence("frame", frame_index);
#else
  (void)timestamp_s;
  (void)frame_index;
#endif
}

void RerunStreamLogger::log_camera_pose(const Eigen::Matrix4f & twc)
{
#ifdef XLERNAV_WITH_RERUN
  if (!enabled_) {
    return;
  }

  const Eigen::Matrix3f r = twc.block<3, 3>(0, 0);
  const Eigen::Vector3f t = twc.block<3, 1>(0, 3);

  Eigen::Quaternionf q(r);
  q.normalize();

  rec_.log(
    "world/camera",
    rerun::Transform3D(
      rerun::components::Translation3D(t.x(), t.y(), t.z()),
      rerun::Rotation3D(rerun::datatypes::Quaternion::from_wxyz(
        q.w(), q.x(), q.y(), q.z()))));
#else
  (void)twc;
#endif
}

void RerunStreamLogger::log_trajectory_point(const Eigen::Vector3f & point)
{
#ifdef XLERNAV_WITH_RERUN
  if (!enabled_) {
    return;
  }
  trajectory_.emplace_back(point.x(), point.y(), point.z());
  rec_.log(
    "world/trajectory",
    rerun::Points3D(trajectory_).with_colors(rerun::Color(70, 220, 90)));
#else
  (void)point;
#endif
}

void RerunStreamLogger::log_rgb(const cv::Mat & bgr)
{
#ifdef XLERNAV_WITH_RERUN
  if (!enabled_ || !options_.log_images || bgr.empty() || bgr.type() != CV_8UC3) {
    return;
  }

  const uint32_t width = static_cast<uint32_t>(bgr.cols);
  const uint32_t height = static_cast<uint32_t>(bgr.rows);
  rec_.log(
    "world/camera/rgb",
    rerun::Image(
      bgr.ptr<uint8_t>(),
      {width, height},
      rerun::datatypes::ColorModel::BGR));
#else
  (void)bgr;
#endif
}

void RerunStreamLogger::log_depth(const cv::Mat & depth_meters)
{
#ifdef XLERNAV_WITH_RERUN
  if (!enabled_ || !options_.log_images || depth_meters.empty()) {
    return;
  }

  cv::Mat depth_float;
  if (depth_meters.type() == CV_32F) {
    depth_float = depth_meters;
  } else {
    depth_meters.convertTo(depth_float, CV_32F);
  }

  cv::Mat depth_u16(depth_float.size(), CV_16UC1, cv::Scalar(0));
  constexpr float kMaxDepthM = 65.535f;
  for (int y = 0; y < depth_float.rows; ++y) {
    const float * in = depth_float.ptr<float>(y);
    uint16_t * out = depth_u16.ptr<uint16_t>(y);
    for (int x = 0; x < depth_float.cols; ++x) {
      const float z = in[x];
      if (!std::isfinite(z) || z <= 0.0f) {
        out[x] = 0;
        continue;
      }
      const float clamped = std::min(z, kMaxDepthM);
      out[x] = static_cast<uint16_t>(std::lround(clamped * 1000.0f));
    }
  }

  const uint32_t width = static_cast<uint32_t>(depth_u16.cols);
  const uint32_t height = static_cast<uint32_t>(depth_u16.rows);
  rec_.log(
    "world/camera/depth",
    rerun::DepthImage(depth_u16.ptr<uint16_t>(), {width, height})
      .with_meter(0.001f));
#else
  (void)depth_meters;
#endif
}

void RerunStreamLogger::log_points(
  const std::string & entity_path,
  const std::vector<Eigen::Vector3f> & points,
  const std::vector<Eigen::Vector3f> * colors)
{
#ifdef XLERNAV_WITH_RERUN
  if (!enabled_ || points.empty()) {
    return;
  }

  auto positions = to_positions(points);

  if (colors && colors->size() == points.size()) {
    auto rr_colors = to_colors(*colors);
    rec_.log(entity_path, rerun::Points3D(positions).with_colors(rr_colors));
  } else {
    rec_.log(entity_path, rerun::Points3D(positions));
  }
#else
  (void)entity_path;
  (void)points;
  (void)colors;
#endif
}

#ifdef XLERNAV_WITH_RERUN
std::vector<rerun::components::Position3D> RerunStreamLogger::to_positions(
  const std::vector<Eigen::Vector3f> & points)
{
  std::vector<rerun::components::Position3D> out;
  out.reserve(points.size());
  for (const auto & p : points) {
    out.emplace_back(p.x(), p.y(), p.z());
  }
  return out;
}

std::vector<rerun::Color> RerunStreamLogger::to_colors(
  const std::vector<Eigen::Vector3f> & colors)
{
  std::vector<rerun::Color> out;
  out.reserve(colors.size());
  for (const auto & c : colors) {
    const uint8_t r = static_cast<uint8_t>(std::clamp(c.x(), 0.0f, 255.0f));
    const uint8_t g = static_cast<uint8_t>(std::clamp(c.y(), 0.0f, 255.0f));
    const uint8_t b = static_cast<uint8_t>(std::clamp(c.z(), 0.0f, 255.0f));
    out.emplace_back(r, g, b);
  }
  return out;
}
#endif

}  // namespace xlernav
