#include "voxel_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace xlernav {

std::size_t VoxelKeyHash::operator()(const VoxelKey & key) const noexcept
{
  std::size_t h1 = std::hash<int>{}(key.x);
  std::size_t h2 = std::hash<int>{}(key.y);
  std::size_t h3 = std::hash<int>{}(key.z);
  std::size_t seed = h1 ^ (h2 + 0x9e3779b9U + (h1 << 6) + (h1 >> 2));
  return seed ^ (h3 + 0x9e3779b9U + (seed << 6) + (seed >> 2));
}

VoxelMap::VoxelMap(float voxel_size, std::size_t max_voxels, double decay_sec)
  : voxel_size_(voxel_size > 0.0f ? voxel_size : 0.1f),
    inv_voxel_size_(1.0f / voxel_size_),
    max_voxels_(max_voxels),
    decay_sec_(decay_sec),
    last_cleanup_sec_(0.0),
    occ_inc_(2),
    free_dec_(1),
    score_min_(-50),
    score_max_(50)
{
}

void VoxelMap::UpdateVoxel(
  const VoxelKey & key,
  bool occupied,
  const Eigen::Vector3f * color,
  double now_sec)
{
  auto it = voxels_.find(key);
  if (it == voxels_.end()) {
    if (!occupied) {
      return;
    }
    if (max_voxels_ > 0 && voxels_.size() >= max_voxels_) {
      return;
    }
    it = voxels_.emplace(key, VoxelData{}).first;
  }

  VoxelData & voxel = it->second;
  const int delta = occupied ? occ_inc_ : -free_dec_;
  voxel.score = std::clamp(voxel.score + delta, score_min_, score_max_);

  if (occupied && color) {
    voxel.color_acc += *color;
    ++voxel.color_count;
  }

  voxel.last_update = now_sec;
}

void VoxelMap::PruneStale(double now_sec)
{
  if (decay_sec_ <= 0.0) {
    return;
  }

  if (last_cleanup_sec_ > 0.0 && (now_sec - last_cleanup_sec_) < 1.0) {
    return;
  }
  last_cleanup_sec_ = now_sec;

  for (auto it = voxels_.begin(); it != voxels_.end(); ) {
    if ((now_sec - it->second.last_update) >= decay_sec_) {
      it = voxels_.erase(it);
    } else {
      ++it;
    }
  }
}

void VoxelMap::Integrate(
  const cv::Mat & depth,
  const cv::Mat & rgb,
  const cv::Mat & k,
  const Eigen::Matrix4f & twc,
  int stride,
  float max_depth,
  float depth_scale,
  double now_sec)
{
  if (depth.empty()) {
    return;
  }

  if (stride < 1) {
    stride = 1;
  }
  if (max_depth <= 0.0f) {
    max_depth = std::numeric_limits<float>::infinity();
  }
  if (depth_scale <= 0.0f) {
    depth_scale = 1.0f;
  }

  float fx = 0.0f;
  float fy = 0.0f;
  float cx = 0.0f;
  float cy = 0.0f;
  if (k.type() == CV_64F) {
    fx = static_cast<float>(k.at<double>(0, 0));
    fy = static_cast<float>(k.at<double>(1, 1));
    cx = static_cast<float>(k.at<double>(0, 2));
    cy = static_cast<float>(k.at<double>(1, 2));
  } else {
    fx = k.at<float>(0, 0);
    fy = k.at<float>(1, 1);
    cx = k.at<float>(0, 2);
    cy = k.at<float>(1, 2);
  }

  if (fx <= 0.0f || fy <= 0.0f) {
    return;
  }

  cv::Mat depth_float;
  if (depth.type() == CV_32F) {
    depth_float = depth;
  } else {
    depth.convertTo(depth_float, CV_32F);
  }

  const Eigen::Matrix3f r = twc.block<3, 3>(0, 0);
  const Eigen::Vector3f t = twc.block<3, 1>(0, 3);
  const Eigen::Vector3f origin = t;
  const float step = voxel_size_;

  for (int v = 0; v < depth_float.rows; v += stride) {
    const float * row = depth_float.ptr<float>(v);
    for (int u = 0; u < depth_float.cols; u += stride) {
      const float z = row[u] * depth_scale;
      if (!std::isfinite(z) || z <= 0.0f || z > max_depth) {
        continue;
      }

      const float x = (static_cast<float>(u) - cx) * z / fx;
      const float y = (static_cast<float>(v) - cy) * z / fy;

      const Eigen::Vector3f pw = r * Eigen::Vector3f(x, y, z) + t;
      const Eigen::Vector3f dir = pw - origin;
      const float dist = dir.norm();
      if (!std::isfinite(dist) || dist <= 0.0f) {
        continue;
      }

      const Eigen::Vector3f dir_norm = dir / dist;
      VoxelKey last_key{std::numeric_limits<int>::min(),
                        std::numeric_limits<int>::min(),
                        std::numeric_limits<int>::min()};

      if (step > 0.0f) {
        for (float d = step; d < (dist - 0.5f * step); d += step) {
          const Eigen::Vector3f p = origin + dir_norm * d;
          const VoxelKey key{
            static_cast<int>(std::floor(p.x() * inv_voxel_size_)),
            static_cast<int>(std::floor(p.y() * inv_voxel_size_)),
            static_cast<int>(std::floor(p.z() * inv_voxel_size_))};
          if (key == last_key) {
            continue;
          }
          last_key = key;
          UpdateVoxel(key, false, nullptr, now_sec);
        }
      }

      const VoxelKey occ_key{
        static_cast<int>(std::floor(pw.x() * inv_voxel_size_)),
        static_cast<int>(std::floor(pw.y() * inv_voxel_size_)),
        static_cast<int>(std::floor(pw.z() * inv_voxel_size_))};

      if (!rgb.empty()) {
        const cv::Vec3b color = rgb.at<cv::Vec3b>(v, u);
        const Eigen::Vector3f color_f(
          static_cast<float>(color[2]),
          static_cast<float>(color[1]),
          static_cast<float>(color[0]));
        const Eigen::Vector3f color_norm = color_f / 255.0f;
        UpdateVoxel(occ_key, true, &color_norm, now_sec);
      } else {
        UpdateVoxel(occ_key, true, nullptr, now_sec);
      }
    }
  }

  PruneStale(now_sec);
}

std::vector<VoxelPoint> VoxelMap::Snapshot(int min_score) const
{
  if (min_score < 1) {
    min_score = 1;
  }

  std::vector<VoxelPoint> result;
  result.reserve(voxels_.size());

  for (const auto & entry : voxels_) {
    const VoxelKey & key = entry.first;
    const VoxelData & voxel = entry.second;
    if (voxel.score < min_score) {
      continue;
    }

    const Eigen::Vector3f center(
      (static_cast<float>(key.x) + 0.5f) * voxel_size_,
      (static_cast<float>(key.y) + 0.5f) * voxel_size_,
      (static_cast<float>(key.z) + 0.5f) * voxel_size_);

    const Eigen::Vector3f color = voxel.color_count > 0
      ? voxel.color_acc / static_cast<float>(voxel.color_count)
      : Eigen::Vector3f(0.9f, 0.9f, 0.9f);

    result.push_back({center, color, voxel.score});
  }

  return result;
}

}  // namespace xlernav
