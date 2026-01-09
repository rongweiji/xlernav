#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <cstddef>
#include <unordered_map>
#include <vector>

namespace xlernav {

struct VoxelKey {
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey & key) const noexcept;
};

struct VoxelData {
  int score = 0;
  Eigen::Vector3f color_acc = Eigen::Vector3f::Zero();
  int color_count = 0;
  double last_update = 0.0;
};

struct VoxelPoint {
  Eigen::Vector3f center;
  Eigen::Vector3f color;
  int score = 0;
};

class VoxelMap {
public:
  VoxelMap(float voxel_size, std::size_t max_voxels, double decay_sec);

  void Integrate(
    const cv::Mat & depth,
    const cv::Mat & rgb,
    const cv::Mat & k,
    const Eigen::Matrix4f & twc,
    int stride,
    float max_depth,
    float depth_scale,
    double now_sec);

  std::vector<VoxelPoint> Snapshot(int min_score) const;

  float voxel_size() const { return voxel_size_; }
  std::size_t size() const { return voxels_.size(); }

private:
  void UpdateVoxel(
    const VoxelKey & key,
    bool occupied,
    const Eigen::Vector3f * color,
    double now_sec);

  void PruneStale(double now_sec);

  float voxel_size_;
  float inv_voxel_size_;
  std::size_t max_voxels_;
  double decay_sec_;
  double last_cleanup_sec_;
  int occ_inc_;
  int free_dec_;
  int score_min_;
  int score_max_;
  std::unordered_map<VoxelKey, VoxelData, VoxelKeyHash> voxels_;
};

}  // namespace xlernav
