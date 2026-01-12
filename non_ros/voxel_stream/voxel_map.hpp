#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

namespace xlernav {

struct VoxelPoint {
  Eigen::Vector3f center;
  Eigen::Vector3f color;
  int score = 0;
};

class VoxelMap {
public:
  VoxelMap(float voxel_size, int size_x, int size_y, int size_z, bool rolling, bool unbounded);

  void Integrate(
    const cv::Mat & depth,
    const cv::Mat & rgb,
    const cv::Mat & k,
    const Eigen::Matrix4f & twc,
    int stride,
    float max_depth,
    float depth_scale);

  void IntegratePoints(const std::vector<VoxelPoint> & voxels);

  std::vector<VoxelPoint> Snapshot(int min_score) const;

  float voxel_size() const { return voxel_size_; }
  std::size_t size() const { return unbounded_ ? map_.size() : grid_.size(); }

private:
  struct Cell {
    int16_t score = 0;
    uint16_t color_count = 0;
    float color_acc[3] = {0.0f, 0.0f, 0.0f};
  };

  struct Vec3iHash {
    std::size_t operator()(const Eigen::Vector3i & v) const noexcept
    {
      const std::size_t h1 = std::hash<int>{}(v.x());
      const std::size_t h2 = std::hash<int>{}(v.y());
      const std::size_t h3 = std::hash<int>{}(v.z());
      return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
  };

  struct Vec3iEqual {
    bool operator()(const Eigen::Vector3i & a, const Eigen::Vector3i & b) const noexcept
    {
      return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
    }
  };

  void Recenter(const Eigen::Vector3f & center);
  void ShiftGrid(const Eigen::Vector3i & delta);
  void ClearAll();
  void ClearSliceX(int logical_x);
  void ClearSliceY(int logical_y);
  void ClearSliceZ(int logical_z);

  bool WorldToIndex(const Eigen::Vector3f & point, Eigen::Vector3i & index) const;
  std::size_t StorageIndex(const Eigen::Vector3i & logical) const;
  std::size_t StorageIndex(int logical_x, int logical_y, int logical_z) const;
  std::size_t StorageIndexStorage(int storage_x, int storage_y, int storage_z) const;

  void UpdateCell(const Eigen::Vector3i & logical, bool occupied, const Eigen::Vector3f * color);

  float voxel_size_;
  float inv_voxel_size_;
  int size_x_;
  int size_y_;
  int size_z_;
  bool rolling_;
  bool unbounded_;
  int occ_inc_;
  int free_dec_;
  int score_min_;
  int score_max_;
  Eigen::Vector3f origin_;
  Eigen::Vector3i offset_;
  bool initialized_;
  std::vector<Cell> grid_;
  std::unordered_map<Eigen::Vector3i, Cell, Vec3iHash, Vec3iEqual> map_;
};

}  // namespace xlernav
