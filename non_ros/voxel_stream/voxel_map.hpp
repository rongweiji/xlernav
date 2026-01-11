#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace xlernav {

struct VoxelPoint {
  Eigen::Vector3f center;
  Eigen::Vector3f color;
  int score = 0;
};

class VoxelMap {
public:
  VoxelMap(float voxel_size, int size_x, int size_y, int size_z, double decay_sec);

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
  std::size_t size() const { return grid_.size(); }

private:
  struct Cell {
    int16_t score = 0;
    uint16_t color_count = 0;
    float color_acc[3] = {0.0f, 0.0f, 0.0f};
    float last_update = 0.0f;
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

  void UpdateCell(const Eigen::Vector3i & logical, bool occupied, const Eigen::Vector3f * color, double now_sec);

  float voxel_size_;
  float inv_voxel_size_;
  int size_x_;
  int size_y_;
  int size_z_;
  double decay_sec_;
  int occ_inc_;
  int free_dec_;
  int score_min_;
  int score_max_;
  Eigen::Vector3f origin_;
  Eigen::Vector3i offset_;
  bool initialized_;
  double last_time_sec_;
  std::vector<Cell> grid_;
};

}  // namespace xlernav
