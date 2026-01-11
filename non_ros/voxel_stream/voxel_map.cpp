#include "voxel_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace xlernav {

namespace {
int wrap_index(int value, int mod)
{
  if (mod <= 0) {
    return 0;
  }
  int result = value % mod;
  if (result < 0) {
    result += mod;
  }
  return result;
}
}  // namespace

VoxelMap::VoxelMap(float voxel_size, int size_x, int size_y, int size_z, double decay_sec)
  : voxel_size_(voxel_size > 0.0f ? voxel_size : 0.1f),
    inv_voxel_size_(1.0f / voxel_size_),
    size_x_(std::max(1, size_x)),
    size_y_(std::max(1, size_y)),
    size_z_(std::max(1, size_z)),
    decay_sec_(decay_sec),
    occ_inc_(2),
    free_dec_(1),
    score_min_(-50),
    score_max_(50),
    origin_(Eigen::Vector3f::Zero()),
    offset_(Eigen::Vector3i::Zero()),
    initialized_(false),
    last_time_sec_(0.0),
    grid_(static_cast<std::size_t>(size_x_ * size_y_ * size_z_))
{
}

void VoxelMap::ClearAll()
{
  std::fill(grid_.begin(), grid_.end(), Cell{});
}

std::size_t VoxelMap::StorageIndexStorage(int storage_x, int storage_y, int storage_z) const
{
  return (static_cast<std::size_t>(storage_x) * size_y_ + storage_y) * size_z_ + storage_z;
}

std::size_t VoxelMap::StorageIndex(int logical_x, int logical_y, int logical_z) const
{
  const int sx = wrap_index(logical_x + offset_.x(), size_x_);
  const int sy = wrap_index(logical_y + offset_.y(), size_y_);
  const int sz = wrap_index(logical_z + offset_.z(), size_z_);
  return StorageIndexStorage(sx, sy, sz);
}

std::size_t VoxelMap::StorageIndex(const Eigen::Vector3i & logical) const
{
  return StorageIndex(logical.x(), logical.y(), logical.z());
}

void VoxelMap::ClearSliceX(int logical_x)
{
  const int sx = wrap_index(logical_x + offset_.x(), size_x_);
  for (int y = 0; y < size_y_; ++y) {
    for (int z = 0; z < size_z_; ++z) {
      grid_[StorageIndexStorage(sx, y, z)] = Cell{};
    }
  }
}

void VoxelMap::ClearSliceY(int logical_y)
{
  const int sy = wrap_index(logical_y + offset_.y(), size_y_);
  for (int x = 0; x < size_x_; ++x) {
    for (int z = 0; z < size_z_; ++z) {
      grid_[StorageIndexStorage(x, sy, z)] = Cell{};
    }
  }
}

void VoxelMap::ClearSliceZ(int logical_z)
{
  const int sz = wrap_index(logical_z + offset_.z(), size_z_);
  for (int x = 0; x < size_x_; ++x) {
    for (int y = 0; y < size_y_; ++y) {
      grid_[StorageIndexStorage(x, y, sz)] = Cell{};
    }
  }
}

void VoxelMap::ShiftGrid(const Eigen::Vector3i & delta)
{
  if (delta == Eigen::Vector3i::Zero()) {
    return;
  }

  origin_ += delta.cast<float>() * voxel_size_;

  offset_.x() = wrap_index(offset_.x() + delta.x(), size_x_);
  offset_.y() = wrap_index(offset_.y() + delta.y(), size_y_);
  offset_.z() = wrap_index(offset_.z() + delta.z(), size_z_);

  if (delta.x() > 0) {
    const int start = size_x_ - delta.x();
    for (int i = start; i < size_x_; ++i) {
      ClearSliceX(i);
    }
  } else if (delta.x() < 0) {
    for (int i = 0; i < -delta.x(); ++i) {
      ClearSliceX(i);
    }
  }

  if (delta.y() > 0) {
    const int start = size_y_ - delta.y();
    for (int i = start; i < size_y_; ++i) {
      ClearSliceY(i);
    }
  } else if (delta.y() < 0) {
    for (int i = 0; i < -delta.y(); ++i) {
      ClearSliceY(i);
    }
  }

  if (delta.z() > 0) {
    const int start = size_z_ - delta.z();
    for (int i = start; i < size_z_; ++i) {
      ClearSliceZ(i);
    }
  } else if (delta.z() < 0) {
    for (int i = 0; i < -delta.z(); ++i) {
      ClearSliceZ(i);
    }
  }
}

void VoxelMap::Recenter(const Eigen::Vector3f & center)
{
  if (!initialized_) {
    origin_ = center - Eigen::Vector3f(size_x_, size_y_, size_z_) * (0.5f * voxel_size_);
    offset_ = Eigen::Vector3i::Zero();
    ClearAll();
    initialized_ = true;
    return;
  }

  const Eigen::Vector3f rel = (center - origin_) * inv_voxel_size_;
  const Eigen::Vector3i idx(
    static_cast<int>(std::floor(rel.x())),
    static_cast<int>(std::floor(rel.y())),
    static_cast<int>(std::floor(rel.z())));
  const Eigen::Vector3i desired(size_x_ / 2, size_y_ / 2, size_z_ / 2);
  const Eigen::Vector3i delta = idx - desired;

  if (delta == Eigen::Vector3i::Zero()) {
    return;
  }

  if (std::abs(delta.x()) >= size_x_ || std::abs(delta.y()) >= size_y_ || std::abs(delta.z()) >= size_z_) {
    origin_ = center - Eigen::Vector3f(size_x_, size_y_, size_z_) * (0.5f * voxel_size_);
    offset_ = Eigen::Vector3i::Zero();
    ClearAll();
    return;
  }

  ShiftGrid(delta);
}

bool VoxelMap::WorldToIndex(const Eigen::Vector3f & point, Eigen::Vector3i & index) const
{
  const Eigen::Vector3f rel = (point - origin_) * inv_voxel_size_;
  const int ix = static_cast<int>(std::floor(rel.x()));
  const int iy = static_cast<int>(std::floor(rel.y()));
  const int iz = static_cast<int>(std::floor(rel.z()));
  if (ix < 0 || ix >= size_x_ || iy < 0 || iy >= size_y_ || iz < 0 || iz >= size_z_) {
    return false;
  }
  index = Eigen::Vector3i(ix, iy, iz);
  return true;
}

void VoxelMap::UpdateCell(
  const Eigen::Vector3i & logical,
  bool occupied,
  const Eigen::Vector3f * color,
  double now_sec)
{
  Cell & cell = grid_[StorageIndex(logical)];
  if (decay_sec_ > 0.0 && cell.last_update > 0.0f && (now_sec - cell.last_update) >= decay_sec_) {
    cell = Cell{};
  }

  if (!occupied && cell.score == 0) {
    return;
  }

  const int delta = occupied ? occ_inc_ : -free_dec_;
  const int next_score = std::clamp(static_cast<int>(cell.score) + delta, score_min_, score_max_);
  cell.score = static_cast<int16_t>(next_score);

  if (occupied && color) {
    cell.color_acc[0] += color->x();
    cell.color_acc[1] += color->y();
    cell.color_acc[2] += color->z();
    if (cell.color_count < std::numeric_limits<uint16_t>::max()) {
      ++cell.color_count;
    }
  }

  cell.last_update = static_cast<float>(now_sec);
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

  last_time_sec_ = now_sec;
  const Eigen::Vector3f origin = twc.block<3, 1>(0, 3);
  Recenter(origin);

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
      Eigen::Vector3i last_idx(std::numeric_limits<int>::max(),
                               std::numeric_limits<int>::max(),
                               std::numeric_limits<int>::max());

      if (step > 0.0f) {
        for (float d = step; d < (dist - 0.5f * step); d += step) {
          const Eigen::Vector3f p = origin + dir_norm * d;
          Eigen::Vector3i idx;
          if (!WorldToIndex(p, idx)) {
            continue;
          }
          if (idx == last_idx) {
            continue;
          }
          last_idx = idx;
          UpdateCell(idx, false, nullptr, now_sec);
        }
      }

      Eigen::Vector3i occ_idx;
      if (!WorldToIndex(pw, occ_idx)) {
        continue;
      }

      if (!rgb.empty()) {
        const cv::Vec3b color = rgb.at<cv::Vec3b>(v, u);
        const Eigen::Vector3f color_f(
          static_cast<float>(color[2]),
          static_cast<float>(color[1]),
          static_cast<float>(color[0]));
        const Eigen::Vector3f color_norm = color_f / 255.0f;
        UpdateCell(occ_idx, true, &color_norm, now_sec);
      } else {
        UpdateCell(occ_idx, true, nullptr, now_sec);
      }
    }
  }
}

std::vector<VoxelPoint> VoxelMap::Snapshot(int min_score) const
{
  if (min_score < 1) {
    min_score = 1;
  }

  std::vector<VoxelPoint> result;
  result.reserve(grid_.size());

  const double now_sec = last_time_sec_;

  for (int x = 0; x < size_x_; ++x) {
    for (int y = 0; y < size_y_; ++y) {
      for (int z = 0; z < size_z_; ++z) {
        const Cell & cell = grid_[StorageIndex(x, y, z)];
        if (cell.score < min_score) {
          continue;
        }
        if (decay_sec_ > 0.0 && cell.last_update > 0.0f && (now_sec - cell.last_update) >= decay_sec_) {
          continue;
        }

        const Eigen::Vector3f center(
          origin_.x() + (static_cast<float>(x) + 0.5f) * voxel_size_,
          origin_.y() + (static_cast<float>(y) + 0.5f) * voxel_size_,
          origin_.z() + (static_cast<float>(z) + 0.5f) * voxel_size_);

        Eigen::Vector3f color(0.9f, 0.9f, 0.9f);
        if (cell.color_count > 0) {
          const float inv_count = 1.0f / static_cast<float>(cell.color_count);
          color = Eigen::Vector3f(
            cell.color_acc[0] * inv_count,
            cell.color_acc[1] * inv_count,
            cell.color_acc[2] * inv_count);
        }

        result.push_back({center, color, static_cast<int>(cell.score)});
      }
    }
  }

  return result;
}

}  // namespace xlernav
