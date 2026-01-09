#pragma once

#include <algorithm>
#include <chrono>

namespace xlernav {

class FpsCounter {
public:
  explicit FpsCounter(double interval_sec)
  : interval_sec_(std::max(interval_sec, 0.1)),
    last_time_(std::chrono::steady_clock::now()),
    fps_(0.0),
    count_(0) {}

  double tick()
  {
    ++count_;
    auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - last_time_).count();
    if (elapsed >= interval_sec_) {
      fps_ = count_ / elapsed;
      count_ = 0;
      last_time_ = now;
    }
    return fps_;
  }

  double fps() const { return fps_; }

private:
  double interval_sec_;
  std::chrono::steady_clock::time_point last_time_;
  double fps_;
  int count_;
};

}  // namespace xlernav
