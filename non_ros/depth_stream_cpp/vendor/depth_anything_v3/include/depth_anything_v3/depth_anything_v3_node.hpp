// Copyright 2025 Institute for Automotive Engineering (ika), RWTH Aachen University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DEPTH_ANYTHING_V3__DEPTH_ANYTHING_V3_NODE_HPP_
#define DEPTH_ANYTHING_V3__DEPTH_ANYTHING_V3_NODE_HPP_

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <memory>
#include <deque>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <mutex>
#include <std_msgs/msg/header.hpp>

#include "depth_anything_v3/tensorrt_depth_anything.hpp"

namespace depth_anything_v3
{
class DepthAnythingV3Node : public rclcpp::Node
{
public:
  explicit DepthAnythingV3Node(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    std::string onnx_path{};
    std::string precision{};
    bool enable_debug{};
    std::string debug_colormap{};
    std::string debug_filepath{};
    bool write_colormap{};
    double debug_colormap_min_depth{};  // Minimum depth value for colormap visualization
    double debug_colormap_max_depth{};  // Maximum depth value for colormap visualization
    double sky_threshold{};             // Threshold for sky classification
    double sky_depth_cap{};             // Cap for sky depth fill-in
    bool publish_point_cloud{};  // Enable point cloud generation/publishing
    int point_cloud_downsample_factor{};  // Only publish every Nth point (1 = no downsampling)
    bool colorize_point_cloud{};  // Add RGB colors from input image to point cloud
  };

private:
  class RollingStats {
   public:
    explicit RollingStats(size_t max_samples) : max_samples_(max_samples) {}

    void push(double value) {
      values_.push_back(value);
      if (values_.size() > max_samples_) {
        values_.pop_front();
      }
    }

    bool empty() const { return values_.empty(); }

    double mean() const {
      if (values_.empty()) {
        return 0.0;
      }
      double sum = 0.0;
      for (double v : values_) {
        sum += v;
      }
      return sum / static_cast<double>(values_.size());
    }

   private:
    size_t max_samples_;
    std::deque<double> values_;
  };

  // Subscribers for raw image and camera_info
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr debug_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr debug_compressed_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr debug_camera_info_sub_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr last_camera_info_;
  std::mutex camera_info_mutex_;

  // Callbacks
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
  void onCompressedImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & image_msg);
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);
  void processCvImage(
    const std_msgs::msg::Header & header,
    const cv::Mat & image,
    const sensor_msgs::msg::CameraInfo & camera_info_msg);
    
  // Debug callbacks for individual topics
  void onImageDebug(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void onCompressedImageDebug(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg);
  void onCameraInfoDebug(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_image_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_point_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_image_debug_;

  // Helper methods
  int getColorMapType(const std::string& colormap_name);

  // Parameter Server
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

  // Parameter
  NodeParam node_param_{};

  // Core
  std::shared_ptr<TensorRTDepthAnything> tensorrt_depth_anything_;
  bool is_initialized_ = false;

  RollingStats inference_time_stats_{60};
  RollingStats total_time_stats_{60};
  RollingStats input_fps_stats_{60};
  rclcpp::Time last_input_stamp_;
};

} // namespace depth_anything_v3

#endif // DEPTH_ANYTHING_V3__DEPTH_ANYTHING_V3_NODE_HPP_
