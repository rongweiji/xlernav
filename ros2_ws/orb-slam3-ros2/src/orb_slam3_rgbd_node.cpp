#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "System.h"
#include "Tracking.h"

namespace orb_slam3_ros2 {

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

  double min() const {
    if (values_.empty()) {
      return 0.0;
    }
    double m = values_.front();
    for (double v : values_) {
      m = std::min(m, v);
    }
    return m;
  }

  double max() const {
    if (values_.empty()) {
      return 0.0;
    }
    double m = values_.front();
    for (double v : values_) {
      m = std::max(m, v);
    }
    return m;
  }

  size_t size() const { return values_.size(); }

 private:
  size_t max_samples_;
  std::deque<double> values_;
};

struct DepthStats {
  double min = 0.0;
  double max = 0.0;
  double mean = 0.0;
  double median = 0.0;
  size_t count = 0;
};

class OrbSlam3RgbdNode : public rclcpp::Node {
 public:
  OrbSlam3RgbdNode()
      : Node("orb_slam3_rgbd"),
        rgb_fps_stats_(60),
        depth_fps_stats_(60),
        sync_dt_stats_(60),
        track_time_stats_(60) {
    vocab_path_ = declare_parameter<std::string>(
        "vocab_path", "/mnt/g/LocalGitProject/ORB_SLAM3/Vocabulary/ORBvoc.txt");
    settings_path_ = declare_parameter<std::string>(
        "settings_path",
        "/mnt/g/LocalGitProject/ORB_SLAM3/Examples/RGB-D/iPhone_640x480_metric_depth.yaml");
    rgb_topic_ = declare_parameter<std::string>("rgb_topic", "/image_raw");
    depth_topic_ = declare_parameter<std::string>(
        "depth_topic", "/depth_anything_v3/output/depth_image");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera_info");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    enable_viewer_ = declare_parameter<bool>("enable_viewer", false);
    image_encoding_ = declare_parameter<std::string>("image_encoding", "");
    sync_queue_size_ = declare_parameter<int>("sync_queue_size", 10);
    sync_slop_sec_ = declare_parameter<double>("sync_slop_sec", 0.05);
    diagnostics_period_sec_ = declare_parameter<double>("diagnostics_period_sec", 2.0);
    depth_stats_stride_ = declare_parameter<int>("depth_stats_stride", 8);
    lock_map_ = declare_parameter<bool>("lock_map", false);
    lock_map_after_frames_ = declare_parameter<int>("lock_map_after_frames", 1);

    if (lock_map_after_frames_ < 1) {
      lock_map_after_frames_ = 1;
    }

    if (vocab_path_.empty() || settings_path_.empty()) {
      throw std::runtime_error("vocab_path and settings_path must be set");
    }

    RCLCPP_INFO(get_logger(), "ORB-SLAM3 vocab: %s", vocab_path_.c_str());
    RCLCPP_INFO(get_logger(), "ORB-SLAM3 settings: %s", settings_path_.c_str());
    RCLCPP_INFO(get_logger(), "Subscribing RGB: %s", rgb_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Subscribing depth: %s", depth_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Subscribing camera_info: %s", camera_info_topic_.c_str());
    if (lock_map_) {
      RCLCPP_WARN(get_logger(),
                  "lock_map enabled: switching to localization-only after %d tracking OK frames",
                  lock_map_after_frames_);
    }

    slam_ = std::make_unique<ORB_SLAM3::System>(
        vocab_path_, settings_path_, ORB_SLAM3::System::RGBD, enable_viewer_);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        "orb_slam3/odom", rclcpp::SystemDefaultsQoS());
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "orb_slam3/pose", rclcpp::SystemDefaultsQoS());

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, rclcpp::SensorDataQoS(),
        std::bind(&OrbSlam3RgbdNode::cameraInfoCallback, this, std::placeholders::_1));

    rgb_sub_.subscribe(this, rgb_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());
    depth_sub_.subscribe(this, depth_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());

    sync_ = std::make_shared<Sync>(SyncPolicy(sync_queue_size_), rgb_sub_, depth_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_sec_));
    sync_->registerCallback(
        std::bind(&OrbSlam3RgbdNode::rgbdCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    diagnostics_timer_ = create_wall_timer(
        std::chrono::duration<double>(diagnostics_period_sec_),
        std::bind(&OrbSlam3RgbdNode::publishDiagnostics, this));
  }

  ~OrbSlam3RgbdNode() override {
    if (slam_) {
      slam_->Shutdown();
    }
  }

 private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    last_camera_info_ = *msg;
    if (!camera_info_logged_) {
      RCLCPP_INFO(get_logger(), "CameraInfo width=%u height=%u K=[%.3f %.3f %.3f %.3f]", msg->width,
                  msg->height, msg->k[0], msg->k[2], msg->k[4], msg->k[5]);
      camera_info_logged_ = true;
    }
  }

  void rgbdCallback(const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
                    const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg) {
    updateRates(rgb_msg, depth_msg);

    cv_bridge::CvImageConstPtr rgb_cv;
    cv_bridge::CvImageConstPtr depth_cv;

    try {
      if (!image_encoding_.empty()) {
        rgb_cv = cv_bridge::toCvShare(rgb_msg, image_encoding_);
      } else {
        rgb_cv = cv_bridge::toCvShare(rgb_msg);
      }
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to convert RGB image: %s", e.what());
      return;
    }

    try {
      depth_cv = cv_bridge::toCvShare(depth_msg);
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to convert depth image: %s", e.what());
      return;
    }

    if (rgb_cv->image.empty() || depth_cv->image.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Received empty RGB or depth image");
      return;
    }

    const rclcpp::Time stamp = rgb_msg->header.stamp;
    const double timestamp_sec = stamp.seconds();

    const auto start = std::chrono::steady_clock::now();
    Sophus::SE3f Tcw = slam_->TrackRGBD(rgb_cv->image, depth_cv->image, timestamp_sec);
    const auto end = std::chrono::steady_clock::now();

    const double track_ms = std::chrono::duration<double, std::milli>(end - start).count();
    track_time_stats_.push(track_ms);

    const int tracking_state = slam_->GetTrackingState();
    if (tracking_state == ORB_SLAM3::Tracking::OK ||
        tracking_state == ORB_SLAM3::Tracking::OK_KLT) {
      publishPose(Tcw, stamp, rgb_msg->header.frame_id);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Tracking state not OK (%d)", tracking_state);
    }

    maybeLockMap(tracking_state);

    depth_stats_counter_++;
    if (depth_stats_counter_ % 30 == 0) {
      last_depth_stats_ = computeDepthStats(depth_cv->image, depth_stats_stride_);
      last_depth_encoding_ = depth_msg->encoding;
    }
  }

  void publishPose(const Sophus::SE3f &Tcw, const rclcpp::Time &stamp,
                   const std::string &frame_id) {
    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Matrix3f R = Twc.rotationMatrix();
    Eigen::Quaternionf q(R);
    Eigen::Vector3f t = Twc.translation();

    const std::string child_frame = camera_frame_.empty() ? frame_id : camera_frame_;

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.position.x = t.x();
    pose_msg.pose.position.y = t.y();
    pose_msg.pose.position.z = t.z();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();
    pose_pub_->publish(pose_msg);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header = pose_msg.header;
    odom_msg.child_frame_id = child_frame;
    odom_msg.pose.pose = pose_msg.pose;
    odom_pub_->publish(odom_msg);

    if (publish_tf_ && !child_frame.empty()) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header = pose_msg.header;
      tf_msg.child_frame_id = child_frame;
      tf_msg.transform.translation.x = t.x();
      tf_msg.transform.translation.y = t.y();
      tf_msg.transform.translation.z = t.z();
      tf_msg.transform.rotation = pose_msg.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);
    }
  }

  void maybeLockMap(int tracking_state) {
    if (!lock_map_ || localization_mode_active_) {
      return;
    }
    if (tracking_state == ORB_SLAM3::Tracking::OK ||
        tracking_state == ORB_SLAM3::Tracking::OK_KLT) {
      tracking_ok_frames_++;
      if (tracking_ok_frames_ >= lock_map_after_frames_) {
        slam_->ActivateLocalizationMode();
        localization_mode_active_ = true;
        RCLCPP_WARN(get_logger(),
                    "Localization-only mode enabled (map updates/new maps disabled).");
      }
    }
  }

  void updateRates(const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
                   const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg) {
    const rclcpp::Time rgb_stamp(rgb_msg->header.stamp);
    const rclcpp::Time depth_stamp(depth_msg->header.stamp);

    if (last_rgb_stamp_.nanoseconds() > 0) {
      const double dt = (rgb_stamp - last_rgb_stamp_).seconds();
      if (dt > 0.0) {
        rgb_fps_stats_.push(1.0 / dt);
      }
    }
    if (last_depth_stamp_.nanoseconds() > 0) {
      const double dt = (depth_stamp - last_depth_stamp_).seconds();
      if (dt > 0.0) {
        depth_fps_stats_.push(1.0 / dt);
      }
    }

    const double sync_dt = std::fabs((rgb_stamp - depth_stamp).seconds());
    sync_dt_stats_.push(sync_dt);

    last_rgb_stamp_ = rgb_stamp;
    last_depth_stamp_ = depth_stamp;
  }

  DepthStats computeDepthStats(const cv::Mat &depth, int stride) {
    DepthStats stats;
    if (depth.empty()) {
      return stats;
    }

    const int step = std::max(1, stride);
    std::vector<float> samples;
    samples.reserve((depth.rows / step) * (depth.cols / step));

    if (depth.type() == CV_32F) {
      for (int v = 0; v < depth.rows; v += step) {
        const float *row = depth.ptr<float>(v);
        for (int u = 0; u < depth.cols; u += step) {
          float value = row[u];
          if (std::isfinite(value) && value > 0.0f) {
            samples.push_back(value);
          }
        }
      }
    } else if (depth.type() == CV_16U) {
      for (int v = 0; v < depth.rows; v += step) {
        const uint16_t *row = depth.ptr<uint16_t>(v);
        for (int u = 0; u < depth.cols; u += step) {
          float value = static_cast<float>(row[u]);
          if (value > 0.0f) {
            samples.push_back(value);
          }
        }
      }
    } else {
      return stats;
    }

    if (samples.empty()) {
      return stats;
    }

    double sum = 0.0;
    stats.min = samples.front();
    stats.max = samples.front();
    for (float v : samples) {
      sum += v;
      stats.min = std::min(stats.min, static_cast<double>(v));
      stats.max = std::max(stats.max, static_cast<double>(v));
    }
    stats.mean = sum / static_cast<double>(samples.size());
    stats.count = samples.size();

    std::nth_element(samples.begin(), samples.begin() + samples.size() / 2, samples.end());
    stats.median = samples[samples.size() / 2];
    return stats;
  }

  void publishDiagnostics() {
    if (!rgb_fps_stats_.empty()) {
      RCLCPP_INFO(get_logger(), "RGB FPS avg=%.2f", rgb_fps_stats_.mean());
    }
    if (!depth_fps_stats_.empty()) {
      RCLCPP_INFO(get_logger(), "Depth FPS avg=%.2f", depth_fps_stats_.mean());
    }
    if (!sync_dt_stats_.empty()) {
      RCLCPP_INFO(get_logger(), "RGB-Depth sync dt avg=%.4f s (max=%.4f)",
                  sync_dt_stats_.mean(), sync_dt_stats_.max());
    }
    if (!track_time_stats_.empty()) {
      RCLCPP_INFO(get_logger(), "TrackRGBD time avg=%.2f ms", track_time_stats_.mean());
    }
    if (last_depth_stats_.count > 0) {
      RCLCPP_INFO(get_logger(),
                  "Depth stats (raw, %s) min=%.3f mean=%.3f median=%.3f max=%.3f",
                  last_depth_encoding_.c_str(), last_depth_stats_.min, last_depth_stats_.mean,
                  last_depth_stats_.median, last_depth_stats_.max);
      if (last_depth_stats_.median > 10.0) {
        RCLCPP_INFO(get_logger(),
                    "Depth looks like mm-scale. Consider RGBD.DepthMapFactor: 1000.0 if needed.");
      }
    }
  }

  std::string vocab_path_;
  std::string settings_path_;
  std::string rgb_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string map_frame_;
  std::string camera_frame_;
  std::string image_encoding_;
  bool publish_tf_ = true;
  bool enable_viewer_ = false;
  int sync_queue_size_ = 10;
  double sync_slop_sec_ = 0.05;
  double diagnostics_period_sec_ = 2.0;
  int depth_stats_stride_ = 8;
  bool lock_map_ = false;
  bool localization_mode_active_ = false;
  int lock_map_after_frames_ = 1;
  int tracking_ok_frames_ = 0;

  std::unique_ptr<ORB_SLAM3::System> slam_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  std::shared_ptr<Sync> sync_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  sensor_msgs::msg::CameraInfo last_camera_info_;
  std::mutex camera_info_mutex_;
  bool camera_info_logged_ = false;

  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  RollingStats rgb_fps_stats_;
  RollingStats depth_fps_stats_;
  RollingStats sync_dt_stats_;
  RollingStats track_time_stats_;

  rclcpp::Time last_rgb_stamp_;
  rclcpp::Time last_depth_stamp_;

  DepthStats last_depth_stats_;
  std::string last_depth_encoding_;
  size_t depth_stats_counter_ = 0;
};

}  // namespace orb_slam3_ros2

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<orb_slam3_ros2::OrbSlam3RgbdNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
