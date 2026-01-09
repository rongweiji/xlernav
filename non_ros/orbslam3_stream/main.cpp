#include "calibration.hpp"
#include "depth_estimator.hpp"
#include "fps_counter.hpp"
#include "stream_receiver.hpp"

#include <sensor_msgs/msg/camera_info.hpp>

#include <System.h>
#include <Tracking.h>

#include <opencv2/opencv.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

struct Options {
  int port = 5600;
  std::string decoder = "avdec_h264";
  std::string vocab_path;
  std::string calib_path;
  std::string engine_path;
  bool show_fps = false;
  bool show_raw = false;
  bool log_fps = false;
  double fps_interval = 1.0;
  bool log_wait = false;
  double wait_log_interval = 5.0;
  double undistort_balance = 0.0;
  bool use_projection = true;
  double camera_fps = 30.0;
  bool viewer = true;
  bool viewer_forced_off = false;
  enum class UiMode { kOrbslam, kCustom, kBoth } ui_mode = UiMode::kOrbslam;
};

struct PoseViewState {
  float yaw_deg = 0.0f;
  float pitch_deg = -15.0f;
  float distance = 2.5f;
  bool dragging = false;
  cv::Point last_pos{0, 0};
  cv::Rect pose_rect;
};

static std::atomic<bool> g_running{true};

static void HandleSignal(int)
{
  g_running = false;
}

static std::string default_vocab_path()
{
  const fs::path path = fs::path(__FILE__).parent_path() / "vocabulary" / "ORBvoc.txt";
  return path.string();
}

static std::string default_calib_path()
{
  const fs::path local = fs::path(__FILE__).parent_path() / "config" / "camera_left.yaml";
  if (fs::exists(local)) {
    return local.string();
  }
  const fs::path shared = fs::path(__FILE__).parent_path().parent_path() / "depth_stream_cpp" / "config" / "camera_left.yaml";
  return shared.string();
}

static std::string default_engine_path()
{
  const fs::path local = fs::path(__FILE__).parent_path() / "models" / "DA3METRIC-LARGE.trt10.engine";
  if (fs::exists(local)) {
    return local.string();
  }
  const fs::path shared = fs::path(__FILE__).parent_path().parent_path() / "depth_stream_cpp" / "models" / "DA3METRIC-LARGE.trt10.engine";
  return shared.string();
}

static std::string runtime_config_path()
{
  const char * home = std::getenv("HOME");
  const fs::path base = home ? fs::path(home) : fs::temp_directory_path();
  const fs::path dir = base / ".cache" / "xlernav";
  std::error_code ec;
  fs::create_directories(dir, ec);
  return (dir / "orbslam3_runtime.yaml").string();
}

static void print_usage(const char * prog)
{
  std::cout << "Usage: " << prog << " [options]\n"
            << "  --port N             UDP port (default 5600)\n"
            << "  --decoder NAME       GStreamer decoder (default avdec_h264)\n"
            << "  --vocab PATH         ORB-SLAM3 vocabulary file\n"
            << "  --calib PATH         Camera calibration YAML\n"
            << "  --engine PATH        TensorRT engine file\n"
            << "  --ui MODE            UI mode: orbslam, custom, both (default orbslam)\n"
            << "  --show-fps           Overlay FPS on the UI (RGB window)\n"
            << "  --show-raw           Show raw vs undistorted side-by-side\n"
            << "  --log-fps            Print FPS to stdout\n"
            << "  --fps-interval SEC   FPS update interval\n"
            << "  --log-wait           Log when waiting for frames\n"
            << "  --wait-log-interval  Seconds between wait logs\n"
            << "  --balance VALUE      Undistort balance (0..1)\n"
            << "  --no-projection      Ignore projection_matrix even if present\n"
            << "  --camera-fps VALUE   Camera FPS for ORB-SLAM3 config\n"
            << "  --no-viewer          Disable Pangolin viewer\n";
}

static Options parse_args(int argc, char ** argv)
{
  Options opt;
  opt.vocab_path = default_vocab_path();
  opt.calib_path = default_calib_path();
  opt.engine_path = default_engine_path();

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for " + arg);
      }
      return argv[++i];
    };

    if (arg == "--port") {
      opt.port = std::stoi(next());
    } else if (arg == "--decoder") {
      opt.decoder = next();
    } else if (arg == "--vocab") {
      opt.vocab_path = next();
    } else if (arg == "--calib") {
      opt.calib_path = next();
    } else if (arg == "--engine") {
      opt.engine_path = next();
    } else if (arg == "--ui") {
      const std::string mode = next();
      if (mode == "orbslam") {
        opt.ui_mode = Options::UiMode::kOrbslam;
      } else if (mode == "custom") {
        opt.ui_mode = Options::UiMode::kCustom;
      } else if (mode == "both") {
        opt.ui_mode = Options::UiMode::kBoth;
      } else {
        throw std::runtime_error("Unknown UI mode: " + mode);
      }
    } else if (arg == "--show-fps") {
      opt.show_fps = true;
    } else if (arg == "--show-raw") {
      opt.show_raw = true;
    } else if (arg == "--log-fps") {
      opt.log_fps = true;
    } else if (arg == "--fps-interval") {
      opt.fps_interval = std::stod(next());
    } else if (arg == "--log-wait") {
      opt.log_wait = true;
    } else if (arg == "--wait-log-interval") {
      opt.wait_log_interval = std::stod(next());
    } else if (arg == "--balance") {
      opt.undistort_balance = std::stod(next());
    } else if (arg == "--no-projection") {
      opt.use_projection = false;
    } else if (arg == "--camera-fps") {
      opt.camera_fps = std::stod(next());
    } else if (arg == "--no-viewer") {
      opt.viewer = false;
      opt.viewer_forced_off = true;
    } else if (arg == "-h" || arg == "--help") {
      print_usage(argv[0]);
      std::exit(0);
    } else {
      throw std::runtime_error("Unknown option: " + arg);
    }
  }

  if (opt.ui_mode == Options::UiMode::kCustom) {
    opt.viewer = false;
  } else if (opt.ui_mode == Options::UiMode::kBoth) {
    opt.viewer = true;
  }
  if (opt.viewer_forced_off) {
    opt.viewer = false;
  }

  return opt;
}

static sensor_msgs::msg::CameraInfo make_camera_info(const cv::Mat & k, int width, int height)
{
  sensor_msgs::msg::CameraInfo info;
  info.width = static_cast<uint32_t>(width);
  info.height = static_cast<uint32_t>(height);
  info.k = {k.at<double>(0, 0), k.at<double>(0, 1), k.at<double>(0, 2),
            k.at<double>(1, 0), k.at<double>(1, 1), k.at<double>(1, 2),
            k.at<double>(2, 0), k.at<double>(2, 1), k.at<double>(2, 2)};
  info.d.assign(5, 0.0);
  info.distortion_model = "plumb_bob";
  return info;
}

static bool write_orbslam_config(
  const std::string & path,
  const cv::Mat & k,
  int width,
  int height,
  double fps)
{
  std::ofstream file(path);
  if (!file.is_open()) {
    return false;
  }

  file << "%YAML:1.0\n\n";
  file << "File.version: \"1.0\"\n\n";
  file << "Camera.type: \"PinHole\"\n\n";
  file << "Camera1.fx: " << k.at<double>(0, 0) << "\n";
  file << "Camera1.fy: " << k.at<double>(1, 1) << "\n";
  file << "Camera1.cx: " << k.at<double>(0, 2) << "\n";
  file << "Camera1.cy: " << k.at<double>(1, 2) << "\n\n";
  file << "Camera1.k1: 0.0\n";
  file << "Camera1.k2: 0.0\n";
  file << "Camera1.p1: 0.0\n";
  file << "Camera1.p2: 0.0\n";
  file << "Camera1.k3: 0.0\n\n";
  file << "Camera.width: " << width << "\n";
  file << "Camera.height: " << height << "\n\n";
  file << "Camera.newWidth: " << width << "\n";
  file << "Camera.newHeight: " << height << "\n\n";
  file << "Camera.fps: " << fps << "\n";
  file << "Camera.RGB: 0\n\n";
  file << "Stereo.ThDepth: 40.0\n";
  file << "Stereo.b: 0.07732\n\n";
  file << "RGBD.DepthMapFactor: 1.0\n\n";
  file << "ORBextractor.nFeatures: 1000\n";
  file << "ORBextractor.scaleFactor: 1.2\n";
  file << "ORBextractor.nLevels: 8\n";
  file << "ORBextractor.iniThFAST: 20\n";
  file << "ORBextractor.minThFAST: 7\n";
  file << "\n";
  file << "Viewer.KeyFrameSize: 0.05\n";
  file << "Viewer.KeyFrameLineWidth: 1.0\n";
  file << "Viewer.GraphLineWidth: 0.9\n";
  file << "Viewer.PointSize: 2.0\n";
  file << "Viewer.CameraSize: 0.08\n";
  file << "Viewer.CameraLineWidth: 3.0\n";
  file << "Viewer.ViewpointX: 0.0\n";
  file << "Viewer.ViewpointY: -0.7\n";
  file << "Viewer.ViewpointZ: -1.8\n";
  file << "Viewer.ViewpointF: 500.0\n";
  return true;
}

static cv::Mat colorize_depth(const cv::Mat & depth)
{
  if (depth.empty()) {
    return {};
  }

  cv::Mat depth_float;
  if (depth.type() != CV_32F) {
    depth.convertTo(depth_float, CV_32F);
  } else {
    depth_float = depth;
  }

  cv::Mat mask = depth_float > 0.0f;
  double min_val = 0.0;
  double max_val = 0.0;
  cv::minMaxLoc(depth_float, &min_val, &max_val, nullptr, nullptr, mask);
  if (max_val <= min_val) {
    return cv::Mat(depth.size(), CV_8UC3, cv::Scalar(0, 0, 0));
  }

  cv::Mat normalized;
  depth_float.convertTo(
    normalized, CV_8U,
    255.0 / (max_val - min_val),
    -min_val * 255.0 / (max_val - min_val));

  cv::Mat colored;
  cv::applyColorMap(normalized, colored, cv::COLORMAP_MAGMA);
  colored.setTo(cv::Scalar(0, 0, 0), ~mask);
  return colored;
}

static void update_view_rotation(PoseViewState & view, const cv::Point & delta)
{
  view.yaw_deg += static_cast<float>(delta.x) * 0.5f;
  view.pitch_deg += static_cast<float>(delta.y) * 0.5f;
  view.pitch_deg = std::clamp(view.pitch_deg, -89.0f, 89.0f);
}

static void update_view_zoom(PoseViewState & view, int wheel_delta)
{
  if (wheel_delta == 0) {
    return;
  }
  const float scale = 1.0f - static_cast<float>(wheel_delta) / 1200.0f;
  view.distance = std::clamp(view.distance * scale, 0.3f, 20.0f);
}

static void on_pose_mouse(int event, int x, int y, int flags, void * userdata)
{
  auto * view = static_cast<PoseViewState *>(userdata);
  if (!view) {
    return;
  }

  const cv::Point pt(x, y);
  const bool inside = view->pose_rect.contains(pt);

  if (event == cv::EVENT_LBUTTONDOWN) {
    if (inside) {
      view->dragging = true;
      view->last_pos = pt;
    }
  } else if (event == cv::EVENT_LBUTTONUP) {
    view->dragging = false;
  } else if (event == cv::EVENT_MOUSEMOVE) {
    if (view->dragging) {
      const cv::Point delta = pt - view->last_pos;
      view->last_pos = pt;
      update_view_rotation(*view, delta);
    }
  } else if (event == cv::EVENT_MOUSEWHEEL) {
    if (inside) {
      const int wheel = cv::getMouseWheelDelta(flags);
      update_view_zoom(*view, wheel);
    }
  }
}

static cv::Mat render_pose_panel(
  const std::vector<Eigen::Vector3f> & trajectory,
  const Sophus::SE3f & twc,
  bool tracking_ok,
  const PoseViewState & view,
  const cv::Size & size)
{
  cv::Mat panel(size, CV_8UC3, cv::Scalar(18, 18, 18));

  const float yaw = view.yaw_deg * 3.14159265f / 180.0f;
  const float pitch = view.pitch_deg * 3.14159265f / 180.0f;

  Eigen::Matrix3f Ry;
  Ry << std::cos(yaw), 0.0f, std::sin(yaw),
    0.0f, 1.0f, 0.0f,
    -std::sin(yaw), 0.0f, std::cos(yaw);

  Eigen::Matrix3f Rx;
  Rx << 1.0f, 0.0f, 0.0f,
    0.0f, std::cos(pitch), -std::sin(pitch),
    0.0f, std::sin(pitch), std::cos(pitch);

  const Eigen::Matrix3f Rview = Ry * Rx;
  const float focal = 0.8f * static_cast<float>(std::min(size.width, size.height));
  const cv::Point2f center(size.width * 0.5f, size.height * 0.5f);

  auto project = [&](const Eigen::Vector3f & p, cv::Point & out) -> bool {
    Eigen::Vector3f pv = Rview * p;
    pv.z() += view.distance;
    if (pv.z() < 0.05f) {
      return false;
    }
    const float x = pv.x() * focal / pv.z();
    const float y = pv.y() * focal / pv.z();
    out = cv::Point(static_cast<int>(std::lround(center.x + x)),
                    static_cast<int>(std::lround(center.y - y)));
    return true;
  };

  auto draw_line = [&](const Eigen::Vector3f & a, const Eigen::Vector3f & b, const cv::Scalar & color, int thickness) {
    cv::Point pa;
    cv::Point pb;
    if (!project(a, pa) || !project(b, pb)) {
      return;
    }
    cv::line(panel, pa, pb, color, thickness, cv::LINE_AA);
  };

  const float axis_scale = 0.2f;
  draw_line(Eigen::Vector3f::Zero(), Eigen::Vector3f(axis_scale, 0.0f, 0.0f), cv::Scalar(60, 60, 255), 2);
  draw_line(Eigen::Vector3f::Zero(), Eigen::Vector3f(0.0f, axis_scale, 0.0f), cv::Scalar(60, 255, 60), 2);
  draw_line(Eigen::Vector3f::Zero(), Eigen::Vector3f(0.0f, 0.0f, axis_scale), cv::Scalar(255, 140, 60), 2);

  for (size_t i = 1; i < trajectory.size(); ++i) {
    draw_line(trajectory[i - 1], trajectory[i], cv::Scalar(60, 220, 60), 2);
  }

  if (tracking_ok) {
    const Eigen::Matrix3f Rpose = twc.rotationMatrix();
    const Eigen::Vector3f tpose = twc.translation();
    const float pose_scale = 0.1f;
    draw_line(tpose, tpose + Rpose * Eigen::Vector3f(pose_scale, 0.0f, 0.0f), cv::Scalar(80, 80, 255), 2);
    draw_line(tpose, tpose + Rpose * Eigen::Vector3f(0.0f, pose_scale, 0.0f), cv::Scalar(80, 255, 80), 2);
    draw_line(tpose, tpose + Rpose * Eigen::Vector3f(0.0f, 0.0f, pose_scale), cv::Scalar(255, 160, 80), 2);
  }

  return panel;
}

int main(int argc, char ** argv)
{
  std::signal(SIGINT, HandleSignal);

  Options opt;
  try {
    opt = parse_args(argc, argv);
  } catch (const std::exception & e) {
    std::cerr << e.what() << "\n";
    print_usage(argv[0]);
    return 1;
  }

  xlernav::CalibrationData calib;
  if (!xlernav::LoadCalibrationYaml(opt.calib_path, calib)) {
    std::cerr << "Failed to load calibration from " << opt.calib_path << "\n";
    return 1;
  }

  if (!fs::exists(opt.engine_path)) {
    std::cerr << "TensorRT engine not found: " << opt.engine_path << "\n";
    return 1;
  }
  if (!fs::exists(opt.vocab_path)) {
    std::cerr << "ORB vocabulary not found: " << opt.vocab_path << "\n";
    return 1;
  }

  cv::Mat map1, map2, new_k;
  if (!xlernav::BuildUndistortMaps(calib, opt.undistort_balance, opt.use_projection, map1, map2, new_k)) {
    std::cerr << "Failed to build undistort maps.\n";
    return 1;
  }

  const int width = calib.width > 0 ? calib.width : 640;
  const int height = calib.height > 0 ? calib.height : 480;

  const std::string orb_config = runtime_config_path();
  if (!write_orbslam_config(orb_config, new_k, width, height, opt.camera_fps)) {
    std::cerr << "Failed to write ORB-SLAM3 config.\n";
    return 1;
  }
  std::cout << "ORB-SLAM3 config: " << orb_config << "\n"
            << "Vocabulary: " << opt.vocab_path << "\n"
            << "Calibration: " << opt.calib_path << "\n"
            << "Engine: " << opt.engine_path << "\n"
            << "Undistort: " << (opt.use_projection ? "projection_matrix" : "intrinsics")
            << " (balance=" << opt.undistort_balance << ")\n";

  xlernav::StreamReceiver receiver(opt.port, opt.decoder);
  if (!receiver.Open()) {
    std::cerr << "Failed to open video stream. Check GStreamer pipeline.\n";
    return 1;
  }

  xlernav::DepthEstimator depth_engine(opt.engine_path);

  sensor_msgs::msg::CameraInfo camera_info = make_camera_info(new_k, width, height);

  ORB_SLAM3::System slam(opt.vocab_path, orb_config, ORB_SLAM3::System::RGBD, opt.viewer);

  xlernav::FpsCounter rx_fps(opt.fps_interval);
  xlernav::FpsCounter depth_fps(opt.fps_interval);
  auto last_wait_log = std::chrono::steady_clock::now();
  const auto start_time = std::chrono::steady_clock::now();

  cv::Mat frame;
  cv::Mat rectified;

  const bool show_custom_ui = opt.ui_mode != Options::UiMode::kOrbslam;
  const bool show_preview = (!show_custom_ui) && (opt.show_fps || opt.show_raw);
  if (show_preview) {
    cv::namedWindow("RGB Preview", cv::WINDOW_NORMAL);
  }

  Sophus::SE3f last_Twc;
  std::vector<Eigen::Vector3f> trajectory;
  PoseViewState pose_view;
  bool custom_ui_inited = false;

  while (g_running) {
    if (!receiver.Read(frame) || frame.empty()) {
      if (show_custom_ui && custom_ui_inited) {
        if (cv::waitKey(1) == 'q') {
          break;
        }
      }
      if (opt.log_wait) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - last_wait_log).count();
        if (elapsed >= opt.wait_log_interval) {
          std::cout << "[stream] waiting for frames...\n";
          last_wait_log = now;
        }
      }
      continue;
    }

    cv::remap(frame, rectified, map1, map2, cv::INTER_LINEAR);

    if (camera_info.width != static_cast<uint32_t>(rectified.cols) ||
        camera_info.height != static_cast<uint32_t>(rectified.rows)) {
      camera_info.width = static_cast<uint32_t>(rectified.cols);
      camera_info.height = static_cast<uint32_t>(rectified.rows);
    }

    rx_fps.tick();

    if (!depth_engine.Infer(rectified, camera_info)) {
      std::cerr << "Depth inference failed.\n";
      continue;
    }
    depth_fps.tick();

    const cv::Mat depth = depth_engine.Depth();
    const auto now = std::chrono::steady_clock::now();
    const double timestamp = std::chrono::duration<double>(now - start_time).count();

    const Sophus::SE3f Tcw = slam.TrackRGBD(rectified, depth, timestamp);
    const int tracking_state = slam.GetTrackingState();
    const bool tracking_ok =
      tracking_state == ORB_SLAM3::Tracking::OK ||
      tracking_state == ORB_SLAM3::Tracking::OK_KLT;
    if (tracking_ok) {
      const Sophus::SE3f Twc = Tcw.inverse();
      last_Twc = Twc;
      trajectory.push_back(Twc.translation());
    }

    if (opt.log_fps && rx_fps.fps() > 0.0 && depth_fps.fps() > 0.0) {
      std::cout << "[fps] rx=" << rx_fps.fps() << " depth=" << depth_fps.fps() << "\n";
    }

    if (show_preview) {
      cv::Mat display;
      if (opt.show_raw) {
        cv::Mat raw;
        if (frame.size() != rectified.size()) {
          cv::resize(frame, raw, rectified.size(), 0, 0, cv::INTER_LINEAR);
        } else {
          raw = frame;
        }
        cv::hconcat(raw, rectified, display);
        cv::putText(display, "RAW", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
        cv::putText(display, "RECT", cv::Point(rectified.cols + 10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
      } else {
        display = rectified;
      }

      if (opt.show_fps) {
        std::ostringstream oss;
        oss << "rx " << std::fixed << std::setprecision(1) << rx_fps.fps()
            << " fps | depth " << depth_fps.fps() << " fps";
        cv::putText(display, oss.str(), cv::Point(10, display.rows - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
      }

      cv::imshow("RGB Preview", display);
    }

    if (show_custom_ui) {
      if (!custom_ui_inited) {
        cv::namedWindow("Custom UI", cv::WINDOW_NORMAL);
        cv::setMouseCallback("Custom UI", on_pose_mouse, &pose_view);
        custom_ui_inited = true;
      }

      const cv::Mat rgb_view = opt.show_raw ? frame : rectified;
      cv::Mat rgb_vis;
      if (rgb_view.size() != rectified.size()) {
        cv::resize(rgb_view, rgb_vis, rectified.size(), 0, 0, cv::INTER_LINEAR);
      } else {
        rgb_vis = rgb_view;
      }

      cv::Mat depth_vis = colorize_depth(depth);
      if (depth_vis.empty()) {
        depth_vis = cv::Mat(rectified.size(), CV_8UC3, cv::Scalar(0, 0, 0));
      } else if (depth_vis.size() != rectified.size()) {
        cv::resize(depth_vis, depth_vis, rectified.size(), 0, 0, cv::INTER_NEAREST);
      }

      cv::Mat top;
      cv::hconcat(rgb_vis, depth_vis, top);
      const int pose_height = std::max(200, rectified.rows / 2);
      pose_view.pose_rect = cv::Rect(0, top.rows, top.cols, pose_height);
      cv::Mat pose_panel = render_pose_panel(
        trajectory, last_Twc, tracking_ok, pose_view, cv::Size(top.cols, pose_height));

      std::ostringstream pose_text;
      if (tracking_ok) {
        const Eigen::Vector3f pos = last_Twc.translation();
        pose_text << "Pose (m): x=" << std::fixed << std::setprecision(2) << pos.x()
                  << " y=" << pos.y() << " z=" << pos.z();
      } else {
        pose_text << "Pose: tracking lost";
      }
      cv::putText(pose_panel, pose_text.str(), cv::Point(10, pose_panel.rows - 12),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

      if (opt.show_fps) {
        std::ostringstream fps_text;
        fps_text << "rx " << std::fixed << std::setprecision(1) << rx_fps.fps()
                 << " fps | depth " << depth_fps.fps() << " fps";
        cv::putText(pose_panel, fps_text.str(), cv::Point(10, 28),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
      }

      cv::Mat canvas(top.rows + pose_height, top.cols, CV_8UC3, cv::Scalar(0, 0, 0));
      top.copyTo(canvas(cv::Rect(0, 0, top.cols, top.rows)));
      pose_panel.copyTo(canvas(cv::Rect(0, top.rows, top.cols, pose_height)));

      const std::string rgb_label = opt.show_raw ? "RGB (raw)" : "RGB (rect)";
      cv::putText(canvas, rgb_label, cv::Point(10, 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
      cv::putText(canvas, "Depth", cv::Point(rectified.cols + 10, 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

      cv::imshow("Custom UI", canvas);
    }

    if (show_preview || show_custom_ui) {
      if (cv::waitKey(1) == 'q') {
        break;
      }
    }
  }

  slam.Shutdown();
  cv::destroyAllWindows();
  return 0;
}
