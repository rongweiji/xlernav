#include "calibration.hpp"

#include <depth_anything_v3/tensorrt_depth_anything.hpp>

#include <sensor_msgs/msg/camera_info.hpp>

#include <System.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
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
};

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
    } else if (arg == "-h" || arg == "--help") {
      print_usage(argv[0]);
      std::exit(0);
    } else {
      throw std::runtime_error("Unknown option: " + arg);
    }
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

  CalibrationData calib;
  if (!LoadCalibrationYaml(opt.calib_path, calib)) {
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
  if (!BuildUndistortMaps(calib, opt.undistort_balance, opt.use_projection, map1, map2, new_k)) {
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

  const std::string pipeline =
    "udpsrc port=" + std::to_string(opt.port) +
    " caps=\"application/x-rtp,media=video,encoding-name=H264,payload=96\" ! "
    "rtph264depay ! h264parse ! " + opt.decoder + " ! "
    "videoconvert ! video/x-raw,format=BGR ! "
    "appsink drop=true max-buffers=1 sync=false";

  cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
  if (!cap.isOpened()) {
    std::cerr << "Failed to open video stream. Check GStreamer pipeline.\n";
    return 1;
  }

  depth_anything_v3::TensorRTDepthAnything depth_engine(
    opt.engine_path, "fp16", tensorrt_common::BuildConfig(), false, std::string(), {1, 1, 1}, (1 << 30));

  bool preprocess_ready = false;

  sensor_msgs::msg::CameraInfo camera_info = make_camera_info(new_k, width, height);

  ORB_SLAM3::System slam(opt.vocab_path, orb_config, ORB_SLAM3::System::RGBD, opt.viewer);

  FpsCounter rx_fps(opt.fps_interval);
  FpsCounter depth_fps(opt.fps_interval);
  auto last_wait_log = std::chrono::steady_clock::now();
  const auto start_time = std::chrono::steady_clock::now();

  cv::Mat frame;
  cv::Mat rectified;

  const bool show_preview = opt.show_fps || opt.show_raw;
  if (show_preview) {
    cv::namedWindow("RGB Preview", cv::WINDOW_NORMAL);
  }

  while (g_running) {
    if (!cap.read(frame) || frame.empty()) {
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

    if (!preprocess_ready) {
      depth_engine.initPreprocessBuffer(rectified.cols, rectified.rows);
      camera_info.width = static_cast<uint32_t>(rectified.cols);
      camera_info.height = static_cast<uint32_t>(rectified.rows);
      preprocess_ready = true;
    }

    rx_fps.tick();

    std::vector<cv::Mat> images{rectified};
    if (!depth_engine.doInference(images, camera_info, 0, false)) {
      std::cerr << "Depth inference failed.\n";
      continue;
    }
    depth_fps.tick();

    const cv::Mat depth = depth_engine.getDepthImage();
    const auto now = std::chrono::steady_clock::now();
    const double timestamp = std::chrono::duration<double>(now - start_time).count();

    slam.TrackRGBD(rectified, depth, timestamp);

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
      if (cv::waitKey(1) == 'q') {
        break;
      }
    }
  }

  slam.Shutdown();
  cap.release();
  cv::destroyAllWindows();
  return 0;
}
