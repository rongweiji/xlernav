#include "camera_info.hpp"
#include "depth_estimator.hpp"
#include "fps_counter.hpp"
#include "stream_receiver.hpp"

#include <opencv2/opencv.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace fs = std::filesystem;

struct Options {
  int port = 5600;
  std::string decoder = "avdec_h264";
  std::string engine_path;
  std::string camera_info_path;
  bool show_fps = false;
  bool log_fps = false;
  double fps_interval = 1.0;
  bool log_wait = false;
  double wait_log_interval = 5.0;
  std::string colormap = "JET";
  double depth_min = std::numeric_limits<double>::quiet_NaN();
  double depth_max = std::numeric_limits<double>::quiet_NaN();
};

static std::string default_engine_path()
{
  const fs::path model_dir = fs::path(__FILE__).parent_path() / "models";

  const fs::path trt10 = model_dir / "DA3METRIC-LARGE.trt10.engine";
  const fs::path legacy = model_dir / "DA3METRIC-LARGE.fp16-batch1.engine";
  if (fs::exists(trt10)) {
    return trt10.string();
  }
  return legacy.string();
}

static std::string default_calib_path()
{
  const fs::path path = fs::path(__FILE__).parent_path() / "config" / "camera_left.yaml";
  return path.string();
}

static void print_usage(const char * prog)
{
  std::cout << "Usage: " << prog << " [options]\n"
            << "  --port N             UDP port (default 5600)\n"
            << "  --decoder NAME       GStreamer decoder (default avdec_h264)\n"
            << "  --engine PATH        TensorRT engine file\n"
            << "  --camera-info PATH   Camera calibration YAML\n"
            << "  --show-fps           Overlay FPS on the UI\n"
            << "  --log-fps            Print FPS to stdout\n"
            << "  --fps-interval SEC   FPS update interval\n"
            << "  --log-wait           Log when waiting for frames\n"
            << "  --wait-log-interval  Seconds between wait logs\n"
            << "  --colormap NAME      Depth colormap (JET, HOT, COOL, BONE, VIRIDIS, PLASMA, INFERNO, MAGMA)\n"
            << "  --depth-min VALUE    Fixed depth min for visualization\n"
            << "  --depth-max VALUE    Fixed depth max for visualization\n";
}

static Options parse_args(int argc, char ** argv)
{
  Options opt;
  opt.engine_path = default_engine_path();
  opt.camera_info_path = default_calib_path();

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
    } else if (arg == "--engine") {
      opt.engine_path = next();
    } else if (arg == "--camera-info") {
      opt.camera_info_path = next();
    } else if (arg == "--show-fps") {
      opt.show_fps = true;
    } else if (arg == "--log-fps") {
      opt.log_fps = true;
    } else if (arg == "--fps-interval") {
      opt.fps_interval = std::stod(next());
    } else if (arg == "--log-wait") {
      opt.log_wait = true;
    } else if (arg == "--wait-log-interval") {
      opt.wait_log_interval = std::stod(next());
    } else if (arg == "--colormap") {
      opt.colormap = next();
    } else if (arg == "--depth-min") {
      opt.depth_min = std::stod(next());
    } else if (arg == "--depth-max") {
      opt.depth_max = std::stod(next());
    } else if (arg == "-h" || arg == "--help") {
      print_usage(argv[0]);
      std::exit(0);
    } else {
      throw std::runtime_error("Unknown option: " + arg);
    }
  }

  return opt;
}

static int colormap_code(const std::string & name)
{
  const std::string key = name;
  if (key == "HOT") return cv::COLORMAP_HOT;
  if (key == "COOL") return cv::COLORMAP_COOL;
  if (key == "BONE") return cv::COLORMAP_BONE;
  if (key == "VIRIDIS") return cv::COLORMAP_VIRIDIS;
  if (key == "PLASMA") return cv::COLORMAP_PLASMA;
  if (key == "INFERNO") return cv::COLORMAP_INFERNO;
  if (key == "MAGMA") return cv::COLORMAP_MAGMA;
  return cv::COLORMAP_JET;
}

static cv::Mat depth_to_colormap(const cv::Mat & depth, const Options & opt)
{
  if (depth.empty()) {
    return cv::Mat();
  }

  cv::Mat mask = depth > 0.0f;
  double min_val = 0.0;
  double max_val = 0.0;

  bool has_min = std::isfinite(opt.depth_min);
  bool has_max = std::isfinite(opt.depth_max);
  if (has_min) {
    min_val = opt.depth_min;
  }
  if (has_max) {
    max_val = opt.depth_max;
  }
  if (!has_min || !has_max) {
    double auto_min = 0.0;
    double auto_max = 0.0;
    cv::minMaxLoc(depth, &auto_min, &auto_max, nullptr, nullptr, mask);
    if (!has_min) {
      min_val = auto_min;
    }
    if (!has_max) {
      max_val = auto_max;
    }
  }

  if (max_val <= min_val) {
    max_val = min_val + 1.0;
  }

  cv::Mat normalized;
  depth.convertTo(normalized, CV_32F, 1.0 / (max_val - min_val), -min_val / (max_val - min_val));
  cv::Mat clipped;
  cv::min(cv::max(normalized, 0.0f), 1.0f, clipped);

  cv::Mat gray;
  clipped.convertTo(gray, CV_8U, 255.0);

  cv::Mat colored;
  cv::applyColorMap(gray, colored, colormap_code(opt.colormap));
  return colored;
}

int main(int argc, char ** argv)
{
  Options opt;
  try {
    opt = parse_args(argc, argv);
  } catch (const std::exception & e) {
    std::cerr << e.what() << "\n";
    print_usage(argv[0]);
    return 1;
  }

  xlernav::StreamReceiver receiver(opt.port, opt.decoder);
  if (!receiver.Open()) {
    std::cerr << "Failed to open video stream. Check GStreamer pipeline.\n";
    return 1;
  }

  sensor_msgs::msg::CameraInfo cam_info;
  bool calib_loaded = xlernav::LoadCameraInfoYaml(opt.camera_info_path, cam_info);
  if (!calib_loaded) {
    std::cerr << "Warning: failed to load camera info from " << opt.camera_info_path << "\n";
  }

  xlernav::DepthEstimator depth_engine(opt.engine_path);

  xlernav::FpsCounter rx_fps(opt.fps_interval);
  xlernav::FpsCounter infer_fps(opt.fps_interval);
  auto last_wait_log = std::chrono::steady_clock::now();

  cv::namedWindow("RGB + Depth", cv::WINDOW_NORMAL);

  while (true) {
    cv::Mat frame;
    if (!receiver.Read(frame) || frame.empty()) {
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

    if (cam_info.width != static_cast<uint32_t>(frame.cols) ||
        cam_info.height != static_cast<uint32_t>(frame.rows)) {
      cam_info.width = static_cast<uint32_t>(frame.cols);
      cam_info.height = static_cast<uint32_t>(frame.rows);
      cam_info.header.frame_id = "camera";
    }

    rx_fps.tick();

    if (!depth_engine.Infer(frame, cam_info)) {
      std::cerr << "Depth inference failed.\n";
      continue;
    }
    infer_fps.tick();

    const cv::Mat depth = depth_engine.Depth();
    cv::Mat depth_vis = depth_to_colormap(depth, opt);

    if (depth_vis.empty()) {
      depth_vis = cv::Mat(frame.rows, frame.cols, CV_8UC3, cv::Scalar(0, 0, 0));
    }

    cv::Mat combined;
    cv::hconcat(frame, depth_vis, combined);

    if (opt.show_fps) {
      std::ostringstream oss;
      oss << "rx " << std::fixed << std::setprecision(1) << rx_fps.fps()
          << " fps | depth " << infer_fps.fps() << " fps";
      cv::putText(combined, oss.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }
    if (opt.log_fps && rx_fps.fps() > 0.0 && infer_fps.fps() > 0.0) {
      std::cout << "[fps] rx=" << rx_fps.fps() << " depth=" << infer_fps.fps() << "\n";
    }

    cv::imshow("RGB + Depth", combined);
    if (cv::waitKey(1) == 'q') {
      break;
    }
  }

  cv::destroyAllWindows();
  return 0;
}
