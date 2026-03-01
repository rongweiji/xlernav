#include "calibration.hpp"
#include "depth_estimator.hpp"
#include "fps_counter.hpp"
#include "rerun_stream_logger.hpp"
#include "stream_receiver.hpp"

#include <sensor_msgs/msg/camera_info.hpp>

#include <System.h>
#include <Tracking.h>

#include <opencv2/opencv.hpp>

#include <Eigen/Core>

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

namespace fs = std::filesystem;

struct Options {
  int port = 5600;
  std::string decoder = "avdec_h264";
  std::string vocab_path;
  std::string calib_path;
  std::string engine_path;
  bool log_fps = false;
  double fps_interval = 1.0;
  bool log_wait = false;
  double wait_log_interval = 5.0;
  double undistort_balance = 0.0;
  bool use_projection = true;
  double camera_fps = 30.0;
  int max_frames = 0;

  bool rerun_spawn = true;
  std::string rerun_save;
  std::string rerun_recording_id = "xlernav_orbslam3";
  std::size_t rerun_log_every_n = 3;
  bool rerun_log_images = true;
};

static std::atomic<bool> g_running{true};

static void HandleSignal(int)
{
  g_running = false;
}

static std::string default_vocab_path()
{
  return (fs::path(__FILE__).parent_path() / "vocabulary" / "ORBvoc.txt").string();
}

static std::string default_calib_path()
{
  const fs::path local = fs::path(__FILE__).parent_path() / "config" / "camera_left.yaml";
  if (fs::exists(local)) {
    return local.string();
  }
  return (fs::path(__FILE__).parent_path().parent_path() / "depth_stream_cpp" / "config" / "camera_left.yaml").string();
}

static std::string default_engine_path()
{
  const fs::path local = fs::path(__FILE__).parent_path() / "models" / "DA3METRIC-LARGE.trt10.engine";
  if (fs::exists(local)) {
    return local.string();
  }
  return (fs::path(__FILE__).parent_path().parent_path() / "depth_stream_cpp" / "models" / "DA3METRIC-LARGE.trt10.engine").string();
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
            << "  --port N                 UDP port (default 5600)\n"
            << "  --decoder NAME           GStreamer decoder (default avdec_h264)\n"
            << "  --vocab PATH             ORB-SLAM3 vocabulary file\n"
            << "  --calib PATH             Camera calibration YAML\n"
            << "  --engine PATH            TensorRT engine file\n"
            << "  --log-fps                Print FPS to stdout\n"
            << "  --fps-interval SEC       FPS update interval\n"
            << "  --log-wait               Log when waiting for frames\n"
            << "  --wait-log-interval SEC  Seconds between wait logs\n"
            << "  --balance VALUE          Undistort balance (0..1)\n"
            << "  --no-projection          Ignore projection_matrix even if present\n"
            << "  --camera-fps VALUE       Camera FPS for ORB-SLAM3 config\n"
            << "  --max-frames N           Stop after N processed frames (0=unlimited)\n"
            << "  --no-rerun-spawn         Do not auto-spawn Rerun viewer\n"
            << "  --rerun-save PATH        Save Rerun recording to .rrd\n"
            << "  --rerun-recording-id ID  Recording ID (default xlernav_orbslam3)\n"
            << "  --rerun-log-every-n N    Log images every N frames (default 3)\n"
            << "  --no-rerun-images        Disable RGB/depth image logging\n";
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
    } else if (arg == "--max-frames") {
      opt.max_frames = std::max(0, std::stoi(next()));
    } else if (arg == "--no-rerun-spawn") {
      opt.rerun_spawn = false;
    } else if (arg == "--rerun-save") {
      opt.rerun_save = next();
    } else if (arg == "--rerun-recording-id") {
      opt.rerun_recording_id = next();
    } else if (arg == "--rerun-log-every-n") {
      opt.rerun_log_every_n = std::max<std::size_t>(1, static_cast<std::size_t>(std::stoul(next())));
    } else if (arg == "--no-rerun-images") {
      opt.rerun_log_images = false;
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
  file << "ORBextractor.minThFAST: 7\n\n";
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

  xlernav::RerunOptions rerun_opt;
  rerun_opt.spawn = opt.rerun_spawn;
  rerun_opt.save_path = opt.rerun_save;
  rerun_opt.recording_id = opt.rerun_recording_id;
  rerun_opt.log_every_n = opt.rerun_log_every_n;
  rerun_opt.log_images = opt.rerun_log_images;
  rerun_opt.depth_meter = 0.001f;

  std::string rerun_warning;
  xlernav::RerunStreamLogger rerun_logger(
    rerun_opt,
    static_cast<float>(new_k.at<double>(0, 0)),
    static_cast<float>(new_k.at<double>(1, 1)),
    width,
    height,
    &rerun_warning);

  if (!rerun_warning.empty()) {
    std::cerr << "[rerun] " << rerun_warning << "\n";
  }
  if (!rerun_logger.enabled()) {
    std::cerr << "Rerun logger is not available; no visualization output will be produced.\n";
  }

  xlernav::StreamReceiver receiver(opt.port, opt.decoder);
  if (!receiver.Open()) {
    std::cerr << "Failed to open video stream. Check GStreamer pipeline.\n";
    return 1;
  }

  xlernav::DepthEstimator depth_engine(opt.engine_path);

  sensor_msgs::msg::CameraInfo camera_info = make_camera_info(new_k, width, height);

  ORB_SLAM3::System slam(opt.vocab_path, orb_config, ORB_SLAM3::System::RGBD, false);

  xlernav::FpsCounter rx_fps(opt.fps_interval);
  xlernav::FpsCounter depth_fps(opt.fps_interval);
  auto last_wait_log = std::chrono::steady_clock::now();
  const auto track_clock_start = std::chrono::steady_clock::now();

  cv::Mat frame;
  cv::Mat rectified;
  std::size_t frame_idx = 0;

  while (g_running) {
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
    const auto now_track = std::chrono::steady_clock::now();
    const double track_timestamp = std::chrono::duration<double>(now_track - track_clock_start).count();

    const Sophus::SE3f Tcw = slam.TrackRGBD(rectified, depth, track_timestamp);
    const int tracking_state = slam.GetTrackingState();
    const bool tracking_ok =
      tracking_state == ORB_SLAM3::Tracking::OK ||
      tracking_state == ORB_SLAM3::Tracking::OK_KLT;

    const bool log_this_frame = (frame_idx % std::max<std::size_t>(1, opt.rerun_log_every_n) == 0);
    if (log_this_frame && rerun_logger.enabled()) {
      const double wall_timestamp =
        std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
      rerun_logger.set_frame_time(wall_timestamp, static_cast<std::int64_t>(frame_idx));
      rerun_logger.log_rgb(rectified);
      rerun_logger.log_depth(depth);
    }

    if (tracking_ok) {
      const Sophus::SE3f Twc = Tcw.inverse();
      const Eigen::Matrix4f twc = Twc.matrix();

      if (log_this_frame && rerun_logger.enabled()) {
        rerun_logger.log_camera_pose(twc);
        rerun_logger.log_trajectory_point(Twc.translation());
      }
    }

    if (opt.log_fps && rx_fps.fps() > 0.0 && depth_fps.fps() > 0.0) {
      std::cout << "[fps] rx=" << rx_fps.fps() << " depth=" << depth_fps.fps() << "\n";
    }

    ++frame_idx;
    if (opt.max_frames > 0 && static_cast<int>(frame_idx) >= opt.max_frames) {
      std::cout << "Reached --max-frames=" << opt.max_frames << ", exiting.\n";
      break;
    }
  }

  slam.Shutdown();
  return 0;
}
