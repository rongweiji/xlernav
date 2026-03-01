#include "calibration.hpp"
#include "depth_estimator.hpp"
#include "fps_counter.hpp"
#include "rerun_stream_logger.hpp"
#include "stream_receiver.hpp"
#include "voxel_map.hpp"

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
#include <memory>
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

  bool log_fps = false;
  bool log_timing = false;
  double fps_interval = 1.0;
  bool log_wait = false;
  double wait_log_interval = 5.0;
  double undistort_balance = 0.0;
  bool use_projection = true;
  double camera_fps = 30.0;

  int grid_x = 50;
  int grid_y = 50;
  int grid_z = 30;
  float voxel_size = 0.1f;
  float max_depth = 6.0f;
  float depth_scale = 1.0f;
  int stride = 4;
  int min_score = 1;
  std::size_t max_render = 100000;
  bool rolling_grid = true;
  bool unbounded_map = true;

  int max_frames = 0;

  bool rerun_spawn = true;
  std::string rerun_save;
  std::string rerun_recording_id = "xlernav_voxel";
  std::size_t rerun_log_every_n = 3;
  bool rerun_log_images = true;
};

struct TimingStats {
  double read_ms = 0.0;
  double remap_ms = 0.0;
  double depth_ms = 0.0;
  double slam_ms = 0.0;
  double integrate_ms = 0.0;
  double snapshot_ms = 0.0;
  double loop_ms = 0.0;
  int frames = 0;
  int loops = 0;
  int map_updates = 0;
  int slam_updates = 0;
  int integrate_updates = 0;

  void reset()
  {
    *this = TimingStats{};
  }
};

static std::atomic<bool> g_running{true};

static void HandleSignal(int)
{
  g_running = false;
}

static double to_ms(const std::chrono::steady_clock::duration & duration)
{
  return std::chrono::duration<double, std::milli>(duration).count();
}

static std::string default_vocab_path()
{
  const fs::path local = fs::path(__FILE__).parent_path() / "vocabulary" / "ORBvoc.txt";
  if (fs::exists(local)) {
    return local.string();
  }
  return (fs::path(__FILE__).parent_path().parent_path() / "orbslam3_stream" / "vocabulary" / "ORBvoc.txt").string();
}

static std::string default_calib_path()
{
  const fs::path local = fs::path(__FILE__).parent_path() / "config" / "camera_left.yaml";
  if (fs::exists(local)) {
    return local.string();
  }
  const fs::path shared = fs::path(__FILE__).parent_path().parent_path() /
    "orbslam3_stream" / "config" / "camera_left.yaml";
  if (fs::exists(shared)) {
    return shared.string();
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
            << "  --log-timing             Print per-stage timing\n"
            << "  --fps-interval SEC       Logging interval\n"
            << "  --log-wait               Log when waiting for frames\n"
            << "  --wait-log-interval SEC  Seconds between wait logs\n"
            << "  --balance VALUE          Undistort balance (0..1)\n"
            << "  --no-projection          Ignore projection_matrix even if present\n"
            << "  --camera-fps VALUE       Camera FPS for ORB-SLAM3 config\n"
            << "  --grid-x N               Local grid size X (default 50)\n"
            << "  --grid-y N               Local grid size Y (default 50)\n"
            << "  --grid-z N               Local grid size Z (default 30)\n"
            << "  --voxel-size M           Voxel size meters (default 0.1)\n"
            << "  --max-depth M            Max depth meters (default 6.0)\n"
            << "  --depth-scale S          Depth scale multiplier (default 1.0)\n"
            << "  --stride N               Pixel stride for integration (default 4)\n"
            << "  --min-score N            Min occupancy score to render (default 1)\n"
            << "  --max-render N           Max voxels rendered (default 100000)\n"
            << "  --unbounded-map          Enable global map (default)\n"
            << "  --bounded-map            Disable global map (local only)\n"
            << "  --fixed-grid             Keep local grid fixed at first pose\n"
            << "  --rolling-grid           Keep local grid centered (default)\n"
            << "  --max-frames N           Stop after N processed frames (0=unlimited)\n"
            << "  --no-rerun-spawn         Do not auto-spawn Rerun viewer\n"
            << "  --rerun-save PATH        Save Rerun recording to .rrd\n"
            << "  --rerun-recording-id ID  Recording ID (default xlernav_voxel)\n"
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
    } else if (arg == "--log-timing") {
      opt.log_timing = true;
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
    } else if (arg == "--grid-x") {
      opt.grid_x = std::max(1, std::stoi(next()));
    } else if (arg == "--grid-y") {
      opt.grid_y = std::max(1, std::stoi(next()));
    } else if (arg == "--grid-z") {
      opt.grid_z = std::max(1, std::stoi(next()));
    } else if (arg == "--voxel-size") {
      opt.voxel_size = std::max(0.01f, std::stof(next()));
    } else if (arg == "--max-depth") {
      opt.max_depth = std::max(0.1f, std::stof(next()));
    } else if (arg == "--depth-scale") {
      opt.depth_scale = std::max(0.0001f, std::stof(next()));
    } else if (arg == "--stride") {
      opt.stride = std::max(1, std::stoi(next()));
    } else if (arg == "--min-score") {
      opt.min_score = std::stoi(next());
    } else if (arg == "--max-render") {
      opt.max_render = std::max<std::size_t>(1, static_cast<std::size_t>(std::stoull(next())));
    } else if (arg == "--unbounded-map") {
      opt.unbounded_map = true;
    } else if (arg == "--bounded-map") {
      opt.unbounded_map = false;
    } else if (arg == "--fixed-grid") {
      opt.rolling_grid = false;
    } else if (arg == "--rolling-grid") {
      opt.rolling_grid = true;
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

static void voxel_points_to_arrays(
  const std::vector<xlernav::VoxelPoint> & voxels,
  std::vector<Eigen::Vector3f> & points,
  std::vector<Eigen::Vector3f> & colors)
{
  points.clear();
  colors.clear();
  points.reserve(voxels.size());
  colors.reserve(voxels.size());
  for (const auto & v : voxels) {
    points.push_back(v.center);
    colors.push_back(v.color);
  }
}

static void esdf_points_to_arrays(
  const std::vector<xlernav::VoxelPoint> & esdf,
  std::vector<Eigen::Vector3f> & points,
  std::vector<Eigen::Vector3f> & colors)
{
  points.clear();
  colors.clear();
  points.reserve(esdf.size());
  colors.reserve(esdf.size());
  for (const auto & p : esdf) {
    points.push_back(p.center);
    const float t = std::clamp((static_cast<float>(p.score) + 10.0f) / 20.0f, 0.0f, 1.0f);
    colors.emplace_back(255.0f * (1.0f - t), 255.0f * t, 60.0f);
  }
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
            << " (balance=" << opt.undistort_balance << ")\n"
            << "Voxel size: " << opt.voxel_size << " m\n"
            << "Global map: " << (opt.unbounded_map ? "enabled\n" : "disabled\n")
            << "Local grid: " << opt.grid_x << " x " << opt.grid_y << " x " << opt.grid_z
            << (opt.rolling_grid ? " (rolling)\n" : " (fixed)\n");

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

  xlernav::VoxelMap local_map(
    opt.voxel_size,
    opt.grid_x,
    opt.grid_y,
    opt.grid_z,
    opt.rolling_grid,
    false);
  std::unique_ptr<xlernav::VoxelMap> global_map;
  if (opt.unbounded_map) {
    global_map = std::make_unique<xlernav::VoxelMap>(
      opt.voxel_size,
      opt.grid_x,
      opt.grid_y,
      opt.grid_z,
      false,
      true);
  }

  cv::Mat frame;
  cv::Mat rectified;

  std::vector<Eigen::Vector3f> rr_points;
  std::vector<Eigen::Vector3f> rr_colors;
  std::vector<Eigen::Vector3f> rr_esdf_points;
  std::vector<Eigen::Vector3f> rr_esdf_colors;

  const auto map_interval = std::chrono::milliseconds(100);
  auto last_map_update = std::chrono::steady_clock::now();

  TimingStats timing;
  auto last_timing_log = std::chrono::steady_clock::now();
  const auto timing_interval = std::chrono::duration<double>(std::max(0.2, opt.fps_interval));

  std::size_t frame_idx = 0;

  while (g_running) {
    const auto loop_start = std::chrono::steady_clock::now();

    const auto read_start = loop_start;
    const bool got_frame = receiver.Read(frame) && !frame.empty();
    const auto read_end = std::chrono::steady_clock::now();

    if (opt.log_timing) {
      timing.read_ms += to_ms(read_end - read_start);
      timing.loops += 1;
    }

    if (!got_frame) {
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

    if (opt.log_timing) {
      timing.frames += 1;
    }

    const auto remap_start = std::chrono::steady_clock::now();
    cv::remap(frame, rectified, map1, map2, cv::INTER_LINEAR);
    if (opt.log_timing) {
      timing.remap_ms += to_ms(std::chrono::steady_clock::now() - remap_start);
    }

    if (camera_info.width != static_cast<uint32_t>(rectified.cols) ||
        camera_info.height != static_cast<uint32_t>(rectified.rows)) {
      camera_info.width = static_cast<uint32_t>(rectified.cols);
      camera_info.height = static_cast<uint32_t>(rectified.rows);
    }

    rx_fps.tick();

    const auto depth_start = std::chrono::steady_clock::now();
    const bool depth_ok = depth_engine.Infer(rectified, camera_info);
    if (opt.log_timing) {
      timing.depth_ms += to_ms(std::chrono::steady_clock::now() - depth_start);
    }
    if (!depth_ok) {
      std::cerr << "Depth inference failed.\n";
      continue;
    }

    depth_fps.tick();

    const cv::Mat depth = depth_engine.Depth();
    const auto now_track = std::chrono::steady_clock::now();
    const double track_timestamp = std::chrono::duration<double>(now_track - track_clock_start).count();

    const auto slam_start = std::chrono::steady_clock::now();
    const Sophus::SE3f Tcw = slam.TrackRGBD(rectified, depth, track_timestamp);
    if (opt.log_timing) {
      timing.slam_ms += to_ms(std::chrono::steady_clock::now() - slam_start);
      timing.slam_updates += 1;
    }

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

      const auto integrate_start = std::chrono::steady_clock::now();
      local_map.Integrate(
        depth,
        rectified,
        new_k,
        Twc.matrix(),
        opt.stride,
        opt.max_depth,
        opt.depth_scale);
      if (opt.log_timing) {
        timing.integrate_ms += to_ms(std::chrono::steady_clock::now() - integrate_start);
        timing.integrate_updates += 1;
      }

      if (log_this_frame && rerun_logger.enabled()) {
        rerun_logger.log_camera_pose(Twc.matrix());
        rerun_logger.log_trajectory_point(Twc.translation());
      }

      const auto now = std::chrono::steady_clock::now();
      if (now - last_map_update >= map_interval) {
        const auto snapshot_start = std::chrono::steady_clock::now();

        std::vector<xlernav::VoxelPoint> local_voxels = local_map.Snapshot(opt.min_score);
        constexpr float kEsdfMaxDistance = 2.0f;
        std::vector<xlernav::VoxelPoint> esdf_points = local_map.EsdfPoints2D(opt.min_score, kEsdfMaxDistance);
        if (global_map) {
          global_map->IntegratePoints(local_voxels);
        }

        std::vector<xlernav::VoxelPoint> voxels = global_map ?
          global_map->Snapshot(opt.min_score) : std::move(local_voxels);

        if (opt.max_render > 0 && voxels.size() > opt.max_render) {
          std::vector<xlernav::VoxelPoint> sampled;
          sampled.reserve(opt.max_render);
          const std::size_t stride = std::max<std::size_t>(1, voxels.size() / opt.max_render);
          for (std::size_t i = 0; i < voxels.size(); i += stride) {
            sampled.push_back(voxels[i]);
          }
          voxels.swap(sampled);
        }

        voxel_points_to_arrays(voxels, rr_points, rr_colors);
        esdf_points_to_arrays(esdf_points, rr_esdf_points, rr_esdf_colors);

        if (rerun_logger.enabled()) {
          rerun_logger.log_points("world/map/voxels", rr_points, &rr_colors);
          rerun_logger.log_points("world/map/esdf", rr_esdf_points, &rr_esdf_colors);
        }

        last_map_update = now;

        if (opt.log_timing) {
          timing.snapshot_ms += to_ms(std::chrono::steady_clock::now() - snapshot_start);
          timing.map_updates += 1;
        }
      }
    }

    if (opt.log_fps && rx_fps.fps() > 0.0 && depth_fps.fps() > 0.0) {
      std::cout << "[fps] rx=" << rx_fps.fps() << " depth=" << depth_fps.fps() << "\n";
    }

    if (opt.log_timing) {
      timing.loop_ms += to_ms(std::chrono::steady_clock::now() - loop_start);
      const auto now = std::chrono::steady_clock::now();
      if (now - last_timing_log >= timing_interval) {
        const double elapsed = std::chrono::duration<double>(now - last_timing_log).count();
        const auto avg = [](double total, int count) -> double {
          return count > 0 ? total / static_cast<double>(count) : 0.0;
        };
        const double fps = timing.frames > 0 ? timing.frames / elapsed : 0.0;
        std::cout << std::fixed << std::setprecision(2)
                  << "[timing] fps=" << fps
                  << " read=" << avg(timing.read_ms, timing.loops)
                  << " remap=" << avg(timing.remap_ms, timing.frames)
                  << " depth=" << avg(timing.depth_ms, timing.frames)
                  << " slam=" << avg(timing.slam_ms, timing.slam_updates)
                  << " integ=" << avg(timing.integrate_ms, timing.integrate_updates)
                  << " snap=" << avg(timing.snapshot_ms, timing.map_updates)
                  << " loop=" << avg(timing.loop_ms, timing.loops)
                  << " (frames=" << timing.frames
                  << " map=" << timing.map_updates
                  << " slam=" << timing.slam_updates
                  << " integ=" << timing.integrate_updates << ")\n";
        timing.reset();
        last_timing_log = now;
      }
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
