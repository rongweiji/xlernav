#include "calibration.hpp"
#include "depth_estimator.hpp"
#include "fps_counter.hpp"
#include "stream_receiver.hpp"
#include "voxel_map.hpp"

#include <sensor_msgs/msg/camera_info.hpp>

#include <System.h>
#include <Tracking.h>

#include <pangolin/pangolin.h>

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
#include <vector>

namespace fs = std::filesystem;

struct Options {
  int port = 5600;
  std::string decoder = "avdec_h264";
  std::string vocab_path;
  std::string calib_path;
  std::string engine_path;
  bool show_fps = false;
  bool log_fps = false;
  double fps_interval = 1.0;
  bool log_wait = false;
  double wait_log_interval = 5.0;
  double undistort_balance = 0.0;
  bool use_projection = true;
  double camera_fps = 30.0;
  bool show_preview = false;
  bool show_raw = false;
  float voxel_size = 0.1f;
  float max_depth = 6.0f;
  float depth_scale = 1.0f;
  int stride = 4;
  int min_score = 1;
  double decay_sec = 120.0;
  std::size_t max_voxels = 200000;
  std::size_t max_render = 100000;
};

static std::atomic<bool> g_running{true};

static void HandleSignal(int)
{
  g_running = false;
}

static std::string default_vocab_path()
{
  const fs::path local = fs::path(__FILE__).parent_path() / "vocabulary" / "ORBvoc.txt";
  if (fs::exists(local)) {
    return local.string();
  }
  const fs::path shared = fs::path(__FILE__).parent_path().parent_path() /
    "orbslam3_stream" / "vocabulary" / "ORBvoc.txt";
  return shared.string();
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
  const fs::path fallback = fs::path(__FILE__).parent_path().parent_path() /
    "depth_stream_cpp" / "config" / "camera_left.yaml";
  return fallback.string();
}

static std::string default_engine_path()
{
  const fs::path local = fs::path(__FILE__).parent_path() / "models" / "DA3METRIC-LARGE.trt10.engine";
  if (fs::exists(local)) {
    return local.string();
  }
  const fs::path shared = fs::path(__FILE__).parent_path().parent_path() /
    "depth_stream_cpp" / "models" / "DA3METRIC-LARGE.trt10.engine";
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
            << "  --show-fps           Overlay FPS on the preview UI\n"
            << "  --log-fps            Print FPS to stdout\n"
            << "  --fps-interval SEC   FPS update interval\n"
            << "  --log-wait           Log when waiting for frames\n"
            << "  --wait-log-interval  Seconds between wait logs\n"
            << "  --balance VALUE      Undistort balance (0..1)\n"
            << "  --no-projection      Ignore projection_matrix even if present\n"
            << "  --camera-fps VALUE   Camera FPS for ORB-SLAM3 config\n"
            << "  --show-preview       Show RGB + depth preview window\n"
            << "  --show-raw           Show raw RGB in the preview window\n"
            << "  --voxel-size M       Voxel size in meters (default 0.1)\n"
            << "  --max-depth M        Max depth in meters (default 6.0)\n"
            << "  --depth-scale S      Depth scale multiplier (default 1.0)\n"
            << "  --stride N           Pixel stride for integration (default 4)\n"
            << "  --min-score N        Min occupancy score to render (default 1)\n"
            << "  --decay-sec S        Voxel decay seconds (default 120)\n"
            << "  --max-voxels N       Max voxels stored (default 200000)\n"
            << "  --max-render N       Max voxels rendered (default 100000)\n";
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
    } else if (arg == "--show-preview") {
      opt.show_preview = true;
    } else if (arg == "--show-raw") {
      opt.show_raw = true;
    } else if (arg == "--voxel-size") {
      opt.voxel_size = std::stof(next());
    } else if (arg == "--max-depth") {
      opt.max_depth = std::stof(next());
    } else if (arg == "--depth-scale") {
      opt.depth_scale = std::stof(next());
    } else if (arg == "--stride") {
      opt.stride = std::stoi(next());
    } else if (arg == "--min-score" || arg == "--min-hits") {
      opt.min_score = std::stoi(next());
    } else if (arg == "--decay-sec") {
      opt.decay_sec = std::stod(next());
    } else if (arg == "--max-voxels") {
      opt.max_voxels = static_cast<std::size_t>(std::stoul(next()));
    } else if (arg == "--max-render") {
      opt.max_render = static_cast<std::size_t>(std::stoul(next()));
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

static void draw_axes(float size)
{
  glLineWidth(2.0f);
  glBegin(GL_LINES);
  glColor3f(1.0f, 0.2f, 0.2f);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(size, 0.0f, 0.0f);
  glColor3f(0.2f, 1.0f, 0.2f);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, size, 0.0f);
  glColor3f(0.2f, 0.2f, 1.0f);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, size);
  glEnd();
}

static void draw_trajectory(const std::vector<Eigen::Vector3f> & trajectory)
{
  if (trajectory.size() < 2) {
    return;
  }

  glLineWidth(2.0f);
  glColor3f(0.2f, 1.0f, 0.2f);
  glBegin(GL_LINE_STRIP);
  for (const auto & point : trajectory) {
    glVertex3f(point.x(), point.y(), point.z());
  }
  glEnd();
}

static void draw_cube(const Eigen::Vector3f & center, float half, const Eigen::Vector3f & color)
{
  const float x = center.x();
  const float y = center.y();
  const float z = center.z();

  glColor3f(color.x(), color.y(), color.z());
  glBegin(GL_QUADS);
  glVertex3f(x - half, y - half, z + half);
  glVertex3f(x + half, y - half, z + half);
  glVertex3f(x + half, y + half, z + half);
  glVertex3f(x - half, y + half, z + half);

  glVertex3f(x - half, y - half, z - half);
  glVertex3f(x - half, y + half, z - half);
  glVertex3f(x + half, y + half, z - half);
  glVertex3f(x + half, y - half, z - half);

  glVertex3f(x - half, y - half, z - half);
  glVertex3f(x - half, y - half, z + half);
  glVertex3f(x - half, y + half, z + half);
  glVertex3f(x - half, y + half, z - half);

  glVertex3f(x + half, y - half, z - half);
  glVertex3f(x + half, y + half, z - half);
  glVertex3f(x + half, y + half, z + half);
  glVertex3f(x + half, y - half, z + half);

  glVertex3f(x - half, y + half, z - half);
  glVertex3f(x - half, y + half, z + half);
  glVertex3f(x + half, y + half, z + half);
  glVertex3f(x + half, y + half, z - half);

  glVertex3f(x - half, y - half, z - half);
  glVertex3f(x + half, y - half, z - half);
  glVertex3f(x + half, y - half, z + half);
  glVertex3f(x - half, y - half, z + half);
  glEnd();
}

static void draw_voxels(
  const std::vector<xlernav::VoxelPoint> & voxels,
  float voxel_size,
  std::size_t max_render)
{
  if (voxels.empty()) {
    return;
  }

  std::size_t stride = 1;
  if (max_render > 0 && voxels.size() > max_render) {
    stride = voxels.size() / max_render;
    if (stride < 1) {
      stride = 1;
    }
  }

  const float half = voxel_size * 0.5f;
  for (std::size_t i = 0; i < voxels.size(); i += stride) {
    const auto & voxel = voxels[i];
    draw_cube(voxel.center, half, voxel.color);
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
            << "Voxel size: " << opt.voxel_size << " m\n";

  xlernav::StreamReceiver receiver(opt.port, opt.decoder);
  if (!receiver.Open()) {
    std::cerr << "Failed to open video stream. Check GStreamer pipeline.\n";
    return 1;
  }

  xlernav::DepthEstimator depth_engine(opt.engine_path);

  sensor_msgs::msg::CameraInfo camera_info = make_camera_info(new_k, width, height);

  ORB_SLAM3::System slam(opt.vocab_path, orb_config, ORB_SLAM3::System::RGBD, true);

  xlernav::FpsCounter rx_fps(opt.fps_interval);
  xlernav::FpsCounter depth_fps(opt.fps_interval);
  auto last_wait_log = std::chrono::steady_clock::now();
  const auto start_time = std::chrono::steady_clock::now();

  xlernav::VoxelMap voxel_map(opt.voxel_size, opt.max_voxels, opt.decay_sec);
  std::vector<Eigen::Vector3f> trajectory;

  pangolin::CreateWindowAndBind("Voxel Map", 1280, 720);
  glEnable(GL_DEPTH_TEST);
  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1280, 720, 900, 900, 640, 360, 0.1, 1000),
    pangolin::ModelViewLookAt(0, -2, -2, 0, 0, 0, pangolin::AxisY));
  pangolin::Handler3D handler(s_cam);
  pangolin::View & d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0, 1.0, -1280.0f / 720.0f)
    .SetHandler(&handler);

  if (opt.show_preview) {
    cv::namedWindow("RGB + Depth", cv::WINDOW_NORMAL);
  }

  cv::Mat frame;
  cv::Mat rectified;

  while (g_running && !pangolin::ShouldQuit()) {
    bool got_frame = receiver.Read(frame) && !frame.empty();
    if (got_frame) {
      cv::remap(frame, rectified, map1, map2, cv::INTER_LINEAR);
      if (camera_info.width != static_cast<uint32_t>(rectified.cols) ||
          camera_info.height != static_cast<uint32_t>(rectified.rows)) {
        camera_info.width = static_cast<uint32_t>(rectified.cols);
        camera_info.height = static_cast<uint32_t>(rectified.rows);
      }

      rx_fps.tick();

      if (!depth_engine.Infer(rectified, camera_info)) {
        std::cerr << "Depth inference failed.\n";
      } else {
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
          trajectory.push_back(Twc.translation());
          voxel_map.Integrate(
            depth,
            rectified,
            new_k,
            Twc.matrix(),
            opt.stride,
            opt.max_depth,
            opt.depth_scale,
            timestamp);
        }

        if (opt.log_fps && rx_fps.fps() > 0.0 && depth_fps.fps() > 0.0) {
          std::cout << "[fps] rx=" << rx_fps.fps() << " depth=" << depth_fps.fps() << "\n";
        }

        if (opt.show_preview) {
          const cv::Mat preview_rgb = opt.show_raw ? frame : rectified;
          cv::Mat rgb_vis;
          if (preview_rgb.size() != rectified.size()) {
            cv::resize(preview_rgb, rgb_vis, rectified.size(), 0, 0, cv::INTER_LINEAR);
          } else {
            rgb_vis = preview_rgb;
          }

          cv::Mat depth_vis = colorize_depth(depth);
          if (depth_vis.empty()) {
            depth_vis = cv::Mat(rectified.size(), CV_8UC3, cv::Scalar(0, 0, 0));
          } else if (depth_vis.size() != rectified.size()) {
            cv::resize(depth_vis, depth_vis, rectified.size(), 0, 0, cv::INTER_NEAREST);
          }

          cv::Mat combined;
          cv::hconcat(rgb_vis, depth_vis, combined);

          if (opt.show_fps) {
            std::ostringstream oss;
            oss << "rx " << std::fixed << std::setprecision(1) << rx_fps.fps()
                << " fps | depth " << depth_fps.fps() << " fps";
            cv::putText(combined, oss.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
          }

          cv::imshow("RGB + Depth", combined);
          if (cv::waitKey(1) == 'q') {
            break;
          }
        }
      }
    } else if (opt.log_wait) {
      const auto now = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(now - last_wait_log).count();
      if (elapsed >= opt.wait_log_interval) {
        std::cout << "[stream] waiting for frames...\n";
        last_wait_log = now;
      }
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    draw_axes(0.5f);
    draw_trajectory(trajectory);
    draw_voxels(voxel_map.Snapshot(opt.min_score), voxel_map.voxel_size(), opt.max_render);
    pangolin::FinishFrame();
  }

  slam.Shutdown();
  cv::destroyAllWindows();
  return 0;
}
