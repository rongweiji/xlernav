#include "calibration.hpp"
#include "depth_estimator.hpp"
#include "fps_counter.hpp"
#include "stream_receiver.hpp"
#include "voxel_map.hpp"

#include <sensor_msgs/msg/camera_info.hpp>

#include <System.h>
#include <Tracking.h>

#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <QApplication>
#include <QCoreApplication>
#include <QCloseEvent>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QMainWindow>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShader>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPixmap>
#include <QStatusBar>
#include <QSplitter>
#include <QSurfaceFormat>
#include <QTimer>
#include <QPushButton>
#include <QVector3D>
#include <QVBoxLayout>

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
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
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
  bool log_timing = false;
  double fps_interval = 1.0;
  bool log_wait = false;
  double wait_log_interval = 5.0;
  double undistort_balance = 0.0;
  bool use_projection = true;
  double camera_fps = 30.0;
  bool show_preview = true;
  bool show_raw = false;
  int grid_x = 50;
  int grid_y = 50;
  int grid_z = 30;
  float voxel_size = 0.1f;
  float max_depth = 6.0f;
  float depth_scale = 1.0f;
  int stride = 4;
  int min_score = 1;
  std::size_t max_voxels = 200000;
  std::size_t max_render = 100000;
  bool rolling_grid = true;
  bool unbounded_map = true;
  float view_roll_deg = 180.0f;
  float view_pitch_deg = 0.0f;
  float view_yaw_deg = 0.0f;
  float view_x = 0.0f;
  float view_y = 0.0f;
  float view_z = 0.0f;
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
            << "  --show-fps           Show FPS in the UI status bar\n"
            << "  --log-fps            Print FPS to stdout\n"
            << "  --log-timing         Print per-stage timing to stdout\n"
            << "  --fps-interval SEC   FPS update interval\n"
            << "  --log-wait           Log when waiting for frames\n"
            << "  --wait-log-interval  Seconds between wait logs\n"
            << "  --balance VALUE      Undistort balance (0..1)\n"
            << "  --no-projection      Ignore projection_matrix even if present\n"
            << "  --camera-fps VALUE   Camera FPS for ORB-SLAM3 config\n"
            << "  --show-preview       Show RGB + depth panels (default)\n"
            << "  --no-preview         Hide RGB + depth panels\n"
            << "  --show-raw           Show raw RGB in the preview panel\n"
            << "  --grid-x N           Local grid size in X (default 50)\n"
            << "  --grid-y N           Local grid size in Y (default 50)\n"
            << "  --grid-z N           Local grid size in Z (default 30)\n"
            << "  --voxel-size M       Voxel size in meters (default 0.1)\n"
            << "  --max-depth M        Max depth in meters (default 6.0)\n"
            << "  --depth-scale S      Depth scale multiplier (default 1.0)\n"
            << "  --stride N           Pixel stride for integration (default 4)\n"
            << "  --min-score N        Min occupancy score to render (default 1)\n"
            << "  --max-voxels N       Legacy flag (no-op with local grid)\n"
            << "  --max-render N       Max voxels rendered (default 100000)\n"
            << "  --unbounded-map      Enable global unbounded map (default)\n"
            << "  --bounded-map        Disable global map (local grid only)\n"
            << "  --fixed-grid         Keep local grid fixed at first pose\n"
            << "  --rolling-grid       Keep local grid centered around the camera (default)\n"
            << "  --view-roll DEG      Display roll alignment (default 180)\n"
            << "  --view-pitch DEG     Display pitch alignment (default 0)\n"
            << "  --view-yaw DEG       Display yaw alignment (default 0)\n"
            << "  --view-x M           Display X offset in meters (default 0)\n"
            << "  --view-y M           Display Y offset in meters (default 0)\n"
            << "  --view-z M           Display Z offset in meters (default 0)\n";
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
    } else if (arg == "--show-preview") {
      opt.show_preview = true;
    } else if (arg == "--no-preview") {
      opt.show_preview = false;
    } else if (arg == "--show-raw") {
      opt.show_raw = true;
    } else if (arg == "--grid-x") {
      opt.grid_x = std::max(1, std::stoi(next()));
    } else if (arg == "--grid-y") {
      opt.grid_y = std::max(1, std::stoi(next()));
    } else if (arg == "--grid-z") {
      opt.grid_z = std::max(1, std::stoi(next()));
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
    } else if (arg == "--max-voxels") {
      opt.max_voxels = static_cast<std::size_t>(std::stoul(next()));
    } else if (arg == "--max-render") {
      opt.max_render = static_cast<std::size_t>(std::stoul(next()));
    } else if (arg == "--unbounded-map") {
      opt.unbounded_map = true;
    } else if (arg == "--bounded-map") {
      opt.unbounded_map = false;
    } else if (arg == "--fixed-grid") {
      opt.rolling_grid = false;
    } else if (arg == "--rolling-grid") {
      opt.rolling_grid = true;
    } else if (arg == "--view-roll") {
      opt.view_roll_deg = std::stof(next());
    } else if (arg == "--view-pitch") {
      opt.view_pitch_deg = std::stof(next());
    } else if (arg == "--view-yaw") {
      opt.view_yaw_deg = std::stof(next());
    } else if (arg == "--view-x") {
      opt.view_x = std::stof(next());
    } else if (arg == "--view-y") {
      opt.view_y = std::stof(next());
    } else if (arg == "--view-z") {
      opt.view_z = std::stof(next());
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

static Eigen::Matrix4f make_view_transform(
  float roll_deg,
  float pitch_deg,
  float yaw_deg,
  float x,
  float y,
  float z)
{
  constexpr float kPi = 3.14159265358979323846f;
  const float roll = roll_deg * kPi / 180.0f;
  const float pitch = pitch_deg * kPi / 180.0f;
  const float yaw = yaw_deg * kPi / 180.0f;

  const Eigen::AngleAxisf roll_axis(roll, Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf pitch_axis(pitch, Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf yaw_axis(yaw, Eigen::Vector3f::UnitZ());
  const Eigen::Matrix3f rotation = (yaw_axis * pitch_axis * roll_axis).toRotationMatrix();

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = rotation;
  transform.block<3, 1>(0, 3) = Eigen::Vector3f(x, y, z);
  return transform;
}

static Eigen::Matrix4f make_view_transform(const Options & opt)
{
  return make_view_transform(
    opt.view_roll_deg,
    opt.view_pitch_deg,
    opt.view_yaw_deg,
    opt.view_x,
    opt.view_y,
    opt.view_z);
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

static QImage mat_to_qimage_rgb(const cv::Mat & bgr)
{
  if (bgr.empty()) {
    return {};
  }

  cv::Mat rgb;
  if (bgr.type() == CV_8UC3) {
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  } else if (bgr.type() == CV_8UC1) {
    cv::cvtColor(bgr, rgb, cv::COLOR_GRAY2RGB);
  } else {
    cv::Mat tmp;
    bgr.convertTo(tmp, CV_8UC3);
    cv::cvtColor(tmp, rgb, cv::COLOR_BGR2RGB);
  }

  QImage image(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), QImage::Format_RGB888);
  return image.copy();
}

struct SharedState {
  std::mutex mutex;
  QImage rgb;
  QImage depth;
  std::vector<xlernav::VoxelPoint> voxels;
  std::vector<xlernav::VoxelPoint> esdf_points;
  std::vector<Eigen::Vector3f> trajectory;
  Eigen::Matrix4f latest_pose = Eigen::Matrix4f::Identity();
  bool has_pose = false;
  double rx_fps = 0.0;
  double depth_fps = 0.0;
  float voxel_size = 0.1f;
  uint64_t image_seq = 0;
  uint64_t map_seq = 0;
};

struct TimingStats {
  double read_ms = 0.0;
  double remap_ms = 0.0;
  double depth_ms = 0.0;
  double slam_ms = 0.0;
  double integrate_ms = 0.0;
  double image_ms = 0.0;
  double snapshot_ms = 0.0;
  double loop_ms = 0.0;
  int frames = 0;
  int loops = 0;
  int image_updates = 0;
  int map_updates = 0;
  int slam_updates = 0;
  int integrate_updates = 0;

  void reset()
  {
    read_ms = 0.0;
    remap_ms = 0.0;
    depth_ms = 0.0;
    slam_ms = 0.0;
    integrate_ms = 0.0;
    image_ms = 0.0;
    snapshot_ms = 0.0;
    loop_ms = 0.0;
    frames = 0;
    loops = 0;
    image_updates = 0;
    map_updates = 0;
    slam_updates = 0;
    integrate_updates = 0;
  }
};

static double to_ms(const std::chrono::steady_clock::duration & duration)
{
  return std::chrono::duration<double, std::milli>(duration).count();
}

class VoxelGLWidget : public QOpenGLWidget, protected QOpenGLFunctions {
public:
  explicit VoxelGLWidget(QWidget * parent = nullptr);

  void setDataTransform(const Eigen::Matrix4f & transform);
  void setData(
    const std::vector<xlernav::VoxelPoint> & voxels,
    const std::vector<Eigen::Vector3f> & trajectory,
    const Eigen::Matrix4f & latest_pose,
    bool has_pose,
    float voxel_size);

protected:
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void wheelEvent(QWheelEvent * event) override;

private:
  void updateBuffers();
  QMatrix4x4 viewProjection() const;
  QVector3D cameraForward() const;

  QOpenGLShaderProgram program_;
  QOpenGLBuffer voxel_vbo_;
  QOpenGLVertexArrayObject voxel_vao_;
  QOpenGLBuffer line_vbo_;
  QOpenGLVertexArrayObject line_vao_;
  std::vector<float> voxel_data_;
  std::vector<float> line_data_;
  int voxel_count_ = 0;
  int axis_vertices_ = 0;
  int traj_vertices_ = 0;
  bool dirty_ = false;
  float voxel_size_ = 0.1f;
  float point_size_ = 4.0f;
  float yaw_deg_ = 0.0f;
  float pitch_deg_ = -20.0f;
  float distance_ = 3.0f;
  QVector3D target_{0.0f, 0.0f, 0.0f};
  QPoint last_pos_;
  Eigen::Matrix4f data_transform_ = Eigen::Matrix4f::Identity();
};

VoxelGLWidget::VoxelGLWidget(QWidget * parent)
: QOpenGLWidget(parent),
  voxel_vbo_(QOpenGLBuffer::VertexBuffer),
  line_vbo_(QOpenGLBuffer::VertexBuffer)
{
  setFocusPolicy(Qt::StrongFocus);
  setMinimumSize(640, 480);
}

void VoxelGLWidget::setDataTransform(const Eigen::Matrix4f & transform)
{
  data_transform_ = transform;
}

void VoxelGLWidget::setData(
  const std::vector<xlernav::VoxelPoint> & voxels,
  const std::vector<Eigen::Vector3f> & trajectory,
  const Eigen::Matrix4f & latest_pose,
  bool has_pose,
  float voxel_size)
{
  voxel_size_ = voxel_size;
  point_size_ = std::clamp(voxel_size_ * 40.0f, 2.0f, 12.0f);

  const Eigen::Matrix3f rot = data_transform_.block<3, 3>(0, 0);
  const Eigen::Vector3f trans = data_transform_.block<3, 1>(0, 3);
  auto transform_point = [&](float x, float y, float z) -> Eigen::Vector3f {
    return rot * Eigen::Vector3f(x, y, z) + trans;
  };

  voxel_data_.clear();
  voxel_data_.reserve(voxels.size() * 6);
  for (const auto & voxel : voxels) {
    const Eigen::Vector3f p = transform_point(voxel.center.x(), voxel.center.y(), voxel.center.z());
    voxel_data_.push_back(p.x());
    voxel_data_.push_back(p.y());
    voxel_data_.push_back(p.z());
    voxel_data_.push_back(voxel.color.x());
    voxel_data_.push_back(voxel.color.y());
    voxel_data_.push_back(voxel.color.z());
  }

  line_data_.clear();
  auto push_line_display = [this](float x, float y, float z, float r, float g, float b) {
    line_data_.push_back(x);
    line_data_.push_back(y);
    line_data_.push_back(z);
    line_data_.push_back(r);
    line_data_.push_back(g);
    line_data_.push_back(b);
  };
  auto push_line_data = [&](float x, float y, float z, float r, float g, float b) {
    const Eigen::Vector3f p = transform_point(x, y, z);
    push_line_display(p.x(), p.y(), p.z(), r, g, b);
  };

  const float grid_extent = 5.0f;
  const float grid_step = 0.5f;
  const float grid_y = 0.0f;
  const float grid_color = 0.18f;
  for (float x = -grid_extent; x <= grid_extent + 1e-4f; x += grid_step) {
    push_line_display(x, grid_y, -grid_extent, grid_color, grid_color, grid_color);
    push_line_display(x, grid_y, grid_extent, grid_color, grid_color, grid_color);
  }
  for (float z = -grid_extent; z <= grid_extent + 1e-4f; z += grid_step) {
    push_line_display(-grid_extent, grid_y, z, grid_color, grid_color, grid_color);
    push_line_display(grid_extent, grid_y, z, grid_color, grid_color, grid_color);
  }

  const float axis_len = 0.5f;
  push_line_display(0.0f, 0.0f, 0.0f, 1.0f, 0.2f, 0.2f);
  push_line_display(axis_len, 0.0f, 0.0f, 1.0f, 0.2f, 0.2f);
  push_line_display(0.0f, 0.0f, 0.0f, 0.2f, 1.0f, 0.2f);
  push_line_display(0.0f, axis_len, 0.0f, 0.2f, 1.0f, 0.2f);
  push_line_display(0.0f, 0.0f, 0.0f, 0.2f, 0.2f, 1.0f);
  push_line_display(0.0f, 0.0f, axis_len, 0.2f, 0.2f, 1.0f);

  if (has_pose) {
    const Eigen::Matrix3f pose_rot = latest_pose.block<3, 3>(0, 0);
    const Eigen::Vector3f pose_t = latest_pose.block<3, 1>(0, 3);
    const Eigen::Matrix3f disp_rot = rot * pose_rot;
    const Eigen::Vector3f disp_t = rot * pose_t + trans;
    const float cam_axis = 0.25f;
    const Eigen::Vector3f x_axis = disp_t + disp_rot * Eigen::Vector3f(cam_axis, 0.0f, 0.0f);
    const Eigen::Vector3f y_axis = disp_t + disp_rot * Eigen::Vector3f(0.0f, cam_axis, 0.0f);
    const Eigen::Vector3f z_axis = disp_t + disp_rot * Eigen::Vector3f(0.0f, 0.0f, cam_axis);
    push_line_display(disp_t.x(), disp_t.y(), disp_t.z(), 0.9f, 0.2f, 0.2f);
    push_line_display(x_axis.x(), x_axis.y(), x_axis.z(), 0.9f, 0.2f, 0.2f);
    push_line_display(disp_t.x(), disp_t.y(), disp_t.z(), 0.2f, 0.9f, 0.2f);
    push_line_display(y_axis.x(), y_axis.y(), y_axis.z(), 0.2f, 0.9f, 0.2f);
    push_line_display(disp_t.x(), disp_t.y(), disp_t.z(), 0.2f, 0.2f, 0.9f);
    push_line_display(z_axis.x(), z_axis.y(), z_axis.z(), 0.2f, 0.2f, 0.9f);
  }
  axis_vertices_ = static_cast<int>(line_data_.size() / 6);

  for (const auto & point : trajectory) {
    push_line_data(point.x(), point.y(), point.z(), 0.2f, 1.0f, 0.2f);
  }
  traj_vertices_ = static_cast<int>(trajectory.size());

  dirty_ = true;
  update();
}

void VoxelGLWidget::initializeGL()
{
  initializeOpenGLFunctions();
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_PROGRAM_POINT_SIZE);
  glClearColor(0.05f, 0.06f, 0.08f, 1.0f);

  const char * vertex_src =
    "#version 330 core\n"
    "layout(location = 0) in vec3 aPos;\n"
    "layout(location = 1) in vec3 aColor;\n"
    "uniform mat4 uMvp;\n"
    "uniform float uPointSize;\n"
    "out vec3 vColor;\n"
    "void main() {\n"
    "  gl_Position = uMvp * vec4(aPos, 1.0);\n"
    "  gl_PointSize = uPointSize;\n"
    "  vColor = aColor;\n"
    "}\n";

  const char * fragment_src =
    "#version 330 core\n"
    "in vec3 vColor;\n"
    "out vec4 FragColor;\n"
    "void main() {\n"
    "  FragColor = vec4(vColor, 1.0);\n"
    "}\n";

  if (!program_.addShaderFromSourceCode(QOpenGLShader::Vertex, vertex_src)) {
    std::cerr << "Failed to compile vertex shader: "
              << program_.log().toStdString() << "\n";
  }
  if (!program_.addShaderFromSourceCode(QOpenGLShader::Fragment, fragment_src)) {
    std::cerr << "Failed to compile fragment shader: "
              << program_.log().toStdString() << "\n";
  }
  if (!program_.link()) {
    std::cerr << "Failed to link shader program: "
              << program_.log().toStdString() << "\n";
  }

  program_.bind();

  voxel_vao_.create();
  voxel_vbo_.create();
  voxel_vao_.bind();
  voxel_vbo_.bind();
  voxel_vbo_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
  program_.enableAttributeArray(0);
  program_.enableAttributeArray(1);
  program_.setAttributeBuffer(0, GL_FLOAT, 0, 3, 6 * sizeof(float));
  program_.setAttributeBuffer(1, GL_FLOAT, 3 * sizeof(float), 3, 6 * sizeof(float));
  voxel_vbo_.release();
  voxel_vao_.release();

  line_vao_.create();
  line_vbo_.create();
  line_vao_.bind();
  line_vbo_.bind();
  line_vbo_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
  program_.enableAttributeArray(0);
  program_.enableAttributeArray(1);
  program_.setAttributeBuffer(0, GL_FLOAT, 0, 3, 6 * sizeof(float));
  program_.setAttributeBuffer(1, GL_FLOAT, 3 * sizeof(float), 3, 6 * sizeof(float));
  line_vbo_.release();
  line_vao_.release();

  program_.release();
}

void VoxelGLWidget::resizeGL(int, int)
{
}

void VoxelGLWidget::paintGL()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if (!program_.isLinked()) {
    return;
  }

  if (dirty_) {
    updateBuffers();
    dirty_ = false;
  }

  const QMatrix4x4 mvp = viewProjection();

  program_.bind();
  program_.setUniformValue("uMvp", mvp);

  if (voxel_count_ > 0) {
    program_.setUniformValue("uPointSize", point_size_);
    voxel_vao_.bind();
    glDrawArrays(GL_POINTS, 0, voxel_count_);
    voxel_vao_.release();
  }

  if (axis_vertices_ > 0) {
    program_.setUniformValue("uPointSize", 1.0f);
    line_vao_.bind();
    glLineWidth(2.0f);
    glDrawArrays(GL_LINES, 0, axis_vertices_);
    if (traj_vertices_ > 1) {
      glDrawArrays(GL_LINE_STRIP, axis_vertices_, traj_vertices_);
    }
    line_vao_.release();
  }

  program_.release();
}

void VoxelGLWidget::mousePressEvent(QMouseEvent * event)
{
  last_pos_ = event->pos();
}

void VoxelGLWidget::mouseMoveEvent(QMouseEvent * event)
{
  const QPoint delta = event->pos() - last_pos_;
  last_pos_ = event->pos();

  if (event->buttons() & Qt::LeftButton) {
    yaw_deg_ += static_cast<float>(delta.x()) * 0.4f;
    pitch_deg_ += static_cast<float>(delta.y()) * 0.4f;
    pitch_deg_ = std::clamp(pitch_deg_, -89.0f, 89.0f);
    update();
  }

  if (event->buttons() & Qt::RightButton || event->buttons() & Qt::MiddleButton) {
    const QVector3D forward = cameraForward();
    QVector3D right = QVector3D::crossProduct(forward, QVector3D(0.0f, 1.0f, 0.0f)).normalized();
    QVector3D up = QVector3D::crossProduct(right, forward).normalized();
    const float pan_scale = std::max(0.01f, distance_ * 0.002f);
    target_ -= right * (static_cast<float>(delta.x()) * pan_scale);
    target_ += up * (static_cast<float>(delta.y()) * pan_scale);
    update();
  }
}

void VoxelGLWidget::wheelEvent(QWheelEvent * event)
{
  const float steps = static_cast<float>(event->angleDelta().y()) / 120.0f;
  if (steps == 0.0f) {
    return;
  }
  distance_ *= std::pow(0.9f, steps);
  distance_ = std::clamp(distance_, 0.2f, 50.0f);
  update();
}

void VoxelGLWidget::updateBuffers()
{
  if (!voxel_vao_.isCreated() || !line_vao_.isCreated()) {
    return;
  }

  voxel_count_ = static_cast<int>(voxel_data_.size() / 6);
  if (voxel_count_ > 0) {
    voxel_vbo_.bind();
    voxel_vbo_.allocate(voxel_data_.data(), static_cast<int>(voxel_data_.size() * sizeof(float)));
    voxel_vbo_.release();
  }

  if (!line_data_.empty()) {
    line_vbo_.bind();
    line_vbo_.allocate(line_data_.data(), static_cast<int>(line_data_.size() * sizeof(float)));
    line_vbo_.release();
  }
}

QMatrix4x4 VoxelGLWidget::viewProjection() const
{
  const float aspect = height() > 0 ? static_cast<float>(width()) / static_cast<float>(height()) : 1.0f;
  QMatrix4x4 projection;
  projection.perspective(45.0f, aspect, 0.05f, 200.0f);

  const QVector3D forward = cameraForward();
  const QVector3D eye = target_ + forward * distance_;
  QMatrix4x4 view;
  view.lookAt(eye, target_, QVector3D(0.0f, 1.0f, 0.0f));

  return projection * view;
}

QVector3D VoxelGLWidget::cameraForward() const
{
  constexpr float kPi = 3.14159265358979323846f;
  const float yaw = yaw_deg_ * kPi / 180.0f;
  const float pitch = pitch_deg_ * kPi / 180.0f;
  const QVector3D forward(
    std::cos(pitch) * std::sin(yaw),
    std::sin(pitch),
    std::cos(pitch) * std::cos(yaw));
  return forward.normalized();
}

class MainWindow : public QMainWindow {
public:
  MainWindow(SharedState * state, const Options & opt, QWidget * parent = nullptr);

protected:
  void closeEvent(QCloseEvent * event) override;

private:
  void refreshUi();
  void updateImage(QLabel * label, const QImage & image, const QString & placeholder);
  void applyViewTransform();

  SharedState * state_ = nullptr;
  QLabel * rgb_label_ = nullptr;
  QLabel * depth_label_ = nullptr;
  QLabel * fps_label_ = nullptr;
  QWidget * preview_panel_ = nullptr;
  VoxelGLWidget * gl_widget_ = nullptr;
  VoxelGLWidget * esdf_widget_ = nullptr;
  QTimer * timer_ = nullptr;
  QDoubleSpinBox * view_roll_ = nullptr;
  QDoubleSpinBox * view_pitch_ = nullptr;
  QDoubleSpinBox * view_yaw_ = nullptr;
  QDoubleSpinBox * view_x_ = nullptr;
  QDoubleSpinBox * view_y_ = nullptr;
  QDoubleSpinBox * view_z_ = nullptr;
  uint64_t last_image_seq_ = 0;
  uint64_t last_map_seq_ = 0;
  bool show_fps_ = false;
  std::vector<xlernav::VoxelPoint> cached_voxels_;
  std::vector<xlernav::VoxelPoint> cached_esdf_points_;
  std::vector<Eigen::Vector3f> cached_trajectory_;
  float cached_voxel_size_ = 0.1f;
  Eigen::Matrix4f cached_pose_ = Eigen::Matrix4f::Identity();
  bool cached_has_pose_ = false;
};

MainWindow::MainWindow(SharedState * state, const Options & opt, QWidget * parent)
: QMainWindow(parent), state_(state), show_fps_(opt.show_fps)
{
  setWindowTitle("xlernav voxel viewer");

  auto * central = new QWidget(this);
  auto * splitter = new QSplitter(Qt::Horizontal, central);
  auto * view_splitter = new QSplitter(Qt::Vertical, splitter);

  preview_panel_ = new QWidget(splitter);
  auto * preview_layout = new QVBoxLayout(preview_panel_);
  preview_layout->setContentsMargins(4, 4, 4, 4);
  preview_layout->setSpacing(6);

  rgb_label_ = new QLabel("RGB", preview_panel_);
  rgb_label_->setAlignment(Qt::AlignCenter);
  rgb_label_->setMinimumSize(320, 240);
  depth_label_ = new QLabel("Depth", preview_panel_);
  depth_label_->setAlignment(Qt::AlignCenter);
  depth_label_->setMinimumSize(320, 240);

  preview_layout->addWidget(rgb_label_);
  preview_layout->addWidget(depth_label_);

  gl_widget_ = new VoxelGLWidget(view_splitter);
  gl_widget_->setDataTransform(make_view_transform(opt));
  gl_widget_->setData({}, {}, Eigen::Matrix4f::Identity(), false, opt.voxel_size);

  esdf_widget_ = new VoxelGLWidget(view_splitter);
  esdf_widget_->setDataTransform(make_view_transform(opt));
  esdf_widget_->setData({}, {}, Eigen::Matrix4f::Identity(), false, opt.voxel_size);

  view_splitter->addWidget(gl_widget_);
  view_splitter->addWidget(esdf_widget_);
  view_splitter->setStretchFactor(0, 1);
  view_splitter->setStretchFactor(1, 1);

  splitter->addWidget(preview_panel_);
  splitter->addWidget(view_splitter);
  splitter->setStretchFactor(0, 0);
  splitter->setStretchFactor(1, 1);

  preview_panel_->setVisible(opt.show_preview);

  auto * layout = new QHBoxLayout(central);
  layout->setContentsMargins(4, 4, 4, 4);
  layout->addWidget(splitter);

  setCentralWidget(central);

  auto * view_dock = new QDockWidget("View Align", this);
  view_dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  auto * view_panel = new QWidget(view_dock);
  auto * view_layout = new QFormLayout(view_panel);
  view_layout->setContentsMargins(8, 8, 8, 8);
  view_layout->setSpacing(6);

  auto make_spin = [&](double min, double max, double step, double value, int decimals) {
    auto * spin = new QDoubleSpinBox(view_panel);
    spin->setRange(min, max);
    spin->setSingleStep(step);
    spin->setDecimals(decimals);
    spin->setValue(value);
    return spin;
  };

  view_roll_ = make_spin(-180.0, 180.0, 1.0, opt.view_roll_deg, 1);
  view_pitch_ = make_spin(-180.0, 180.0, 1.0, opt.view_pitch_deg, 1);
  view_yaw_ = make_spin(-180.0, 180.0, 1.0, opt.view_yaw_deg, 1);
  view_x_ = make_spin(-10.0, 10.0, 0.05, opt.view_x, 2);
  view_y_ = make_spin(-10.0, 10.0, 0.05, opt.view_y, 2);
  view_z_ = make_spin(-10.0, 10.0, 0.05, opt.view_z, 2);

  view_layout->addRow("Roll (deg)", view_roll_);
  view_layout->addRow("Pitch (deg)", view_pitch_);
  view_layout->addRow("Yaw (deg)", view_yaw_);
  view_layout->addRow("X (m)", view_x_);
  view_layout->addRow("Y (m)", view_y_);
  view_layout->addRow("Z (m)", view_z_);

  auto * reset_button = new QPushButton("Reset", view_panel);
  auto * print_button = new QPushButton("Print CLI", view_panel);
  auto * button_row = new QWidget(view_panel);
  auto * button_layout = new QHBoxLayout(button_row);
  button_layout->setContentsMargins(0, 0, 0, 0);
  button_layout->addWidget(reset_button);
  button_layout->addWidget(print_button);
  view_layout->addRow(button_row);

  view_dock->setWidget(view_panel);
  addDockWidget(Qt::RightDockWidgetArea, view_dock);

  auto on_view_change = [this]() { applyViewTransform(); };
  connect(view_roll_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [on_view_change](double) { on_view_change(); });
  connect(view_pitch_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [on_view_change](double) { on_view_change(); });
  connect(view_yaw_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [on_view_change](double) { on_view_change(); });
  connect(view_x_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [on_view_change](double) { on_view_change(); });
  connect(view_y_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [on_view_change](double) { on_view_change(); });
  connect(view_z_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [on_view_change](double) { on_view_change(); });

  connect(reset_button, &QPushButton::clicked, this, [this, opt]() {
    view_roll_->setValue(opt.view_roll_deg);
    view_pitch_->setValue(opt.view_pitch_deg);
    view_yaw_->setValue(opt.view_yaw_deg);
    view_x_->setValue(opt.view_x);
    view_y_->setValue(opt.view_y);
    view_z_->setValue(opt.view_z);
  });

  connect(print_button, &QPushButton::clicked, this, [this]() {
    if (!view_roll_ || !view_pitch_ || !view_yaw_ || !view_x_ || !view_y_ || !view_z_) {
      return;
    }
    std::cout << "[view] --view-roll " << view_roll_->value()
              << " --view-pitch " << view_pitch_->value()
              << " --view-yaw " << view_yaw_->value()
              << " --view-x " << view_x_->value()
              << " --view-y " << view_y_->value()
              << " --view-z " << view_z_->value() << "\n";
  });

  if (show_fps_) {
    fps_label_ = new QLabel(this);
    statusBar()->addPermanentWidget(fps_label_);
  }

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &MainWindow::refreshUi);
  timer_->start(33);

  applyViewTransform();
}

void MainWindow::closeEvent(QCloseEvent * event)
{
  g_running = false;
  QMainWindow::closeEvent(event);
}

void MainWindow::refreshUi()
{
  QImage rgb;
  QImage depth;
  std::vector<xlernav::VoxelPoint> voxels;
  std::vector<xlernav::VoxelPoint> esdf_points;
  std::vector<Eigen::Vector3f> trajectory;
  Eigen::Matrix4f latest_pose = Eigen::Matrix4f::Identity();
  bool has_pose = false;
  double rx_fps = 0.0;
  double depth_fps = 0.0;
  float voxel_size = 0.1f;
  uint64_t image_seq = 0;
  uint64_t map_seq = 0;

  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    image_seq = state_->image_seq;
    map_seq = state_->map_seq;
    if (image_seq != last_image_seq_) {
      rgb = state_->rgb;
      depth = state_->depth;
      rx_fps = state_->rx_fps;
      depth_fps = state_->depth_fps;
    }
    if (map_seq != last_map_seq_) {
      voxels = state_->voxels;
      esdf_points = state_->esdf_points;
      trajectory = state_->trajectory;
      latest_pose = state_->latest_pose;
      has_pose = state_->has_pose;
      voxel_size = state_->voxel_size;
    }
  }

  if (image_seq != last_image_seq_) {
    updateImage(rgb_label_, rgb, "RGB");
    updateImage(depth_label_, depth, "Depth");
    last_image_seq_ = image_seq;
    if (show_fps_ && fps_label_) {
      std::ostringstream oss;
      oss << "rx " << std::fixed << std::setprecision(1) << rx_fps
          << " fps | depth " << depth_fps << " fps";
      fps_label_->setText(QString::fromStdString(oss.str()));
    }
  }

  if (map_seq != last_map_seq_) {
    cached_voxels_ = std::move(voxels);
    cached_esdf_points_ = std::move(esdf_points);
    cached_trajectory_ = std::move(trajectory);
    cached_voxel_size_ = voxel_size;
    cached_pose_ = latest_pose;
    cached_has_pose_ = has_pose;
    gl_widget_->setData(
      cached_voxels_,
      cached_trajectory_,
      cached_pose_,
      cached_has_pose_,
      cached_voxel_size_);
    if (esdf_widget_) {
      esdf_widget_->setData(
        cached_esdf_points_,
        cached_trajectory_,
        cached_pose_,
        cached_has_pose_,
        cached_voxel_size_);
    }
    last_map_seq_ = map_seq;
  }
}

void MainWindow::applyViewTransform()
{
  if (!gl_widget_ || !view_roll_ || !view_pitch_ || !view_yaw_ || !view_x_ || !view_y_ || !view_z_) {
    return;
  }

  const Eigen::Matrix4f transform = make_view_transform(
    static_cast<float>(view_roll_->value()),
    static_cast<float>(view_pitch_->value()),
    static_cast<float>(view_yaw_->value()),
    static_cast<float>(view_x_->value()),
    static_cast<float>(view_y_->value()),
    static_cast<float>(view_z_->value()));
  gl_widget_->setDataTransform(transform);
  if (esdf_widget_) {
    esdf_widget_->setDataTransform(transform);
  }

  if (!cached_voxels_.empty() || !cached_trajectory_.empty() || cached_has_pose_) {
    gl_widget_->setData(
      cached_voxels_,
      cached_trajectory_,
      cached_pose_,
      cached_has_pose_,
      cached_voxel_size_);
  }
  if (esdf_widget_ && (!cached_esdf_points_.empty() || !cached_trajectory_.empty() || cached_has_pose_)) {
    esdf_widget_->setData(
      cached_esdf_points_,
      cached_trajectory_,
      cached_pose_,
      cached_has_pose_,
      cached_voxel_size_);
  }
}

void MainWindow::updateImage(QLabel * label, const QImage & image, const QString & placeholder)
{
  if (!label) {
    return;
  }

  if (image.isNull()) {
    label->setPixmap(QPixmap());
    label->setText(placeholder);
    return;
  }

  const QSize target = label->size();
  QImage scaled = image;
  if (target.width() > 0 && target.height() > 0) {
    scaled = image.scaled(target, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  }
  label->setText(QString());
  label->setPixmap(QPixmap::fromImage(scaled));
}

class PipelineRunner {
public:
  PipelineRunner(const Options & opt, SharedState * state)
  : opt_(opt), state_(state) {}

  ~PipelineRunner() { stop(); }

  void start()
  {
    if (running_.exchange(true)) {
      return;
    }
    thread_ = std::thread(&PipelineRunner::run, this);
  }

  void stop()
  {
    if (!running_.exchange(false)) {
      return;
    }
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  void run()
  {
    xlernav::CalibrationData calib;
    if (!xlernav::LoadCalibrationYaml(opt_.calib_path, calib)) {
      std::cerr << "Failed to load calibration from " << opt_.calib_path << "\n";
      g_running = false;
      return;
    }

    if (!fs::exists(opt_.engine_path)) {
      std::cerr << "TensorRT engine not found: " << opt_.engine_path << "\n";
      g_running = false;
      return;
    }
    if (!fs::exists(opt_.vocab_path)) {
      std::cerr << "ORB vocabulary not found: " << opt_.vocab_path << "\n";
      g_running = false;
      return;
    }

    cv::Mat map1, map2, new_k;
    if (!xlernav::BuildUndistortMaps(calib, opt_.undistort_balance, opt_.use_projection, map1, map2, new_k)) {
      std::cerr << "Failed to build undistort maps.\n";
      g_running = false;
      return;
    }

    const int width = calib.width > 0 ? calib.width : 640;
    const int height = calib.height > 0 ? calib.height : 480;

    const std::string orb_config = runtime_config_path();
    if (!write_orbslam_config(orb_config, new_k, width, height, opt_.camera_fps)) {
      std::cerr << "Failed to write ORB-SLAM3 config.\n";
      g_running = false;
      return;
    }

    std::cout << "ORB-SLAM3 config: " << orb_config << "\n"
              << "Vocabulary: " << opt_.vocab_path << "\n"
              << "Calibration: " << opt_.calib_path << "\n"
              << "Engine: " << opt_.engine_path << "\n"
              << "Undistort: " << (opt_.use_projection ? "projection_matrix" : "intrinsics")
              << " (balance=" << opt_.undistort_balance << ")\n"
              << "Voxel size: " << opt_.voxel_size << " m\n"
              << "Global map: " << (opt_.unbounded_map ? "enabled\n" : "disabled\n")
              << "Local grid: " << opt_.grid_x << " x " << opt_.grid_y << " x " << opt_.grid_z
              << (opt_.rolling_grid ? " (rolling)\n" : " (fixed)\n")
              << "View align (deg): roll=" << opt_.view_roll_deg
              << " pitch=" << opt_.view_pitch_deg
              << " yaw=" << opt_.view_yaw_deg << "\n"
              << "View offset (m): x=" << opt_.view_x
              << " y=" << opt_.view_y
              << " z=" << opt_.view_z << "\n";

    xlernav::StreamReceiver receiver(opt_.port, opt_.decoder);
    if (!receiver.Open()) {
      std::cerr << "Failed to open video stream. Check GStreamer pipeline.\n";
      g_running = false;
      return;
    }

    xlernav::DepthEstimator depth_engine(opt_.engine_path);

    sensor_msgs::msg::CameraInfo camera_info = make_camera_info(new_k, width, height);

    ORB_SLAM3::System slam(opt_.vocab_path, orb_config, ORB_SLAM3::System::RGBD, false);

    xlernav::FpsCounter rx_fps(opt_.fps_interval);
    xlernav::FpsCounter depth_fps(opt_.fps_interval);
    auto last_wait_log = std::chrono::steady_clock::now();
    const auto start_time = std::chrono::steady_clock::now();

    xlernav::VoxelMap local_map(
      opt_.voxel_size,
      opt_.grid_x,
      opt_.grid_y,
      opt_.grid_z,
      opt_.rolling_grid,
      false);
    std::unique_ptr<xlernav::VoxelMap> global_map;
    if (opt_.unbounded_map) {
      global_map = std::make_unique<xlernav::VoxelMap>(
        opt_.voxel_size,
        opt_.grid_x,
        opt_.grid_y,
        opt_.grid_z,
        false,
        true);
    }
    std::vector<Eigen::Vector3f> trajectory;
    Eigen::Matrix4f latest_pose = Eigen::Matrix4f::Identity();
    bool has_pose = false;

    cv::Mat frame;
    cv::Mat rectified;

    const auto image_interval = std::chrono::milliseconds(33);
    const auto map_interval = std::chrono::milliseconds(100);
    auto last_image_update = std::chrono::steady_clock::now();
    auto last_map_update = std::chrono::steady_clock::now();
    TimingStats timing;
    auto last_timing_log = std::chrono::steady_clock::now();
    const auto timing_interval =
      std::chrono::duration<double>(std::max(0.2, opt_.fps_interval));

    while (running_ && g_running) {
      const auto loop_start = std::chrono::steady_clock::now();
      const auto read_start = loop_start;
      const bool got_frame = receiver.Read(frame) && !frame.empty();
      const auto read_end = std::chrono::steady_clock::now();
      if (opt_.log_timing) {
        timing.read_ms += to_ms(read_end - read_start);
        timing.loops += 1;
      }
      if (got_frame) {
        if (opt_.log_timing) {
          timing.frames += 1;
        }
        const auto remap_start = std::chrono::steady_clock::now();
        cv::remap(frame, rectified, map1, map2, cv::INTER_LINEAR);
        if (opt_.log_timing) {
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
        if (opt_.log_timing) {
          timing.depth_ms += to_ms(std::chrono::steady_clock::now() - depth_start);
        }
        if (!depth_ok) {
          std::cerr << "Depth inference failed.\n";
        } else {
          depth_fps.tick();
          const cv::Mat depth = depth_engine.Depth();
          const auto now = std::chrono::steady_clock::now();
          const double timestamp = std::chrono::duration<double>(now - start_time).count();

          const auto slam_start = std::chrono::steady_clock::now();
          const Sophus::SE3f Tcw = slam.TrackRGBD(rectified, depth, timestamp);
          if (opt_.log_timing) {
            timing.slam_ms += to_ms(std::chrono::steady_clock::now() - slam_start);
            timing.slam_updates += 1;
          }
          const int tracking_state = slam.GetTrackingState();
          const bool tracking_ok =
            tracking_state == ORB_SLAM3::Tracking::OK ||
            tracking_state == ORB_SLAM3::Tracking::OK_KLT;
          if (tracking_ok) {
            const Sophus::SE3f Twc = Tcw.inverse();
            trajectory.push_back(Twc.translation());
            latest_pose = Twc.matrix();
            has_pose = true;
            const auto integrate_start = std::chrono::steady_clock::now();
            local_map.Integrate(
              depth,
              rectified,
              new_k,
              Twc.matrix(),
              opt_.stride,
              opt_.max_depth,
              opt_.depth_scale);
            if (opt_.log_timing) {
              timing.integrate_ms += to_ms(std::chrono::steady_clock::now() - integrate_start);
              timing.integrate_updates += 1;
            }
          }

          if (opt_.log_fps && rx_fps.fps() > 0.0 && depth_fps.fps() > 0.0) {
            std::cout << "[fps] rx=" << rx_fps.fps() << " depth=" << depth_fps.fps() << "\n";
          }

          const bool update_images = opt_.show_preview && (now - last_image_update >= image_interval);
          if (update_images) {
            const auto image_start = std::chrono::steady_clock::now();
            const cv::Mat preview_rgb = opt_.show_raw ? frame : rectified;
            const cv::Mat depth_vis = colorize_depth(depth);
            QImage rgb_img = mat_to_qimage_rgb(preview_rgb);
            QImage depth_img = mat_to_qimage_rgb(depth_vis);
            if (!rgb_img.isNull() && !depth_img.isNull()) {
              std::lock_guard<std::mutex> lock(state_->mutex);
              state_->rgb = std::move(rgb_img);
              state_->depth = std::move(depth_img);
              state_->rx_fps = rx_fps.fps();
              state_->depth_fps = depth_fps.fps();
              ++state_->image_seq;
            }
            last_image_update = now;
            if (opt_.log_timing) {
              timing.image_ms += to_ms(std::chrono::steady_clock::now() - image_start);
              timing.image_updates += 1;
            }
          }

          if (now - last_map_update >= map_interval) {
            const auto snapshot_start = std::chrono::steady_clock::now();
            std::vector<xlernav::VoxelPoint> local_voxels = local_map.Snapshot(opt_.min_score);
            constexpr float kEsdfMaxDistance = 2.0f;
            std::vector<xlernav::VoxelPoint> esdf_points =
              local_map.EsdfPoints2D(opt_.min_score, kEsdfMaxDistance);
            if (global_map) {
              global_map->IntegratePoints(local_voxels);
            }

            std::vector<xlernav::VoxelPoint> voxels = global_map ?
              global_map->Snapshot(opt_.min_score) : std::move(local_voxels);
            if (opt_.max_render > 0 && voxels.size() > opt_.max_render) {
              std::vector<xlernav::VoxelPoint> sampled;
              sampled.reserve(opt_.max_render);
              const std::size_t stride =
                std::max<std::size_t>(1, voxels.size() / opt_.max_render);
              for (std::size_t i = 0; i < voxels.size(); i += stride) {
                sampled.push_back(voxels[i]);
              }
              voxels.swap(sampled);
            }

            {
              std::lock_guard<std::mutex> lock(state_->mutex);
              state_->voxels = std::move(voxels);
              state_->esdf_points = std::move(esdf_points);
              state_->trajectory = trajectory;
              state_->voxel_size = local_map.voxel_size();
              state_->latest_pose = latest_pose;
              state_->has_pose = has_pose;
              ++state_->map_seq;
            }
            last_map_update = now;
            if (opt_.log_timing) {
              timing.snapshot_ms += to_ms(std::chrono::steady_clock::now() - snapshot_start);
              timing.map_updates += 1;
            }
          }
        }
      } else if (opt_.log_wait) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - last_wait_log).count();
        if (elapsed >= opt_.wait_log_interval) {
          std::cout << "[stream] waiting for frames...\n";
          last_wait_log = now;
        }
      }

      if (opt_.log_timing) {
        timing.loop_ms += to_ms(std::chrono::steady_clock::now() - loop_start);
        const auto now = std::chrono::steady_clock::now();
        if (now - last_timing_log >= timing_interval) {
          const double elapsed = std::chrono::duration<double>(now - last_timing_log).count();
          const auto avg = [](double total, int count) -> double {
            return count > 0 ? total / static_cast<double>(count) : 0.0;
          };
          const double fps = timing.frames > 0 ? timing.frames / elapsed : 0.0;
          std::cout << std::fixed << std::setprecision(2);
          std::cout << "[timing] fps=" << fps
                    << " read=" << avg(timing.read_ms, timing.loops)
                    << " remap=" << avg(timing.remap_ms, timing.frames)
                    << " depth=" << avg(timing.depth_ms, timing.frames)
                    << " slam=" << avg(timing.slam_ms, timing.slam_updates)
                    << " integ=" << avg(timing.integrate_ms, timing.integrate_updates)
                    << " snap=" << avg(timing.snapshot_ms, timing.map_updates)
                    << " img=" << avg(timing.image_ms, timing.image_updates)
                    << " loop=" << avg(timing.loop_ms, timing.loops)
                    << " (frames=" << timing.frames
                    << " img=" << timing.image_updates
                    << " map=" << timing.map_updates
                    << " slam=" << timing.slam_updates
                    << " integ=" << timing.integrate_updates << ")\n";
          timing.reset();
          last_timing_log = now;
        }
      }
    }

    slam.Shutdown();
  }

  Options opt_;
  SharedState * state_;
  std::atomic<bool> running_{false};
  std::thread thread_;
};

int main(int argc, char ** argv)
{
  std::signal(SIGINT, HandleSignal);

  QSurfaceFormat format;
  format.setVersion(3, 3);
  format.setProfile(QSurfaceFormat::CoreProfile);
  format.setDepthBufferSize(24);
  QSurfaceFormat::setDefaultFormat(format);

  QApplication app(argc, argv);

  Options opt;
  try {
    opt = parse_args(argc, argv);
  } catch (const std::exception & e) {
    std::cerr << e.what() << "\n";
    print_usage(argv[0]);
    return 1;
  }

  SharedState state;
  PipelineRunner runner(opt, &state);
  runner.start();

  MainWindow window(&state, opt);
  window.show();

  QObject::connect(&app, &QCoreApplication::aboutToQuit, [&]() {
    g_running = false;
    runner.stop();
  });

  const int result = app.exec();
  runner.stop();
  return result;
}
