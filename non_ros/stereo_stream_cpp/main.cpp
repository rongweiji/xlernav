#include "fps_counter.hpp"

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#include <opencv2/opencv.hpp>

#include <QApplication>
#include <QCoreApplication>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QPixmap>
#include <QString>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

struct Options {
  int port_left = 5600;
  int port_right = 5601;
  int payload_left = 96;
  int payload_right = 97;
  std::string decoder = "avdec_h264";
  int latency_ms = 30;
  int sync_threshold_ms = 40;
  int pull_timeout_ms = 5;
  double fps_interval = 1.0;
  bool log_wait = false;
  double wait_log_interval = 5.0;
};

static void print_usage(const char * prog)
{
  std::cout << "Usage: " << prog << " [options]\n"
            << "  --port-left N           UDP port for left stream (default 5600)\n"
            << "  --port-right N          UDP port for right stream (default 5601)\n"
            << "  --payload-left N        RTP payload type for left (default 96)\n"
            << "  --payload-right N       RTP payload type for right (default 97)\n"
            << "  --decoder NAME          GStreamer decoder (default avdec_h264)\n"
            << "  --latency-ms N          RTP jitterbuffer latency (default 30)\n"
            << "  --sync-threshold-ms N   Max timestamp delta to sync (default 40)\n"
            << "  --pull-timeout-ms N     Appsink pull timeout in ms (default 5)\n"
            << "  --fps-interval SEC      FPS update interval\n"
            << "  --log-wait              Log when waiting for frames\n"
            << "  --wait-log-interval SEC Seconds between wait logs\n";
}

static Options parse_args(int argc, char ** argv)
{
  Options opt;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for " + arg);
      }
      return argv[++i];
    };

    if (arg == "--port-left") {
      opt.port_left = std::stoi(next());
    } else if (arg == "--port-right") {
      opt.port_right = std::stoi(next());
    } else if (arg == "--payload-left") {
      opt.payload_left = std::stoi(next());
    } else if (arg == "--payload-right") {
      opt.payload_right = std::stoi(next());
    } else if (arg == "--decoder") {
      opt.decoder = next();
    } else if (arg == "--latency-ms") {
      opt.latency_ms = std::stoi(next());
    } else if (arg == "--sync-threshold-ms") {
      opt.sync_threshold_ms = std::stoi(next());
    } else if (arg == "--pull-timeout-ms") {
      opt.pull_timeout_ms = std::stoi(next());
    } else if (arg == "--fps-interval") {
      opt.fps_interval = std::stod(next());
    } else if (arg == "--log-wait") {
      opt.log_wait = true;
    } else if (arg == "--wait-log-interval") {
      opt.wait_log_interval = std::stod(next());
    } else if (arg == "-h" || arg == "--help") {
      print_usage(argv[0]);
      std::exit(0);
    } else {
      throw std::runtime_error("Unknown option: " + arg);
    }
  }

  return opt;
}

static int64_t now_ns()
{
  const auto now = std::chrono::steady_clock::now();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
}

struct Frame {
  cv::Mat image;
  int64_t pts_ns = -1;
  int64_t recv_ns = -1;
  bool pts_valid = false;
};

static std::string build_pipeline(int port, int payload, const std::string & decoder, int latency_ms)
{
  return
    "udpsrc port=" + std::to_string(port) +
    " caps=\"application/x-rtp,media=video,encoding-name=H264,payload=" +
    std::to_string(payload) + "\" ! "
    "rtpjitterbuffer latency=" + std::to_string(latency_ms) + " ! "
    "rtph264depay ! h264parse ! " + decoder + " ! "
    "videoconvert ! video/x-raw,format=BGR ! "
    "appsink name=sink max-buffers=1 drop=true sync=false";
}

class GstReceiver {
public:
  GstReceiver(int port, int payload, std::string decoder, int latency_ms)
  : port_(port),
    payload_(payload),
    decoder_(std::move(decoder)),
    latency_ms_(latency_ms)
  {
  }

  ~GstReceiver() { Close(); }

  bool Open()
  {
    GError * error = nullptr;
    const std::string pipeline_desc = build_pipeline(port_, payload_, decoder_, latency_ms_);
    pipeline_ = gst_parse_launch(pipeline_desc.c_str(), &error);
    if (!pipeline_) {
      std::cerr << "Failed to create GStreamer pipeline: "
                << (error ? error->message : "unknown error") << "\n";
      if (error) {
        g_error_free(error);
      }
      return false;
    }
    if (error) {
      std::cerr << "GStreamer pipeline warning: " << error->message << "\n";
      g_error_free(error);
    }

    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
    if (!appsink_) {
      std::cerr << "Failed to find appsink in pipeline.\n";
      Close();
      return false;
    }

    gst_app_sink_set_drop(GST_APP_SINK(appsink_), true);
    gst_app_sink_set_max_buffers(GST_APP_SINK(appsink_), 1);

    const GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
      std::cerr << "Failed to start GStreamer pipeline.\n";
      Close();
      return false;
    }

    return true;
  }

  void Close()
  {
    if (pipeline_) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (appsink_) {
      gst_object_unref(appsink_);
      appsink_ = nullptr;
    }
    if (pipeline_) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
  }

  bool Read(Frame & out, int timeout_ms)
  {
    if (!appsink_) {
      return false;
    }

    const GstClockTime timeout_ns = static_cast<GstClockTime>(timeout_ms) * GST_MSECOND;
    GstSample * sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_), timeout_ns);
    if (!sample) {
      return false;
    }

    GstCaps * caps = gst_sample_get_caps(sample);
    GstBuffer * buffer = gst_sample_get_buffer(sample);
    if (!caps || !buffer) {
      gst_sample_unref(sample);
      return false;
    }

    GstVideoInfo info;
    if (!gst_video_info_from_caps(&info, caps)) {
      gst_sample_unref(sample);
      return false;
    }

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      gst_sample_unref(sample);
      return false;
    }

    const int width = static_cast<int>(GST_VIDEO_INFO_WIDTH(&info));
    const int height = static_cast<int>(GST_VIDEO_INFO_HEIGHT(&info));
    const int stride = static_cast<int>(GST_VIDEO_INFO_PLANE_STRIDE(&info, 0));
    cv::Mat frame(height, width, CV_8UC3, map.data, stride);
    out.image = frame.clone();
    gst_buffer_unmap(buffer, &map);

    out.recv_ns = now_ns();
    const GstClockTime pts = GST_BUFFER_PTS(buffer);
    out.pts_valid = (pts != GST_CLOCK_TIME_NONE);
    out.pts_ns = out.pts_valid ? static_cast<int64_t>(pts) : -1;

    gst_sample_unref(sample);
    return true;
  }

private:
  int port_;
  int payload_;
  std::string decoder_;
  int latency_ms_;
  GstElement * pipeline_ = nullptr;
  GstElement * appsink_ = nullptr;
};

struct SharedState {
  std::mutex mutex;
  cv::Mat left;
  cv::Mat right;
  double left_fps = 0.0;
  double right_fps = 0.0;
  double sync_delta_ms = 0.0;
  uint64_t frame_id = 0;
};

static QImage mat_to_qimage(const cv::Mat & bgr)
{
  if (bgr.empty()) {
    return QImage();
  }
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  return QImage(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888).copy();
}

class StereoWindow : public QWidget {
public:
  StereoWindow(SharedState * state, int sync_threshold_ms, int refresh_ms)
  : state_(state),
    sync_threshold_ms_(sync_threshold_ms)
  {
    setWindowTitle("Stereo Stream");

    left_fps_label_ = new QLabel("Left FPS: --");
    right_fps_label_ = new QLabel("Right FPS: --");
    sync_label_ = new QLabel("Sync delta: -- ms");

    left_image_ = new QLabel();
    right_image_ = new QLabel();
    left_image_->setMinimumSize(320, 240);
    right_image_->setMinimumSize(320, 240);
    left_image_->setAlignment(Qt::AlignCenter);
    right_image_->setAlignment(Qt::AlignCenter);

    auto * left_layout = new QVBoxLayout();
    left_layout->addWidget(left_fps_label_);
    left_layout->addWidget(left_image_);

    auto * right_layout = new QVBoxLayout();
    right_layout->addWidget(right_fps_label_);
    right_layout->addWidget(right_image_);

    auto * row_layout = new QHBoxLayout();
    row_layout->addLayout(left_layout);
    row_layout->addLayout(right_layout);

    auto * main_layout = new QVBoxLayout();
    main_layout->addLayout(row_layout);
    main_layout->addWidget(sync_label_);

    setLayout(main_layout);

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this]() { refresh_ui(); });
    timer_->start(refresh_ms);
  }

private:
  void refresh_ui()
  {
    cv::Mat left;
    cv::Mat right;
    double left_fps = 0.0;
    double right_fps = 0.0;
    double sync_delta_ms = 0.0;
    uint64_t frame_id = 0;

    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      frame_id = state_->frame_id;
      if (frame_id == last_frame_id_) {
        return;
      }
      left = state_->left.clone();
      right = state_->right.clone();
      left_fps = state_->left_fps;
      right_fps = state_->right_fps;
      sync_delta_ms = state_->sync_delta_ms;
    }

    last_frame_id_ = frame_id;

    if (!left.empty()) {
      const QImage left_img = mat_to_qimage(left);
      if (!left_img.isNull()) {
        const QPixmap pix = QPixmap::fromImage(left_img)
                              .scaled(left_image_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
        left_image_->setPixmap(pix);
      }
    }

    if (!right.empty()) {
      const QImage right_img = mat_to_qimage(right);
      if (!right_img.isNull()) {
        const QPixmap pix = QPixmap::fromImage(right_img)
                              .scaled(right_image_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
        right_image_->setPixmap(pix);
      }
    }

    left_fps_label_->setText(QString("Left FPS: %1").arg(left_fps, 0, 'f', 1));
    right_fps_label_->setText(QString("Right FPS: %1").arg(right_fps, 0, 'f', 1));
    sync_label_->setText(QString("Sync delta: %1 ms (threshold %2 ms)")
                         .arg(sync_delta_ms, 0, 'f', 2)
                         .arg(sync_threshold_ms_));
  }

  SharedState * state_;
  int sync_threshold_ms_;
  QLabel * left_fps_label_ = nullptr;
  QLabel * right_fps_label_ = nullptr;
  QLabel * sync_label_ = nullptr;
  QLabel * left_image_ = nullptr;
  QLabel * right_image_ = nullptr;
  QTimer * timer_ = nullptr;
  uint64_t last_frame_id_ = 0;
};

static void capture_loop(const Options & opt, SharedState * state, std::atomic<bool> * running)
{
  GstReceiver left_rx(opt.port_left, opt.payload_left, opt.decoder, opt.latency_ms);
  GstReceiver right_rx(opt.port_right, opt.payload_right, opt.decoder, opt.latency_ms);

  if (!left_rx.Open() || !right_rx.Open()) {
    running->store(false);
    return;
  }

  xlernav::FpsCounter left_fps(opt.fps_interval);
  xlernav::FpsCounter right_fps(opt.fps_interval);

  Frame left_frame;
  Frame right_frame;
  bool have_left = false;
  bool have_right = false;

  auto last_wait_log = std::chrono::steady_clock::now();
  const int64_t sync_threshold_ns = static_cast<int64_t>(opt.sync_threshold_ms) * 1000000LL;

  while (running->load()) {
    Frame candidate;
    bool got_left = left_rx.Read(candidate, opt.pull_timeout_ms);
    if (got_left) {
      left_frame = std::move(candidate);
      have_left = true;
      left_fps.tick();
    }

    bool got_right = right_rx.Read(candidate, opt.pull_timeout_ms);
    if (got_right) {
      right_frame = std::move(candidate);
      have_right = true;
      right_fps.tick();
    }

    if (!got_left && !got_right && opt.log_wait) {
      const auto now = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(now - last_wait_log).count();
      if (elapsed >= opt.wait_log_interval) {
        std::cout << "[stream] waiting for frames...\n";
        last_wait_log = now;
      }
    }

    if (!have_left || !have_right) {
      continue;
    }

    const bool use_pts = left_frame.pts_valid && right_frame.pts_valid;
    const int64_t left_ts = use_pts ? left_frame.pts_ns : left_frame.recv_ns;
    const int64_t right_ts = use_pts ? right_frame.pts_ns : right_frame.recv_ns;
    const int64_t delta = std::llabs(left_ts - right_ts);
    if (delta > sync_threshold_ns) {
      if (left_ts < right_ts) {
        have_left = false;
      } else {
        have_right = false;
      }
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->left = left_frame.image;
      state->right = right_frame.image;
      state->left_fps = left_fps.fps();
      state->right_fps = right_fps.fps();
      state->sync_delta_ms = static_cast<double>(delta) / 1000000.0;
      state->frame_id++;
    }

    have_left = false;
    have_right = false;
  }
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

  gst_init(nullptr, nullptr);

  SharedState state;
  std::atomic<bool> running(true);
  std::thread worker(capture_loop, opt, &state, &running);

  QApplication app(argc, argv);
  StereoWindow window(&state, opt.sync_threshold_ms, 33);
  window.show();

  QObject::connect(&app, &QCoreApplication::aboutToQuit, [&running]() {
    running.store(false);
  });

  const int ret = app.exec();

  running.store(false);
  if (worker.joinable()) {
    worker.join();
  }

  return ret;
}
