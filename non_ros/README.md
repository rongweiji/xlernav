# Non-ROS Streaming (Pi -> WSL)

Low-latency H.264 stream over RTP/UDP from the Pi, with a WSL viewer that
decodes to BGR frames and displays a live preview.

## Files
- `run_pi_stream_h264.sh`: Capture `/dev/videoX` and send H.264 RTP/UDP.
- `recv_view.py`: Receive, decode, and display frames (latest-only).
- `run_depth_stream_cpp.sh`: C++ depth viewer using the TensorRT C++ engine.
- `run_orbslam3_stream.sh`: RGB + depth + ORB-SLAM3 viewer (non-ROS).
- `install_pi.sh`: Install Pi streaming dependencies.
- `install_wsl.sh`: Install WSL streaming dependencies.
- `depth_stream_cpp/`: C++ depth viewer sources (builds on demand).
- `orbslam3_stream/`: ORB-SLAM3 streaming + undistort sources.

## Quick Start

### 0) Install dependencies

On the Pi:
```
bash non_ros/install_pi.sh
```

On WSL:
```
bash non_ros/install_wsl.sh
```

### 1) On the Pi (sender)

```
  v4l2-ctl --list-devices 
```


```
bash non_ros/run_pi_stream_h264.sh --host <WSL_IP> --device /dev/video0
```

Optional flags:
```
--width 640 --height 480 --fps 30 --bitrate 2000 --port 5600 --encoder auto
```

### 2) On WSL (receiver GUI)
```
python3 non_ros/recv_view.py --port 5600 --show-fps
```

Press `q` to quit the viewer.

### 3) On WSL (RGB + Depth GUI)
```
bash non_ros/run_depth_stream_cpp.sh --port 5600 --show-fps
```
This uses the C++ Depth Anything TensorRT engine (no Python bindings).
Engine defaults to `non_ros/depth_stream_cpp/models/DA3METRIC-LARGE.trt10.engine`.
Camera calibration defaults to `non_ros/depth_stream_cpp/config/camera_left.yaml`.

### 4) On WSL (ORB-SLAM3 GUI)
```
bash non_ros/install_wsl.sh
bash non_ros/run_orbslam3_stream.sh --port 5600
```
This builds ORB-SLAM3 (if missing) and launches the Pangolin viewer.
Undistort is applied before depth + SLAM.
Optional: add `--show-fps` to open an undistorted preview window.
Use `--show-raw` to see raw vs undistorted side-by-side.
If the undistorted view is black, try `--no-projection` to ignore
`projection_matrix` and use intrinsics instead.
Defaults:
- Vocabulary: `non_ros/orbslam3_stream/vocabulary/ORBvoc.txt`
- Calibration: `non_ros/orbslam3_stream/config/camera_left.yaml`
- Depth engine: `non_ros/depth_stream_cpp/models/DA3METRIC-LARGE.trt10.engine`
First run will clone ORB-SLAM3 into `non_ros/orbslam3_stream/vendor`.

## Notes
- The sender script auto-selects an encoder:
  - `v4l2h264enc` (preferred) -> `omxh264enc` -> `x264enc` fallback.
- `recv_view.py` uses an appsink with `max-buffers=1` and `drop=true`
  to keep the stream real-time by discarding old frames.
- Use `--log-fps` to print FPS in the terminal.
- If the UI opens but no frames arrive, add `--log-wait` to print a
  periodic "waiting for frames..." message.
- Depth inference uses the local TensorRT engine under `non_ros/depth_stream_cpp/models`.
