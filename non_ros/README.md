# Non-ROS Streaming (Pi -> WSL)

Low-latency H.264 stream over RTP/UDP from the Pi, with a WSL viewer that
decodes to BGR frames and displays a live preview.

## Files
- `run_pi_stream_h264.sh`: Capture `/dev/videoX` and send H.264 RTP/UDP.
- `recv_view.py`: Receive, decode, and display frames (latest-only).
- `run_depth_stream_cpp.sh`: C++ depth viewer using the TensorRT C++ engine.
- `run_orbslam3_stream.sh`: RGB + depth + ORB-SLAM3 viewer (non-ROS).
- `run_voxel_stream.sh`: RGB + depth + ORB-SLAM3 + voxel occupancy viewer (non-ROS).
- `install_pi.sh`: Install Pi streaming dependencies.
- `install_wsl.sh`: Install WSL streaming dependencies.
- `depth_stream_cpp/`: C++ depth viewer sources (builds on demand).
- `orbslam3_stream/`: ORB-SLAM3 streaming + undistort sources.
- `voxel_stream/`: Voxel occupancy viewer sources.

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
bash non_ros/run_pi_stream_h264.sh --host 192.168.50.219 --device /dev/video1 --encoder x264
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

UI modes:
- Default (ORB-SLAM3 viewer): `--ui orbslam` (default)
- Custom UI (single OpenCV window with RGB + depth + 3D pose): `--ui custom`
- Both viewers: `--ui both` (may be unreliable on WSL)

Custom UI options:
- `--show-fps` overlays rx/depth FPS on the custom UI
- `--show-raw` shows the raw frame instead of the rectified frame
Use the mouse in the 3D panel to rotate/zoom the pose view.
The custom UI uses CPU rendering for the 3D panel to avoid OpenGL crashes on WSL.

If the undistorted view is black, try `--no-projection` to ignore
`projection_matrix` and use intrinsics instead.
Defaults:
- Vocabulary: `non_ros/orbslam3_stream/vocabulary/ORBvoc.txt`
- Calibration: `non_ros/orbslam3_stream/config/camera_left.yaml`
- Depth engine: `non_ros/depth_stream_cpp/models/DA3METRIC-LARGE.trt10.engine`
First run will clone ORB-SLAM3 into `non_ros/orbslam3_stream/vendor`.

### 5) On WSL (Voxel map GUI)
```
bash non_ros/run_voxel_stream.sh --port 5600
```
This builds ORB-SLAM3 (if missing) and launches a Qt window with:
- RGB panel
- Depth panel
- 3D voxel + pose path view
Defaults:
- Vocabulary: `non_ros/orbslam3_stream/vocabulary/ORBvoc.txt`
- Calibration: `non_ros/orbslam3_stream/config/camera_left.yaml`
- Depth engine: `non_ros/depth_stream_cpp/models/DA3METRIC-LARGE.trt10.engine`
Voxel defaults:
- `--voxel-size 0.1` meters
- `--grid-x 50 --grid-y 50 --grid-z 30` (fixed local grid dimensions)
- `--stride 4`
- `--max-depth 6.0`
- `--decay-sec 120`
UI options:
- `--no-preview` hides the RGB/depth panels
- `--show-raw` shows raw RGB instead of the rectified frame

## Notes
- The sender script auto-selects an encoder:
  - `v4l2h264enc` (preferred) -> `omxh264enc` -> `x264enc` fallback.
- `recv_view.py` uses an appsink with `max-buffers=1` and `drop=true`
  to keep the stream real-time by discarding old frames.
- Use `--log-fps` to print FPS in the terminal.
- Use `--log-timing` to print per-stage timing (decode/remap/depth/SLAM/voxel/UI).
- If the UI opens but no frames arrive, add `--log-wait` to print a
  periodic "waiting for frames..." message.
- Depth inference uses the local TensorRT engine under `non_ros/depth_stream_cpp/models`.
- On Ubuntu 24.04, `libpangolin-dev` may be missing from apt; if the ORB-SLAM3 build fails,
  install Pangolin from source.
