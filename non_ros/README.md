# Non-ROS Streaming (Pi -> WSL)

Low-latency H.264 stream over RTP/UDP from the Pi, with a WSL viewer that
decodes to BGR frames and displays a live preview.

## Files
- `run_pi_stream_h264.sh`: Capture `/dev/videoX` and send H.264 RTP/UDP.
- `recv_view.py`: Receive, decode, and display frames (latest-only).
- `install_pi.sh`: Install Pi streaming dependencies.
- `install_wsl.sh`: Install WSL streaming dependencies.

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

## Notes
- The sender script auto-selects an encoder:
  - `v4l2h264enc` (preferred) -> `omxh264enc` -> `x264enc` fallback.
- `recv_view.py` uses an appsink with `max-buffers=1` and `drop=true`
  to keep the stream real-time by discarding old frames.
- Use `--log-fps` to print FPS in the terminal.
- If the UI opens but no frames arrive, add `--log-wait` to print a
  periodic "waiting for frames..." message.
