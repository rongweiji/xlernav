# xlernav

Navigation stack for lekiwi-based robots with a split architecture:
the Raspberry Pi is a thin sensor/actuator endpoint, and WSL is the
compute server for SLAM, depth, Nav2, and visualization.

This `ros/` folder contains all ROS code, configs, and scripts. These instructions
assume you are running commands from this directory:
```
cd ros
```

## Quick Start

### Step 1: Install ROS 2

WSL (Ubuntu 24.04, ROS 2 Jazzy):
```
bash scripts/install_ros2_wsl.sh
```

Raspberry Pi (Raspberry Pi OS 64-bit, Docker-based ROS 2 Jazzy):
```
ROS_DISTRO=jazzy bash scripts/install_ros2_pi.sh
```

### Step 2: Configure Networking

- Use the same `ROS_DOMAIN_ID` on both Pi and WSL (default is `0`).
- Ensure both devices are on the same Wi-Fi network.
- Keep time in sync (NTP/chrony recommended).
- Enable WSL2 mirrored networking (Windows 11):
  - `%UserProfile%\.wslconfig`:
    ```
    [wsl2]
    networkingMode=mirrored
    ```
  - Apply: `wsl --shutdown`, then reopen WSL.
- **Default middleware: FastDDS (no extra config needed).** In every shell:
  ```
  unset RMW_IMPLEMENTATION
  unset CYCLONEDDS_URI
  export ROS_DOMAIN_ID=0
  export ROS_LOCALHOST_ONLY=0
  source /opt/ros/jazzy/setup.bash
  ```
- Optional (only if discovery is flaky): use CycloneDDS with a peer file that lists `127.0.0.1`
  and the other host’s IP, then set:
  ```
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file://$HOME/.config/cyclonedds/cyclonedds.xml
  ```
  and re-source ROS. This is not required for the default flow.

### Step 3: Verify ROS 2 Connectivity

On WSL:
```
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
```

On Pi (inside Docker container):
```
docker run --rm -it --net=host --ipc=host \
  -e ROS_DOMAIN_ID=0 \
  ros:jazzy-ros-base

source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp listener
```

If the listener receives messages, the install + network setup is working.

### Step 4: Run Camera -> Depth -> SLAM (Pi + WSL)

High-level: the Pi publishes the camera image from inside Docker, and WSL runs depth + SLAM.

1) On the Pi, identify the camera device:
```
ls /dev/video*
```
Optional (if you have `v4l2-ctl`):
```
v4l2-ctl --list-devices
```

2) On the Pi, start the ROS 2 Docker container:
```
bash scripts/run_pi_camera_docker.sh --video <INDEX> --calib cfg/camera_left.yaml
```
Note: with FastDDS (default), no target IP is needed. `--wslip` is only for explicit
CycloneDDS peer configs (the script itself does not use it).
Inside the container, run the camera node (this publishes `/image_raw` and `/camera_info`), if video1:
```
apt update
apt install -y ros-jazzy-v4l2-camera ros-jazzy-image-transport ros-jazzy-compressed-image-transport
source /opt/ros/jazzy/setup.bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p camera_info_url:=file:///root/xlernav/cfg/camera_left.yaml \
  -p video_device:=/dev/video1 \
  -p brightness:=0 \
  -p contrast:=32 \
  -p saturation:=64 \
  -p hue:=0 \
  -p gamma:=100 \
  -p gain:=0 \
  -p power_line_frequency:=1 \
  -p sharpness:=3 \
  -p backlight_compensation:=12 \
  -p white_balance_automatic:=false \
  -p white_balance_temperature:=4600 \
  -r image_raw:=/image_raw \
  -r camera_info:=/camera_info \
  -p framerate:=30

```

```
ros2 run image_transport republish --ros-args     --remap in:=/image_raw     --remap out:=/image_raw     --param in_transport:=raw     --param out_transport:=compressed
```



3) On WSL, start Depth Anything V3 (subscribes to the Pi topics):
```
bash scripts/run_wsl_depth_anything_v3_viz.sh <PI_IP>
```

4) On WSL, start ORB-SLAM3 (subscribes to RGB + depth):
```
ENABLE_VIEWER=true bash scripts/run_wsl_orb_slam3_rgbd.sh <PI_IP>
```
Note: run this after confirming the depth topic is publishing; keep this command handy for later.

## Roadmap

See `ROADMAP.md`.


// uncompressed
  Terminal 1: start republisher

unset RMW_IMPLEMENTATION CYCLONEDDS_URI
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
source /opt/ros/jazzy/setup.bash

ros2 run image_transport republish --ros-args \
  -r __node:=image_republisher_rgb \
  -p in_transport:=compressed \
  -p out_transport:=raw \
  -r in:=/image_raw \
  -r out:=/image_raw_uncompressed

  Terminal 2: keep‑alive subscriber (prevents lazy shutdown)

  unset RMW_IMPLEMENTATION CYCLONEDDS_URI
  export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
  source /opt/ros/jazzy/setup.bash

  ros2 topic echo /image_raw_uncompressed --qos-profile sensor_data
