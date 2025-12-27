#!/usr/bin/env bash
set -euo pipefail

# Run ROS 2 Jazzy camera publisher in Docker on Raspberry Pi (USB webcam, v4l2).
# Configure via env vars:
#   WSL_IP        (required) WSL host IP for CycloneDDS static peer.
#   VIDEO_DEVICE  (default: /dev/video0) camera device on Pi.
#   ROS_DOMAIN_ID (default: 0)
#   ROS_IMAGE_TOPIC (default: /image_raw)
#   ROS_INFO_TOPIC  (default: /camera_info)

WSL_IP="${WSL_IP:-}"
VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video0}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
ROS_IMAGE_TOPIC="${ROS_IMAGE_TOPIC:-/image_raw}"
ROS_INFO_TOPIC="${ROS_INFO_TOPIC:-/camera_info}"

if [[ -z "${WSL_IP}" ]]; then
  echo "WSL_IP is required. Example: WSL_IP=192.168.50.219 ${0}" >&2
  exit 1
fi

CONFIG_DIR="${HOME}/.config/cyclonedds"
CONFIG_FILE="${CONFIG_DIR}/cyclonedds.xml"

mkdir -p "${CONFIG_DIR}"
cat > "${CONFIG_FILE}" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>wlan0</NetworkInterfaceAddress>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="${WSL_IP}"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

echo "Using WSL_IP=${WSL_IP}"
echo "Using VIDEO_DEVICE=${VIDEO_DEVICE}"
echo "Using ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "Publishing topics:"
echo "  ${ROS_IMAGE_TOPIC}"
echo "  ${ROS_INFO_TOPIC}"

docker run --rm -it --net=host --ipc=host --privileged \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  -e ROS_LOCALHOST_ONLY=0 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI="file:///root/.config/cyclonedds/cyclonedds.xml" \
  -v "${CONFIG_DIR}:/root/.config/cyclonedds" \
  ros:jazzy-ros-base bash -lc "\
    apt update && \
    apt install -y ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-v4l2-camera && \
    source /opt/ros/jazzy/setup.bash && \
    ros2 run v4l2_camera v4l2_camera_node --ros-args \
      -p video_device:=${VIDEO_DEVICE} \
      -r image_raw:=${ROS_IMAGE_TOPIC} \
      -r camera_info:=${ROS_INFO_TOPIC} \
  "
