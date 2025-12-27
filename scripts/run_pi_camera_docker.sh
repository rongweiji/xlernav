#!/usr/bin/env bash
set -euo pipefail

# Run ROS 2 Jazzy camera publisher in Docker on Raspberry Pi (USB webcam, v4l2).
# Configure via flags or env vars:
#   --wslip IP          (or WSL_IP) required: WSL host IP for CycloneDDS static peer.
#   --video INDEX|PATH  (or VIDEO_DEVICE) default: /dev/video0
#   --brightness VALUE  (or BRIGHTNESS) optional: camera brightness
#   ROS_DOMAIN_ID       default: 0
#   ROS_IMAGE_TOPIC     default: /image_raw
#   ROS_INFO_TOPIC      default: /camera_info

WSL_IP="${WSL_IP:-}"
VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video0}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
ROS_IMAGE_TOPIC="${ROS_IMAGE_TOPIC:-/image_raw}"
ROS_INFO_TOPIC="${ROS_INFO_TOPIC:-/camera_info}"
BRIGHTNESS="${BRIGHTNESS:-}"

usage() {
  cat <<EOF
Usage: ${0} --wslip <IP> [--video <INDEX|/dev/videoX>]

Examples:
  ${0} --wslip 192.168.50.219 --video 0
  ${0} --wslip 192.168.50.219 --video /dev/video2
  ${0} --wslip 192.168.50.219 --video 0 --brightness 1
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --wslip)
      WSL_IP="${2:-}"
      shift 2
      ;;
    --video)
      VIDEO_DEVICE="${2:-}"
      shift 2
      ;;
    --brightness)
      BRIGHTNESS="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ -n "${VIDEO_DEVICE}" ]] && [[ "${VIDEO_DEVICE}" != /dev/video* ]]; then
  VIDEO_DEVICE="/dev/video${VIDEO_DEVICE}"
fi

if [[ -z "${WSL_IP}" ]]; then
  echo "WSL_IP is required. Example: WSL_IP=192.168.50.219 ${0}" >&2
  exit 1
fi

if [[ -z "${BRIGHTNESS}" ]] && command -v v4l2-ctl >/dev/null 2>&1; then
  BRIGHTNESS="$(v4l2-ctl --device="${VIDEO_DEVICE}" --list-ctrls 2>/dev/null | \
    awk '/brightness/ {for (i=1;i<=NF;i++) if ($i ~ /^min=/) {sub(/^min=/,"",$i); print $i; exit}}')"
fi
if [[ -z "${BRIGHTNESS}" ]]; then
  BRIGHTNESS="1"
  echo "BRIGHTNESS not set; defaulting to ${BRIGHTNESS}" >&2
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
echo "Using BRIGHTNESS=${BRIGHTNESS}"
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
      -p brightness:=${BRIGHTNESS} \
      -r image_raw:=${ROS_IMAGE_TOPIC} \
      -r camera_info:=${ROS_INFO_TOPIC} \
  "
