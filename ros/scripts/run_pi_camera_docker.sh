#!/usr/bin/env bash
set -euo pipefail

# Run ROS 2 Jazzy Docker environment on Raspberry Pi (USB webcam, v4l2).
# This script only prepares and opens the container; you run ROS 2 commands manually.
# Configure via flags or env vars:
#   --wslip IP          (or WSL_IP) optional: WSL host IP (not needed for FastDDS default).
#   --video INDEX|PATH  (or VIDEO_DEVICE) default: /dev/video0
#   --brightness VALUE  (or BRIGHTNESS) optional: camera brightness
#   --repo PATH         (or REPO_DIR) optional: repo path to mount into container
#   --calib PATH        (or CALIB_FILE) optional: calibration file path (host)
#   ROS_DOMAIN_ID       default: 0
#   ROS_IMAGE_TOPIC     default: /image_raw
#   ROS_INFO_TOPIC      default: /camera_info

WSL_IP="${WSL_IP:-}"
VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video0}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
ROS_IMAGE_TOPIC="${ROS_IMAGE_TOPIC:-/image_raw}"
ROS_INFO_TOPIC="${ROS_INFO_TOPIC:-/camera_info}"
BRIGHTNESS="${BRIGHTNESS:-}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
REPO_DIR="${REPO_DIR:-${REPO_ROOT}}"
CALIB_FILE="${CALIB_FILE:-}"

usage() {
  cat <<EOF
Usage: ${0} --wslip <IP> [--video <INDEX|/dev/videoX>]

Examples:
  ${0} --video 0
  ${0} --video /dev/video2
  ${0} --video 0 --brightness 1
  ${0} --video 0 --repo ~/xlernav/ros --calib cfg/camera_left.yaml
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
    --repo)
      REPO_DIR="${2:-}"
      shift 2
      ;;
    --calib)
      CALIB_FILE="${2:-}"
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

echo "Using WSL_IP=${WSL_IP:-unset}"
echo "Using VIDEO_DEVICE=${VIDEO_DEVICE}"
echo "Using ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "Publishing topics:"
echo "  ${ROS_IMAGE_TOPIC}"
echo "  ${ROS_INFO_TOPIC}"
if [[ -n "${BRIGHTNESS}" ]]; then
  echo "Using BRIGHTNESS=${BRIGHTNESS}"
fi
if [[ -n "${CALIB_FILE}" ]]; then
  echo "Using CALIB_FILE=${CALIB_FILE}"
fi

echo ""
echo "Container will start now. Inside it, run:"
echo "  apt update"
echo "  apt install -y ros-jazzy-v4l2-camera"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  ros2 run v4l2_camera v4l2_camera_node --ros-args \\"
if [[ -n "${CALIB_FILE}" ]]; then
  echo "    -p camera_info_url:=file:///root/xlernav/${CALIB_FILE} \\"
fi
echo "    -p video_device:=${VIDEO_DEVICE} \\"
if [[ -n "${BRIGHTNESS}" ]]; then
  echo "    -p brightness:=${BRIGHTNESS} \\"
fi
echo "    -r image_raw:=${ROS_IMAGE_TOPIC} \\"
echo "    -r camera_info:=${ROS_INFO_TOPIC}"

docker run --rm -it --net=host --ipc=host --privileged \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  -e ROS_LOCALHOST_ONLY=0 \
  -v "${REPO_DIR}:/root/xlernav" \
  ros:jazzy-ros-base bash
