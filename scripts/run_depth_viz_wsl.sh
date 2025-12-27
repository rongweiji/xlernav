#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${REPO_DIR}/ros2_ws"
PARAM_FILE="${WS_DIR}/ros2-depth-anything-v3-trt/depth_anything_v3/config/depth_anything_v3.param.yaml"

# Defaults (override via env vars).
RAW_IMAGE_TOPIC="${RAW_IMAGE_TOPIC:-/image_raw}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera_info}"
CLEANUP_EXISTING="${CLEANUP_EXISTING:-1}"
RESET_RVIZ_CONFIG="${RESET_RVIZ_CONFIG:-0}"

cleanup() {
  local pids=("$@")
  for pid in "${pids[@]}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
      kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
}

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not found in PATH. Source ROS 2 first." >&2
  exit 1
fi

if [[ ! -f "${PARAM_FILE}" ]]; then
  echo "Param file not found: ${PARAM_FILE}" >&2
  exit 1
fi

# Avoid ~/.ros/log permission issues on WSL/Windows mounts
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"
mkdir -p "${ROS_LOG_DIR}"

set +u
source /opt/ros/jazzy/setup.bash
source "${WS_DIR}/install/setup.bash"
set -u

if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
  export ROS_DOMAIN_ID=0
fi
if [[ -z "${ROS_LOCALHOST_ONLY:-}" ]]; then
  export ROS_LOCALHOST_ONLY=0
fi
if [[ -z "${RMW_IMPLEMENTATION:-}" ]]; then
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
fi
if [[ -z "${CYCLONEDDS_URI:-}" ]] && [[ -f "${HOME}/.config/cyclonedds/cyclonedds.xml" ]]; then
  export CYCLONEDDS_URI="file://${HOME}/.config/cyclonedds/cyclonedds.xml"
fi

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-unset}"

echo "Using RAW_IMAGE_TOPIC=${RAW_IMAGE_TOPIC}"
echo "Using CAMERA_INFO_TOPIC=${CAMERA_INFO_TOPIC}"

if [[ "${CLEANUP_EXISTING}" -eq 1 ]]; then
  echo "Cleaning up existing processes..."
  pkill -f depth_anything_v3_main >/dev/null 2>&1 || true
  pkill -f rviz2 >/dev/null 2>&1 || true
fi

if [[ "${RESET_RVIZ_CONFIG}" -eq 1 ]]; then
  rm -rf "${HOME}/.rviz2"
fi

echo "Starting Depth Anything V3 node..."
ros2 run depth_anything_v3 depth_anything_v3_main --ros-args \
  --params-file "${PARAM_FILE}" \
  -r __node:=depth_anything_v3 \
  -p enable_debug:=true \
  -p write_colormap:=true \
  -p debug_filepath:=/tmp/depth_anything_v3_debug/ \
  -r /depth_anything_v3/input/image:="${RAW_IMAGE_TOPIC}" \
  -r /depth_anything_v3/input/image/compressed:="${RAW_IMAGE_TOPIC}/compressed" \
  -r /depth_anything_v3/input/camera_info:="${CAMERA_INFO_TOPIC}" &
depth_pid="$!"

trap 'cleanup "${depth_pid}"' EXIT INT TERM

# Give the node a moment to initialize and build TensorRT engine
echo "Waiting for depth node to initialize..."
sleep 3

echo ""
echo "==================================================="
echo "Depth Anything V3 Visualization Running"
echo "==================================================="
echo "Topics:"
echo "  - Depth output: /depth_anything_v3/output/depth_image_debug"
echo "  - Raw image: ${RAW_IMAGE_TOPIC}"
echo ""
echo "Opening depth viewer... Press Q to quit."
echo "==================================================="
echo ""

# Use software rendering to avoid GLX issues in WSL
export LIBGL_ALWAYS_SOFTWARE=1

# Run the simple depth viewer
python3 "${SCRIPT_DIR}/depth_viewer.py" /depth_anything_v3/output/depth_image_debug &
viewer_pid="$!"

trap 'cleanup "${depth_pid}" "${viewer_pid}"; exit 0' EXIT INT TERM

wait "${viewer_pid}"
