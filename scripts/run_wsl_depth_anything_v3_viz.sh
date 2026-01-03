#!/usr/bin/env bash
set -euo pipefail

# Runs Depth Anything V3 on WSL (subscribing to the Pi camera topics) and opens RViz.
#
# Usage:
#   bash scripts/run_wsl_depth_anything_v3_viz.sh <PI_IP>
#
# Optional env vars:
#   ROS_DOMAIN_ID            (default: 0)
#   INPUT_IMAGE_TOPIC        (default: /image_raw)
#   INPUT_CAMERA_INFO_TOPIC  (default: /camera_info)
#   PARAMS_FILE              (default: repo depth_anything_v3.param.yaml)
#   ONNX_PATH                (default: use params file setting)
#   PUBLISH_POINT_CLOUD      (default: 1; set 0 to disable)
#   USE_COMPRESSED           (default: 1)
#   COMPRESSED_IMAGE_TOPIC   (default: /image_raw/compressed)
#   OUTPUT_DEPTH_TOPIC       (default: /depth_anything_v3/output/depth_image)
#   OUTPUT_POINT_CLOUD_TOPIC (default: /depth_anything_v3/output/point_cloud)
#   RVIZ_CONFIG              (default: scripts/depth_viz.rviz)
#   DIAG=1                   Print ROS topic diagnostics
#   USE_CYCLONE              (default: 0)

PI_IP="${1:-${PI_IP:-192.168.50.124}}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
INPUT_IMAGE_TOPIC="${INPUT_IMAGE_TOPIC:-/image_raw}"
INPUT_CAMERA_INFO_TOPIC="${INPUT_CAMERA_INFO_TOPIC:-/camera_info}"
PARAMS_FILE="${PARAMS_FILE:-}"
ONNX_PATH="${ONNX_PATH:-}"
PUBLISH_POINT_CLOUD="${PUBLISH_POINT_CLOUD:-1}"
USE_COMPRESSED="${USE_COMPRESSED:-1}"
COMPRESSED_IMAGE_TOPIC="${COMPRESSED_IMAGE_TOPIC:-/image_raw/compressed}"
OUTPUT_DEPTH_TOPIC="${OUTPUT_DEPTH_TOPIC:-/depth_anything_v3/output/depth_image}"
OUTPUT_POINT_CLOUD_TOPIC="${OUTPUT_POINT_CLOUD_TOPIC:-/depth_anything_v3/output/point_cloud}"
RVIZ_CONFIG="${RVIZ_CONFIG:-}"
DIAG="${DIAG:-0}"
USE_CYCLONE="${USE_CYCLONE:-0}"

usage() {
  cat <<EOF
Usage: ${0} [PI_IP]

Example:
  ROS_DOMAIN_ID=0 ${0} 192.168.50.124
  ROS_DOMAIN_ID=0 ${0}    # defaults to ${PI_IP}

Overrides:
  INPUT_IMAGE_TOPIC=/image_raw
  INPUT_CAMERA_INFO_TOPIC=/camera_info
  PARAMS_FILE=/path/to/depth_anything_v3.param.yaml
  ONNX_PATH=/path/to/model.onnx
  USE_COMPRESSED=1 COMPRESSED_IMAGE_TOPIC=/image_raw/compressed
  PUBLISH_POINT_CLOUD=0
  OUTPUT_DEPTH_TOPIC=/depth_anything_v3/output/depth_image
  OUTPUT_POINT_CLOUD_TOPIC=/depth_anything_v3/output/point_cloud
  RVIZ_CONFIG=${PWD}/scripts/depth_viz.rviz
EOF
}

if [[ "${1:-}" == "-h" ]] || [[ "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

RVIZ_CONFIG="${RVIZ_CONFIG:-${SCRIPT_DIR}/depth_viz.rviz}"
WS_SETUP="${REPO_ROOT}/ros2_ws/install/setup.bash"
if [[ -z "${PARAMS_FILE}" ]]; then
  PARAMS_FILE="${REPO_ROOT}/ros2_ws/ros2-depth-anything-v3-trt/depth_anything_v3/config/depth_anything_v3.param.yaml"
fi

if [[ ! -f "/opt/ros/jazzy/setup.bash" ]]; then
  echo "Missing /opt/ros/jazzy/setup.bash. Install ROS 2 Jazzy first." >&2
  exit 1
fi
if [[ ! -f "${WS_SETUP}" ]]; then
  echo "Missing ${WS_SETUP}. Build the workspace first (colcon build)." >&2
  exit 1
fi
if [[ ! -f "${RVIZ_CONFIG}" ]]; then
  echo "Missing RViz config: ${RVIZ_CONFIG}" >&2
  exit 1
fi
if [[ ! -f "${PARAMS_FILE}" ]]; then
  echo "Missing params file: ${PARAMS_FILE}" >&2
  exit 1
fi

if [[ "${USE_CYCLONE}" == "1" ]]; then
  CONFIG_DIR="${HOME}/.config/cyclonedds"
  CONFIG_FILE="${CONFIG_DIR}/cyclonedds.xml"
  mkdir -p "${CONFIG_DIR}"
  cat > "${CONFIG_FILE}" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain>
    <General>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="127.0.0.1"/>
        <Peer address="${PI_IP}"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="file://${CONFIG_FILE}"
else
  unset RMW_IMPLEMENTATION
  unset CYCLONEDDS_URI
fi

export ROS_DOMAIN_ID
export ROS_LOCALHOST_ONLY=0

# GUI fixes for WSL/WSLg:
# - rviz2/OGRE needs an X11 window handle (Wayland can break it); force Qt to XCB by default.
# - Some WSL setups create /run/user/$UID with wrong perms; use a private XDG_RUNTIME_DIR.
if [[ -z "${QT_QPA_PLATFORM:-}" ]]; then
  export QT_QPA_PLATFORM=xcb
fi
if [[ -z "${XDG_RUNTIME_DIR:-}" ]]; then
  export XDG_RUNTIME_DIR="/tmp/runtime-${UID}"
else
  if [[ ! -d "${XDG_RUNTIME_DIR}" ]] || [[ "$(stat -c '%a' "${XDG_RUNTIME_DIR}" 2>/dev/null || echo '')" != "700" ]]; then
    export XDG_RUNTIME_DIR="/tmp/runtime-${UID}"
  fi
fi
mkdir -p "${XDG_RUNTIME_DIR}"
chmod 700 "${XDG_RUNTIME_DIR}" 2>/dev/null || true

set +u
source /opt/ros/jazzy/setup.bash
source "${WS_SETUP}"
set -u

cleanup() {
  if [[ -n "${DEPTH_PID:-}" ]] && kill -0 "${DEPTH_PID}" 2>/dev/null; then
    kill -INT "${DEPTH_PID}" 2>/dev/null || true
    wait "${DEPTH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

DEPTH_ARGS=(
  "input_image_topic:=${INPUT_IMAGE_TOPIC}"
  "input_camera_info_topic:=${INPUT_CAMERA_INFO_TOPIC}"
  "params_file:=${PARAMS_FILE}"
  "output_depth_topic:=${OUTPUT_DEPTH_TOPIC}"
  "output_point_cloud_topic:=${OUTPUT_POINT_CLOUD_TOPIC}"
)

echo "Pi peer: ${PI_IP}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-FastDDS (default)}"
echo "Subscribing:"
echo "  image:       ${INPUT_IMAGE_TOPIC}"
if [[ "${USE_COMPRESSED}" == "1" ]]; then
  echo "  image (compressed): ${COMPRESSED_IMAGE_TOPIC}"
fi
echo "  camera_info: ${INPUT_CAMERA_INFO_TOPIC}"
echo "Point cloud: ${PUBLISH_POINT_CLOUD}"
echo "RViz config: ${RVIZ_CONFIG}"
echo ""

if [[ "${USE_COMPRESSED}" == "1" ]]; then
  DEPTH_CMD=(ros2 run depth_anything_v3 depth_anything_v3_main --ros-args --params-file "${PARAMS_FILE}")
  if [[ -n "${ONNX_PATH}" ]]; then
    DEPTH_CMD+=(-p "onnx_path:=${ONNX_PATH}")
  fi
  if [[ "${PUBLISH_POINT_CLOUD}" == "0" ]]; then
    DEPTH_CMD+=(-p "publish_point_cloud:=false" -p "point_cloud_downsample_factor:=0" -p "colorize_point_cloud:=false")
  fi
  DEPTH_CMD+=(
    --remap "/depth_anything_v3/input/image:=/unused"
    --remap "/depth_anything_v3/input/image/compressed:=${COMPRESSED_IMAGE_TOPIC}"
    --remap "/depth_anything_v3/input/camera_info:=${INPUT_CAMERA_INFO_TOPIC}"
    --remap "/depth_anything_v3/output/depth_image:=${OUTPUT_DEPTH_TOPIC}"
    --remap "/depth_anything_v3/output/point_cloud:=${OUTPUT_POINT_CLOUD_TOPIC}"
  )
  "${DEPTH_CMD[@]}" &
else
  ros2 launch depth_anything_v3 depth_anything_v3.launch.py "${DEPTH_ARGS[@]}" &
fi
DEPTH_PID="$!"

if [[ "${DIAG}" == "1" ]]; then
  DEBUG_IMAGE_TOPIC="/depth_anything_v3/output/depth_image_debug"
  DIAG_IMAGE_TOPIC="${INPUT_IMAGE_TOPIC}"
  if [[ "${USE_COMPRESSED}" == "1" ]]; then
    DIAG_IMAGE_TOPIC="${COMPRESSED_IMAGE_TOPIC}"
  fi
  echo ""
  echo "Diagnostics (DIAG=1)"
  echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
  echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-unset}"
  echo ""
  echo "ros2 topic info (publishers/subscribers):"
  ros2 topic info "${DIAG_IMAGE_TOPIC}" || true
  ros2 topic info "${INPUT_CAMERA_INFO_TOPIC}" || true
  ros2 topic info "${DEBUG_IMAGE_TOPIC}" || true
  echo ""
fi

if rviz2 -d "${RVIZ_CONFIG}"; then
  exit 0
fi

echo "" >&2
echo "rviz2 failed to start (common on WSL graphics setups)." >&2
echo "If you want to try software rendering:" >&2
echo "  QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 bash ${0} ${PI_IP}" >&2
echo "" >&2

if command -v rqt_image_view >/dev/null 2>&1; then
  echo "Falling back to rqt_image_view..." >&2
  rqt_image_view "${INPUT_IMAGE_TOPIC}" &
  CAM_VIEW_PID="$!"
  rqt_image_view "/depth_anything_v3/output/depth_image_debug" &
  DEPTH_VIEW_PID="$!"
  wait -n "${CAM_VIEW_PID}" "${DEPTH_VIEW_PID}"
else
  echo "" >&2
  echo "Install rqt image viewer and retry:" >&2
  echo "  sudo apt install -y ros-jazzy-rqt-image-view" >&2
  exit 1
fi
