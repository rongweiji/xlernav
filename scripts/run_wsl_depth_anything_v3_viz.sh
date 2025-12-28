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
#   PARAMS_FILE              (default: depth_anything_v3 default params file)
#   RVIZ_CONFIG              (default: scripts/depth_viz.rviz)
#   DIAG=1                   Print ROS topic diagnostics

PI_IP="${1:-${PI_IP:-192.168.50.124}}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
INPUT_IMAGE_TOPIC="${INPUT_IMAGE_TOPIC:-/image_raw}"
INPUT_CAMERA_INFO_TOPIC="${INPUT_CAMERA_INFO_TOPIC:-/camera_info}"
PARAMS_FILE="${PARAMS_FILE:-}"
RVIZ_CONFIG="${RVIZ_CONFIG:-}"
DIAG="${DIAG:-0}"

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

# CycloneDDS static peer config (recommended for WSL2 <-> Pi)
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
)
if [[ -n "${PARAMS_FILE}" ]]; then
  DEPTH_ARGS+=("params_file:=${PARAMS_FILE}")
fi

echo "Pi peer: ${PI_IP}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "Subscribing:"
echo "  image:       ${INPUT_IMAGE_TOPIC}"
echo "  camera_info: ${INPUT_CAMERA_INFO_TOPIC}"
echo "RViz config: ${RVIZ_CONFIG}"
echo ""

ros2 launch depth_anything_v3 depth_anything_v3.launch.py "${DEPTH_ARGS[@]}" &
DEPTH_PID="$!"

if [[ "${DIAG}" == "1" ]]; then
  DEBUG_IMAGE_TOPIC="/depth_anything_v3/output/depth_image_debug"
  echo ""
  echo "Diagnostics (DIAG=1)"
  echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
  echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-unset}"
  echo ""
  echo "ros2 topic info (publishers/subscribers):"
  ros2 topic info "${INPUT_IMAGE_TOPIC}" || true
  ros2 topic info "${INPUT_CAMERA_INFO_TOPIC}" || true
  ros2 topic info "${DEBUG_IMAGE_TOPIC}" || true
  echo ""
  echo "Tip: if publisher count is 0 for ${DEBUG_IMAGE_TOPIC}, CycloneDDS peers likely block local discovery."
  echo "     Ensure your CycloneDDS peers include 127.0.0.1 (this repo's setup script does)."
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
