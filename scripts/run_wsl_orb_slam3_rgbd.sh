#!/usr/bin/env bash
set -euo pipefail

# Runs ORB-SLAM3 RGB-D node on WSL, subscribing to Pi camera + Depth Anything topics.
#
# Usage:
#   bash scripts/run_wsl_orb_slam3_rgbd.sh <PI_IP>
#
# Optional env vars:
#   ROS_DOMAIN_ID           (default: 0)
#   RGB_TOPIC               (default: /image_raw)
#   DEPTH_TOPIC             (default: /depth_anything_v3/output/depth_image)
#   CAMERA_INFO_TOPIC       (default: /camera_info)
#   VOCAB_PATH              (default: ORB_SLAM3/Vocabulary/ORBvoc.txt)
#   SETTINGS_PATH           (default: config/raspi_rgbd.yaml)
#   ENABLE_VIEWER           (default: false)
#   IMAGE_ENCODING          (default: "")
#   DIAG_PERIOD             (default: 2.0)
#   SYNC_QUEUE_SIZE         (default: 10)
#   SYNC_SLOP_SEC           (default: 0.05)

PI_IP="${1:-${PI_IP:-192.168.50.124}}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
RGB_TOPIC="${RGB_TOPIC:-/image_raw}"
DEPTH_TOPIC="${DEPTH_TOPIC:-/depth_anything_v3/output/depth_image}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera_info}"
ENABLE_VIEWER="${ENABLE_VIEWER:-false}"
IMAGE_ENCODING="${IMAGE_ENCODING:-}"
DIAG_PERIOD="${DIAG_PERIOD:-2.0}"
SYNC_QUEUE_SIZE="${SYNC_QUEUE_SIZE:-10}"
SYNC_SLOP_SEC="${SYNC_SLOP_SEC:-0.05}"
USE_CYCLONE="${USE_CYCLONE:-0}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
WS_SETUP="${REPO_ROOT}/ros2_ws/install/setup.bash"

ORB_SLAM3_ROOT="${ORB_SLAM3_ROOT:-/mnt/g/LocalGitProject/ORB_SLAM3}"
VOCAB_PATH="${VOCAB_PATH:-${ORB_SLAM3_ROOT}/Vocabulary/ORBvoc.txt}"
SETTINGS_PATH="${SETTINGS_PATH:-${REPO_ROOT}/ros2_ws/orb-slam3-ros2/config/raspi_rgbd.yaml}"

if [[ ! -f "/opt/ros/jazzy/setup.bash" ]]; then
  echo "Missing /opt/ros/jazzy/setup.bash. Install ROS 2 Jazzy first." >&2
  exit 1
fi
if [[ ! -f "${WS_SETUP}" ]]; then
  echo "Missing ${WS_SETUP}. Build the workspace first (colcon build)." >&2
  exit 1
fi
if [[ ! -f "${VOCAB_PATH}" ]]; then
  echo "Missing ORB vocabulary: ${VOCAB_PATH}" >&2
  exit 1
fi
if [[ ! -f "${SETTINGS_PATH}" ]]; then
  echo "Missing settings YAML: ${SETTINGS_PATH}" >&2
  exit 1
fi

if [[ "${USE_CYCLONE}" == "1" ]]; then
  CONFIG_DIR="${HOME}/.config/cyclonedds"
  CONFIG_FILE="${CONFIG_DIR}/cyclonedds.xml"
  mkdir -p "${CONFIG_DIR}"
  cat > "${CONFIG_FILE}" <<EOF_CFG
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
EOF_CFG
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="file://${CONFIG_FILE}"
else
  unset RMW_IMPLEMENTATION
  unset CYCLONEDDS_URI
fi

export ROS_DOMAIN_ID
export ROS_LOCALHOST_ONLY=0

# ORB-SLAM3 runtime libs
export LD_LIBRARY_PATH="${ORB_SLAM3_ROOT}/lib:${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib:${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}"

set +u
source /opt/ros/jazzy/setup.bash
source "${WS_SETUP}"
set -u

echo "Pi peer: ${PI_IP}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-FastDDS (default)}"
echo "RGB topic: ${RGB_TOPIC}"
echo "Depth topic: ${DEPTH_TOPIC}"
echo "CameraInfo topic: ${CAMERA_INFO_TOPIC}"
echo "Vocab: ${VOCAB_PATH}"
echo "Settings: ${SETTINGS_PATH}"
echo ""

ROS_ARGS=(
  -p vocab_path:="${VOCAB_PATH}"
  -p settings_path:="${SETTINGS_PATH}"
  -p rgb_topic:="${RGB_TOPIC}"
  -p depth_topic:="${DEPTH_TOPIC}"
  -p camera_info_topic:="${CAMERA_INFO_TOPIC}"
  -p enable_viewer:="${ENABLE_VIEWER}"
  -p diagnostics_period_sec:="${DIAG_PERIOD}"
  -p sync_queue_size:="${SYNC_QUEUE_SIZE}"
  -p sync_slop_sec:="${SYNC_SLOP_SEC}"
)

if [[ -n "${IMAGE_ENCODING}" ]]; then
  ROS_ARGS+=(-p image_encoding:="${IMAGE_ENCODING}")
fi

ros2 run orb_slam3_ros2 orb_slam3_rgbd_node --ros-args "${ROS_ARGS[@]}"
