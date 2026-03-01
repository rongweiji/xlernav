#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BACKEND="orbslam3"
BUILD_TYPE="Release"

usage() {
  cat <<USAGE
Usage: $0 [--backend orbslam3|cuvslam] [--build-type Release|Debug] [backend-args...]

Backends:
  orbslam3   Live UDP stream pipeline (run_orbslam3_stream.sh)
  cuvslam    Dataset pipeline from third_party/cuVSLAM/cuvslam_cli

Examples:
  $0 --backend orbslam3 -- --port 5600 --rerun-save outputs/orbslam3.rrd
  $0 --backend cuvslam -- --dataset_root data/tum/rgbd_dataset_freiburg1_xyz --dataset_format tum --enable_rerun
USAGE
}

ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --backend)
      BACKEND="$2"
      shift 2
      ;;
    --build-type)
      BUILD_TYPE="$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --)
      shift
      ARGS+=("$@")
      break
      ;;
    *)
      ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ "${BACKEND}" == "orbslam3" ]]; then
  exec "${SCRIPT_DIR}/run_orbslam3_stream.sh" "${ARGS[@]}"
fi

if [[ "${BACKEND}" != "cuvslam" ]]; then
  echo "Unknown backend: ${BACKEND}" >&2
  usage >&2
  exit 1
fi

CUV_DIR="${SCRIPT_DIR}/third_party/cuVSLAM"
if [[ ! -d "${CUV_DIR}" ]]; then
  echo "Missing cuVSLAM submodule at ${CUV_DIR}" >&2
  echo "Run: git submodule update --init --recursive" >&2
  exit 1
fi

BUILD_DIR="${CUV_DIR}/build_rerun"
SDK_DIR="${SCRIPT_DIR}/third_party/cuvslam_sdk"
CMAKE_ARGS=(
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
  -DCUVSLAM_ENABLE_RERUN=ON
)

if [[ -f "${SDK_DIR}/lib/libcuvslam.so" ]]; then
  CMAKE_ARGS+=(
    -DCUVSLAM_SDK_ROOT="${SDK_DIR}"
    -DCUVSLAM_REQUIRE_LIBRARY=ON
  )
  export CUVSLAM_LIB_PATH="${SDK_DIR}/lib/libcuvslam.so"
  export LD_LIBRARY_PATH="${SDK_DIR}/lib:${LD_LIBRARY_PATH:-}"
else
  echo "Warning: local SDK not found at ${SDK_DIR}/lib/libcuvslam.so" >&2
  echo "cuVSLAM runtime may fail unless libcuvslam is available on your system path." >&2
fi

cmake -S "${CUV_DIR}" -B "${BUILD_DIR}" "${CMAKE_ARGS[@]}"
cmake --build "${BUILD_DIR}" -j"$(nproc)"

exec "${BUILD_DIR}/cuvslam_cli" "${ARGS[@]}"
