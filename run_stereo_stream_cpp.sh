#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="${SCRIPT_DIR}/stereo_stream_cpp"
BUILD_DIR="${SRC_DIR}/build"
BIN="${BUILD_DIR}/stereo_stream_cpp"

if [[ ! -f "${BUILD_DIR}/CMakeCache.txt" ]]; then
  mkdir -p "${BUILD_DIR}"
  cmake -S "${SRC_DIR}" -B "${BUILD_DIR}" \
    -DCMAKE_PREFIX_PATH="/opt/ros/jazzy"
fi
cmake --build "${BUILD_DIR}" -j"$(nproc)"

exec "${BIN}" "$@"
