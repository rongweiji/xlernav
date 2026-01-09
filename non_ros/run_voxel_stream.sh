#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
STREAM_DIR="${SCRIPT_DIR}/voxel_stream"
BUILD_DIR="${STREAM_DIR}/build"
BIN="${BUILD_DIR}/voxel_stream"
ORB_DIR="${SCRIPT_DIR}/orbslam3_stream/vendor/ORB_SLAM3"
VOCAB_FILE="${SCRIPT_DIR}/orbslam3_stream/vocabulary/ORBvoc.txt"
DEPTH_LIB_DIR="${SCRIPT_DIR}/depth_stream_cpp/vendor/depth_anything_v3/lib"

if [[ ! -f "${DEPTH_LIB_DIR}/libtensorrt_depth_anything.so" ]]; then
  echo "Depth engine library not found at ${DEPTH_LIB_DIR}/libtensorrt_depth_anything.so"
  echo "Make sure the Depth Anything vendor files are present under non_ros/depth_stream_cpp/vendor."
  exit 1
fi

if [[ ! -d "${ORB_DIR}" ]]; then
  mkdir -p "$(dirname -- "${ORB_DIR}")"
  git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git "${ORB_DIR}"
  (cd "${ORB_DIR}" && git submodule update --init --recursive)
fi

ORB_CMAKE="${ORB_DIR}/CMakeLists.txt"
if [[ -f "${ORB_CMAKE}" ]]; then
  if ! grep -q "std=c\\+\\+14" "${ORB_CMAKE}"; then
    sed -i 's/std=c++11/std=c++14/g' "${ORB_CMAKE}"
    echo "Patched ORB-SLAM3 to use C++14 (Pangolin/sigslot requirement)."
  fi
fi

ORB_CACHE="${ORB_DIR}/build/CMakeCache.txt"
if [[ -f "${ORB_CACHE}" ]] && grep -q "std=c\\+\\+11" "${ORB_CACHE}"; then
  rm -rf "${ORB_DIR}/build"
fi

if [[ ! -f "${VOCAB_FILE}" ]]; then
  mkdir -p "$(dirname -- "${VOCAB_FILE}")"
  if [[ -f "${ORB_DIR}/Vocabulary/ORBvoc.txt" ]]; then
    cp "${ORB_DIR}/Vocabulary/ORBvoc.txt" "${VOCAB_FILE}"
  elif [[ -f "${ORB_DIR}/Vocabulary/ORBvoc.txt.tar.gz" ]]; then
    tar -xzf "${ORB_DIR}/Vocabulary/ORBvoc.txt.tar.gz" -C "$(dirname -- "${VOCAB_FILE}")"
  else
    echo "ORB vocabulary not found. Expected ${ORB_DIR}/Vocabulary/ORBvoc.txt"
    exit 1
  fi
fi

if [[ ! -f "${ORB_DIR}/lib/libORB_SLAM3.so" ]]; then
  (cd "${ORB_DIR}" && ./build.sh)
fi

if [[ ! -f "${BUILD_DIR}/CMakeCache.txt" ]] || [[ "${STREAM_DIR}/CMakeLists.txt" -nt "${BUILD_DIR}/CMakeCache.txt" ]]; then
  mkdir -p "${BUILD_DIR}"
  cmake -S "${STREAM_DIR}" -B "${BUILD_DIR}" \
    -DCMAKE_PREFIX_PATH="/opt/ros/jazzy"
fi
cmake --build "${BUILD_DIR}" -j"$(nproc)"

export LD_LIBRARY_PATH="${ORB_DIR}/lib:${ORB_DIR}/Thirdparty/DBoW2/lib:${ORB_DIR}/Thirdparty/g2o/lib:${DEPTH_LIB_DIR}:/opt/ros/jazzy/lib:${LD_LIBRARY_PATH:-}"

exec "${BIN}" "$@"
