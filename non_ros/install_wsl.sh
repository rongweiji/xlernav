#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  gir1.2-gstreamer-1.0 \
  libboost-all-dev \
  libeigen3-dev \
  libgl1-mesa-dev \
  libglew-dev \
  libglfw3-dev \
  libopencv-dev \
  libjpeg-dev \
  libpng-dev \
  libtiff-dev \
  libqt6opengl6-dev \
  qt6-base-dev \
  qt6-base-dev-tools \
  python3-gi \
  python3-opencv \
  python3-numpy

if ! dpkg -s libpangolin-dev >/dev/null 2>&1; then
  if ! sudo apt install -y libpangolin-dev; then
    echo "libpangolin-dev not available via apt. ORB-SLAM3 viewer needs Pangolin; build from source if required."
  fi
fi
