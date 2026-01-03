#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"

ARCH="$(uname -m)"
if [[ "${ARCH}" != "aarch64" ]]; then
  echo "This script expects 64-bit ARM (aarch64). Detected: ${ARCH}" >&2
  exit 1
fi

if ! command -v lsb_release >/dev/null 2>&1; then
  echo "lsb_release not found; please install it first." >&2
  exit 1
fi

OS_ID="$(lsb_release -is | tr '[:upper:]' '[:lower:]')"
if [[ "${OS_ID}" != "debian" ]]; then
  echo "This script targets Raspberry Pi OS (Debian-based). Detected: ${OS_ID}" >&2
  exit 1
fi

sudo apt update
sudo apt install -y curl ca-certificates gnupg lsb-release

# Install Docker using the official convenience script.
curl -fsSL https://get.docker.com -o /tmp/get-docker.sh
sudo sh /tmp/get-docker.sh
rm -f /tmp/get-docker.sh

sudo usermod -aG docker "$USER"
sudo systemctl enable --now docker

sudo docker pull "ros:${ROS_DISTRO}-ros-base"

cat <<EOF
ROS 2 will run in Docker on the Pi.

Example (host networking):
  docker run --rm -it --net=host --ipc=host \\
    -e ROS_DOMAIN_ID=0 \\
    ros:${ROS_DISTRO}-ros-base

Log out and back in to apply docker group permissions.
EOF
