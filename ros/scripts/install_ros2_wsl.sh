#!/usr/bin/env bash
set -euo pipefail

if ! command -v lsb_release >/dev/null 2>&1; then
  echo "lsb_release not found; please install it first." >&2
  exit 1
fi

UBUNTU_CODENAME="$(lsb_release -cs)"
if [[ "${UBUNTU_CODENAME}" != "noble" ]]; then
  echo "This script expects Ubuntu 24.04 (noble). Detected: ${UBUNTU_CODENAME}" >&2
  exit 1
fi

sudo apt update
sudo apt install -y curl gnupg2 lsb-release software-properties-common locales

sudo add-apt-repository universe -y

# Ensure a UTF-8 locale is available (recommended by ROS docs).
if ! locale -a | grep -qi 'utf-8'; then
  sudo locale-gen en_US.UTF-8
  sudo update-locale LANG=en_US.UTF-8
fi

# Try official ros2-apt-source; if unavailable, fall back to manual key/repo.
sudo apt update
if sudo apt install -y ros2-apt-source; then
  :
else
  echo "ros2-apt-source not available; falling back to manual repo setup."
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
fi

sudo apt update
sudo apt upgrade -y
sudo apt install -y \
  ros-jazzy-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-argcomplete

sudo rosdep init || true
rosdep update

if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

echo "ROS 2 Jazzy (desktop) installed. Open a new shell or run: source /opt/ros/jazzy/setup.bash"
