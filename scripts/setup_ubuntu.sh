#!/usr/bin/env bash
# Run inside Ubuntu 22.04 guest after auto_aim is copied to ~/rm_auto_aim/src/auto_aim
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"

sudo apt-get update
sudo apt-get install -y \
  "ros-${ROS_DISTRO}-desktop" \
  "ros-${ROS_DISTRO}-cv-bridge" \
  python3-colcon-common-extensions \
  libopencv-dev \
  build-essential \
  cmake

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

WS="${HOME}/rm_auto_aim"
mkdir -p "${WS}/src"
if [[ ! -d "${WS}/src/auto_aim" ]]; then
  echo "Place auto_aim package at ${WS}/src/auto_aim first" >&2
  exit 1
fi

cd "${WS}"
colcon build --packages-select auto_aim --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "Build OK. Next:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source ${WS}/install/setup.bash"
echo "  ros2 run auto_aim auto_aim_node --ros-args --params-file ${WS}/src/auto_aim/config/params.yaml"
