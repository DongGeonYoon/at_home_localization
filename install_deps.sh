#!/usr/bin/env bash
# System dependencies of at_home_localization on Ubuntu 22.04 + ROS 2 Humble
# (apt packages and Livox-SDK2). Run once from the repository root with conda
# deactivated: ./install_deps.sh
# The python packages of the nodes come from environment.yml (README.md,
# 2. 설치와 빌드).
set -euo pipefail

if [[ "$(. /etc/os-release && echo "$VERSION_ID")" != "22.04" ]]; then
    echo "WARNING: tested on Ubuntu 22.04 only." >&2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ROS 2 Humble not found (/opt/ros/humble). Install it first:" >&2
    echo "  https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html" >&2
    exit 1
fi
# conda's cmake / compilers / python must not be picked up by the builds.
if [[ -n "${CONDA_PREFIX:-}" ]]; then
    echo "Deactivate conda first (conda deactivate, also base), then run again." >&2
    exit 1
fi

echo "== apt packages"
sudo apt-get update
sudo apt-get install -y \
    build-essential cmake git python3-pip python3-colcon-common-extensions \
    libpcl-dev libeigen3-dev libapr1-dev \
    ros-humble-navigation2 \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-tf-transformations \
    ros-humble-rmw-zenoh-cpp \
    ros-humble-rviz2

echo "== Livox-SDK2 (/usr/local)"
if [[ -f /usr/local/lib/liblivox_lidar_sdk_shared.so ]]; then
    echo "already installed"
else
    src_dir="$(mktemp -d)"
    git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git "$src_dir/Livox-SDK2"
    cmake -S "$src_dir/Livox-SDK2" -B "$src_dir/Livox-SDK2/build"
    cmake --build "$src_dir/Livox-SDK2/build" -j"$(nproc)"
    sudo cmake --install "$src_dir/Livox-SDK2/build"
    sudo ldconfig
    rm -rf "$src_dir"
fi

echo "Done. Next: the conda environment and colcon build (README.md, 2. 설치와 빌드)."
