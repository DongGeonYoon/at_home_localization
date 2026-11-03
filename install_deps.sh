#!/usr/bin/env bash
# Dependencies of at_home_localization on Ubuntu 22.04 + ROS 2 Humble.
# Run once from the repository root: ./install_deps.sh
# Then build: see README.md (2. 설치와 빌드).
set -euo pipefail

if [[ "$(. /etc/os-release && echo "$VERSION_ID")" != "22.04" ]]; then
    echo "WARNING: tested on Ubuntu 22.04 only." >&2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ROS 2 Humble not found (/opt/ros/humble). Install it first:" >&2
    echo "  https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html" >&2
    exit 1
fi
# conda's python and site settings break the system ROS python packages.
if [[ -n "${CONDA_PREFIX:-}" ]]; then
    echo "Deactivate conda first (conda deactivate), then run again." >&2
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

echo "== python packages (user site, system python3)"
# numpy is pinned last: open3d accepts numpy 2, but ros2_numpy / the ROS
# Humble python stack do not, and global_localization.py exits on import.
/usr/bin/python3 -m pip install --user "open3d==0.19.0" "ros2_numpy==0.0.5"
/usr/bin/python3 -m pip install --user "numpy==1.24.2"

/usr/bin/python3 -c "import numpy, open3d, ros2_numpy; print('numpy', numpy.__version__, '| open3d', open3d.__version__)"
echo "Done. Next: colcon build (README.md, 2. 설치와 빌드)."
