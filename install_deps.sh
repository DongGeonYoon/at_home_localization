#!/usr/bin/env bash
# Dependencies of at_home_localization on Ubuntu 22.04 + ROS 2 Humble.
# Run once from the repository root, inside the activated virtual
# environment (README.md, 2. 설치와 빌드):
#   source ~/at_home/bin/activate && ./install_deps.sh
# --skip-system installs only the python packages (no apt, no Livox-SDK2).
set -euo pipefail

skip_system=false
[[ "${1:-}" == "--skip-system" ]] && skip_system=true

# Python packages go into the virtual environment, never the system/user
# site. It must see the ROS python packages (rclpy, ...): --system-site-packages.
if [[ -z "${VIRTUAL_ENV:-}" ]]; then
    echo "Activate the virtual environment first:" >&2
    echo "  python3 -m venv --system-site-packages ~/at_home && source ~/at_home/bin/activate" >&2
    exit 1
fi
if ! grep -q "include-system-site-packages = true" "$VIRTUAL_ENV/pyvenv.cfg"; then
    echo "$VIRTUAL_ENV was created without --system-site-packages; ROS python" >&2
    echo "packages are not visible in it. Recreate it with that option." >&2
    exit 1
fi

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

if ! $skip_system; then
echo "== apt packages"
sudo apt-get update
sudo apt-get install -y \
    build-essential cmake git python3-pip python3-venv python3-colcon-common-extensions \
    libpcl-dev libeigen3-dev libapr1-dev python3-opencv python3-yaml \
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
fi

# --system-site-packages also exposes the user site (~/.local), whose numpy /
# open3d would shadow or replace the pinned ones; the venv ignores it.
if ! grep -q "PYTHONNOUSERSITE" "$VIRTUAL_ENV/bin/activate"; then
    printf '\n# at_home_localization: ignore ~/.local packages\nexport PYTHONNOUSERSITE=1\n' \
        >> "$VIRTUAL_ENV/bin/activate"
fi
export PYTHONNOUSERSITE=1

# rclpy (checked below) comes from ROS; its setup script trips set -u.
set +u
source /opt/ros/humble/setup.bash
set -u

echo "== python packages ($VIRTUAL_ENV)"
# numpy is pinned last: open3d accepts numpy 2, but ros2_numpy / the ROS
# Humble python stack do not, and global_localization.py exits on import.
python -m pip install "open3d==0.19.0" "ros2_numpy==0.0.5"
python -m pip install "numpy==1.24.2"

python -c "import numpy, open3d, ros2_numpy, rclpy; print('numpy', numpy.__version__, '| open3d', open3d.__version__, '|', numpy.__file__)"
echo "Done. Next: colcon build (README.md, 2. 설치와 빌드)."
