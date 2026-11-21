#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$(readlink -f -- "$0")")"
ROS_DISTRO="${ROS_DISTRO:-lyrical}"
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
for package in leo_description leo_gz_plugins gz_sim_vendor ros_gz_bridge robot_state_publisher xacro; do
  ros2 pkg prefix "$package" >/dev/null || {
    echo "Missing ROS package: $package (expected ros-${ROS_DISTRO}-${package//_/-})" >&2
    exit 1
  }
done
python3 -c 'import cv2; assert hasattr(cv2, "aruco")' >/dev/null 2>&1 || {
  echo "Missing Python OpenCV ArUco support (install python3-opencv)." >&2
  exit 1
}
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements-gazebo.txt
if [[ ! -f assets/terrain.npy ]]; then
  .venv/bin/pip install 'rasterio>=1.4' 'pyproj>=3.6'
  .venv/bin/python scripts/build_terrain.py
fi
mkdir -p logs
.venv/bin/python gazebo/build_world.py
gz sdf -k gazebo/mdrs.sdf
cmake -S gazebo/drive_keys -B gazebo/drive_keys/build -DCMAKE_BUILD_TYPE=Release
cmake --build gazebo/drive_keys/build -j2
