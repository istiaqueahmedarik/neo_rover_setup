#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$(readlink -f -- "$0")")"
exec 9>"$PWD/.simulation.lock"
flock -n 9 || { echo 'This rover simulation is already running.' >&2; exit 1; }
set +u
source "/opt/ros/${ROS_DISTRO:-lyrical}/setup.bash"
set -u
export GZ_PARTITION="${GZ_PARTITION:-my_rover_$UID}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
if command -v nvidia-smi >/dev/null 2>&1; then
  export __NV_PRIME_RENDER_OFFLOAD="${__NV_PRIME_RENDER_OFFLOAD:-1}"
  export __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-nvidia}"
fi
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export ROS_LOG_DIR="$PWD/logs/ros"
mkdir -p "$ROS_LOG_DIR"
if [[ ! -f gazebo/mdrs.sdf ]]; then
  if [[ ! -x .venv/bin/python ]]; then
    echo 'Missing .venv — run ./setup.sh first.' >&2
    exit 1
  fi
  .venv/bin/python gazebo/build_world.py
fi
if [[ ! -f gazebo/drive_keys/build/libDriveKeys.so ]]; then
  cmake -S gazebo/drive_keys -B gazebo/drive_keys/build -DCMAKE_BUILD_TYPE=Release
  cmake --build gazebo/drive_keys/build -j2
fi
export GZ_GUI_PLUGIN_PATH="$PWD/gazebo/drive_keys/build${GZ_GUI_PLUGIN_PATH:+:$GZ_GUI_PLUGIN_PATH}"
exec /usr/bin/python3 "$PWD/gazebo/supervisor.py" "$@"
