#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$(readlink -f -- "$0")")"
set +u
source "$PWD/ros_env.sh"
set -u
exec rviz2 -d "$PWD/gazebo/lidar.rviz" --ros-args -p use_sim_time:=true "$@"
