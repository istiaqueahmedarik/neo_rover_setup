#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$(readlink -f -- "$0")")"
set +u
source "/opt/ros/${ROS_DISTRO:-lyrical}/setup.bash"
set -u
exec .venv/bin/python scripts/teleop.py
