#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-can0}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="$(cd "${PKG_DIR}/../.." && pwd)"

source "/opt/ros/${ROS_DISTRO}/setup.bash"

"${SCRIPT_DIR}/setup_can0.sh" "${IFACE}" 500000 200

cd "${WS_DIR}"
colcon build --symlink-install --packages-select hl_global_mobility_team1_aurix
source "${WS_DIR}/install/setup.bash"

exec ros2 launch hl_global_mobility_team1_aurix aurix_can_alive.launch.py interface:="${IFACE}"
