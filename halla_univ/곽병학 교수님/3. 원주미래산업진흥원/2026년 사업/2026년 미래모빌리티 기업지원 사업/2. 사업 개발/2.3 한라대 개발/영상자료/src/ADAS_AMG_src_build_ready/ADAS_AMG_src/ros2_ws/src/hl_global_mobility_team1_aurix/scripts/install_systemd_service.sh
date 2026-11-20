#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-can0}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="$(cd "${PKG_DIR}/../.." && pwd)"
SERVICE_PATH="/etc/systemd/system/hl-team1-aurix-can.service"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
cd "${WS_DIR}"
colcon build --symlink-install --packages-select hl_global_mobility_team1_aurix

sudo tee "${SERVICE_PATH}" >/dev/null <<SERVICE
[Unit]
Description=HL Global Mobility Team 1 AURIX CAN Alive
After=network.target

[Service]
Type=simple
Environment=ROS_DISTRO=${ROS_DISTRO}
ExecStartPre=/bin/bash -lc '${SCRIPT_DIR}/setup_can0.sh ${IFACE} 500000 200'
ExecStart=/bin/bash -lc 'source /opt/ros/${ROS_DISTRO}/setup.bash && source ${WS_DIR}/install/setup.bash && exec ros2 launch hl_global_mobility_team1_aurix aurix_can_alive.launch.py interface:=${IFACE}'
Restart=always
RestartSec=0.2

[Install]
WantedBy=multi-user.target
SERVICE

sudo systemctl daemon-reload
sudo systemctl enable hl-team1-aurix-can.service

echo "Installed ${SERVICE_PATH}"
echo "Start now: sudo systemctl start hl-team1-aurix-can.service"
echo "Logs:      journalctl -u hl-team1-aurix-can.service -f"
