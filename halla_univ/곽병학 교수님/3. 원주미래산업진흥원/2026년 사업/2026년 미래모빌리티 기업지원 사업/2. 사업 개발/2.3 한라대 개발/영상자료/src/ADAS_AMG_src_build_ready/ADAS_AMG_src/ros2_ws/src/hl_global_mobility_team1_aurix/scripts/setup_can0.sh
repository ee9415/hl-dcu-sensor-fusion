#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-can0}"
BITRATE="${2:-500000}"
RESTART_MS="${3:-200}"

if ! command -v ip >/dev/null 2>&1; then
    echo "ip command not found. Install iproute2 first."
    exit 1
fi

sudo modprobe can || true
sudo modprobe can_raw || true
sudo modprobe peak_usb || true

sudo ip link set "${IFACE}" down 2>/dev/null || true
sudo ip link set "${IFACE}" type can bitrate "${BITRATE}" restart-ms "${RESTART_MS}"
sudo ip link set "${IFACE}" up

echo "SocketCAN ${IFACE} is ready."
ip -details link show "${IFACE}"
