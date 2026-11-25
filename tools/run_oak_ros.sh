#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/artifacts/ros_offload/install/setup.bash"
# Keep multi-megabyte frame bundles and result images in shared memory.
# Preserve an explicitly selected deployment profile.
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$ROOT/tools/ros_fastdds_large_images.xml}"
exec ros2 launch hl_oak_perception oak_host.launch.py "device_ip:=${1:-169.254.100.228}"
