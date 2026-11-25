#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/artifacts/camera_can/install/setup.bash"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$ROOT/tools/ros_fastdds_large_images.xml}"
# One bridge/sequence owner per control computer through this launcher.
exec flock -n "$ROOT/artifacts/camera_can/launch.lock" \
    ros2 launch hl_camera_can camera_can.launch.py "$@"
