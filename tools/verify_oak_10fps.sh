#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/artifacts/ros_offload/install/setup.bash"
exec python3 "$ROOT/tools/verify_oak_10fps.py" "$@"
