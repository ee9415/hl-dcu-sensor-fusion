#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
ros2 pkg prefix vision_msgs
command -v colcon
python3 -c 'import cv2,numpy; print("OpenCV", cv2.__version__)'
