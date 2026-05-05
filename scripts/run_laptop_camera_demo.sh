#!/usr/bin/env bash
set -euo pipefail

cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

STREAM_URL="${STREAM_URL:-http://host.docker.internal:8090/video}"

ros2 launch child_safety_monitoring live_laptop_camera_demo.launch.py \
  stream_url:="${STREAM_URL}"
