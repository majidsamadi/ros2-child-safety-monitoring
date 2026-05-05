#!/usr/bin/env bash
set -eo pipefail

cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Open this in your browser:"
echo "  http://localhost:8080/stream?topic=/camera/pose_overlay"
echo ""
echo "Raw camera:"
echo "  http://localhost:8080/stream?topic=/camera/image_raw"
echo ""

ros2 run web_video_server web_video_server
