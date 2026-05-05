#!/usr/bin/env bash
set -eo pipefail

cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Nodes:"
ros2 node list
echo ""

echo "Relevant topics:"
ros2 topic list | grep -E "camera|poses|tracked|interaction|suspicion|alarm" || true
echo ""

echo "Checking /poses/raw for 5 seconds..."
timeout 5 ros2 topic hz /poses/raw || true
echo ""

echo "Checking /poses/tracked for 5 seconds..."
timeout 5 ros2 topic hz /poses/tracked || true
echo ""

echo "Checking /interaction/features for 5 seconds..."
timeout 5 ros2 topic hz /interaction/features || true
echo ""

echo "One interaction feature sample:"
timeout 5 ros2 topic echo /interaction/features --once || true
echo ""

echo "Checking for false alerts for 5 seconds:"
timeout 5 ros2 topic echo /suspicion_event || true
