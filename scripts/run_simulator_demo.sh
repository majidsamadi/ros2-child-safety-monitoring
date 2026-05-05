#!/usr/bin/env bash
set -eo pipefail

cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch child_safety_monitoring scenario_demo.launch.py
