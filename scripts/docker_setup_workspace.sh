#!/usr/bin/env bash
set -eo pipefail

cd /root/ros2_ws
source /opt/ros/humble/setup.bash

apt update
apt install -y \
  python3-pip \
  python3-colcon-common-extensions \
  ros-humble-vision-msgs \
  ros-humble-cv-bridge \
  ros-humble-web-video-server \
  python3-opencv

# Install CPU PyTorch first to avoid downloading huge CUDA builds when possible.
# If this fails on a machine, comment these two lines and use the fallback pip install below.
pip3 install --no-cache-dir --index-url https://download.pytorch.org/whl/cpu \
  "torch==2.2.2+cpu" "torchvision==0.17.2+cpu" || true

pip3 install --no-cache-dir -r src/ros2-child-safety-monitoring/requirements.txt

# Keep cv_bridge compatible with ROS Humble.
pip3 install --no-cache-dir --force-reinstall \
  "numpy==1.26.4" "opencv-python==4.10.0.84"

rm -rf build install log
colcon build --symlink-install
source install/setup.bash

echo ""
echo "Workspace setup complete."
echo "Run:"
echo "  ros2 launch child_safety_monitoring live_laptop_camera_demo.launch.py stream_url:='http://host.docker.internal:8090/video'"
