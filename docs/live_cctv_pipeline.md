# Live CCTV Pipeline

This project has two demos:

1. `scenario_demo.launch.py` — backup demo with simulated NORMAL / WARNING / HIGH inputs.
2. `live_cctv_demo.launch.py` — real pipeline using a CCTV/IP-camera stream.

## Live pipeline

```text
CCTV/IP camera stream
  -> cctv_stream_node              publishes /camera/image_raw
  -> pose_estimator_node           publishes /poses/raw
  -> tracker_node                  publishes /poses/tracked
  -> interaction_analyzer_node     publishes /interaction/features
  -> decision_node                 publishes /suspicion_event
  -> alert_console_node + alarm_node
```

## Important privacy note

Do not commit real CCTV URLs, usernames, or passwords to GitHub. Pass them at runtime using the `stream_url` launch argument.

## Install dependencies inside Docker

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
apt update
apt install -y python3-pip python3-colcon-common-extensions ros-humble-vision-msgs ros-humble-cv-bridge python3-opencv
pip3 install -r src/ros2-child-safety-monitoring/requirements.txt
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

## Run live CCTV demo

```bash
ros2 launch child_safety_monitoring live_cctv_demo.launch.py \
  stream_url:='rtsp://USERNAME:PASSWORD@CAMERA_IP:554/STREAM_PATH'
```

For a Linux USB webcam test inside Docker, start Docker with `--device=/dev/video0:/dev/video0`, then run:

```bash
ros2 launch child_safety_monitoring live_cctv_demo.launch.py stream_url:='0'
```

## Watch useful topics

```bash
ros2 topic list
ros2 topic echo /alarm/state
ros2 topic echo /suspicion_event
ros2 topic hz /camera/image_raw
ros2 topic hz /poses/raw
```

If you have GUI display support, you can inspect the annotated image topic:

```bash
rqt_image_view /camera/pose_overlay
```

## What this prototype can and cannot do

It can detect suspicious movement patterns from pose features. It cannot prove kidnapping, identify people, or infer criminal intent. A real deployment needs human review, privacy approval, camera calibration, and extensive testing on hard negative cases.
