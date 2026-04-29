# ROS 2 Child Safety Monitoring Prototype

This repository contains a ROS 2 prototype for detecting **suspicious child-lifting interaction patterns** from camera/video input.

The project is intentionally framed as a **safety-monitoring prototype**, not as a system that proves kidnapping or criminal intent. It detects a combination of explainable movement cues:

1. sudden close contact between a larger and smaller person candidate,
2. lifting/upward movement of the smaller person candidate,
3. feet-off-ground or strong vertical displacement cue,
4. rapid limb movement that may indicate struggling,
5. co-movement of the two people while close together.

The output is an alert level, suspicion score, and evidence summary for human review.

## Repository structure

```text
ros2-child-safety-monitoring/
├── child_safety_msgs/          # ROS 2 custom messages and action
├── child_safety_monitoring/    # Python ROS 2 package
├── docs/                       # architecture and project notes
├── scripts/                    # helper scripts
├── data/                       # local videos/rosbags/debug clips, ignored by git
├── requirements.txt
└── README.md
```

## ROS 2 packages

### `child_safety_msgs`

Custom interfaces:

- `PersonPose2D.msg`
- `PersonPose2DArray.msg`
- `InteractionFeatures.msg`
- `SuspicionEvent.msg`
- `StartMonitoring.action`

### `child_safety_monitoring`

Python nodes:

- `video_source_node` — publishes webcam or video file frames to `/camera/image_raw`
- `pose_estimator_node` — runs YOLO pose and publishes `/poses/raw`
- `tracker_node` — assigns simple track IDs and smaller/larger candidate roles
- `interaction_analyzer_node` — calculates contact, lift, struggle, and suspicion features
- `decision_node` — publishes warning/high alert events
- `visualization_node` — draws boxes, keypoints, and alert status on `/annotated_image`

## Recommended environment

Use Ubuntu with ROS 2 Humble or Jazzy. For class/lab work, ROS 2 Humble on Ubuntu 22.04 is usually a stable option.

The first implementation uses a webcam or recorded video. RGB-D support can be added later.

## Install dependencies

```bash
python3 -m pip install -r requirements.txt

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-vision-msgs \
  ros-$ROS_DISTRO-rqt-image-view
```

## Build

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
source install/setup.bash
```

## Run webcam demo

```bash
source install/setup.bash
ros2 launch child_safety_monitoring demo_webcam.launch.py
```

View annotated output:

```bash
rqt_image_view /annotated_image
```

Watch alerts:

```bash
ros2 topic echo /suspicion_event
```

## Run video-file demo

```bash
source install/setup.bash
ros2 launch child_safety_monitoring demo_video.launch.py video_path:=/absolute/path/to/video.mp4
```

## Safety and ethics rules

- Use consenting adults, mannequins, staged safe movements, or simulation.
- Do not collect real videos of minors for the first demo.
- Do not identify faces or people.
- Do not claim the system proves kidnapping or intent.
- Treat all alerts as signals for human review only.
- Avoid storing raw video unless needed for debugging and consent has been obtained.

## Current implementation level

This is the initial working repository scaffold. The core ROS 2 architecture, messages, launch files, and first-pass detection logic are included.

Next steps:

1. run the webcam pipeline,
2. verify YOLO pose output,
3. tune thresholds in `child_safety_monitoring/config/detection_params.yaml`,
4. record short safe acted clips,
5. evaluate false positives and detection delay.
