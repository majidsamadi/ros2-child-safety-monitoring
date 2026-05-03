# ROS 2 Child Safety Monitoring Prototype

This repository contains a ROS 2 prototype for detecting suspicious child-safety interaction patterns from a live laptop camera stream.

The project is built around this idea:

```text
Laptop camera
    -> host webcam streamer
    -> Docker ROS 2 environment
    -> YOLO pose detection
    -> person tracking
    -> interaction feature extraction
    -> decision node
    -> alert / alarm output
```

The project also includes a simulator demo, so we can still show `NORMAL`, `WARNING`, and `HIGH ALERT` behavior even when a live camera test is not possible.

## Current project status

Working so far:

```text
✅ Docker + ROS 2 Humble environment works
✅ Laptop camera stream works from the host machine
✅ Docker can read the laptop camera stream
✅ YOLO pose overlay can be viewed in the browser
✅ /poses/raw publishes pose detections
✅ /poses/tracked publishes tracked people
✅ /interaction/features publishes interaction features
✅ Normal standing gives suspicion_score: 0.0
✅ Normal standing does not trigger /suspicion_event
✅ Simulator demo still produces NORMAL -> WARNING -> HIGH ALERT
```

The live laptop-camera pipeline is working for normal behavior and feature extraction. The high-alert behavior is still best demonstrated with the simulator while live detection thresholds are tuned.

## Important safety note

Do not test the system by lifting a real child or doing risky acting.

For live tests, use safe actions only:

```text
- two people standing far apart
- two people standing closer
- one person raising arms near another person without touching or lifting
```

Use the simulator demo to show the warning/high-alert logic.

## Repository structure

```text
child_safety_msgs/
  msg/                      # Custom ROS messages
  action/                   # Custom ROS action

child_safety_monitoring/
  child_safety_monitoring/
    nodes/
      cctv_stream_node.py              # Generic video-stream reader node
      pose_estimator_node.py           # YOLO pose node
      tracker_node.py                  # Person tracking node
      interaction_analyzer_node.py     # Calculates interaction features
      decision_node.py                 # Converts features into warning/high alerts
      alert_console_node.py            # Clean terminal output
      alarm_node.py                    # Alarm state output
      scenario_simulator_node.py       # Backup simulator demo
      video_source_node.py             # Older/local video source helper
      visualization_node.py            # Older visualization helper

  launch/
    live_laptop_camera_demo.launch.py  # Main live laptop-camera demo
    live_cctv_demo.launch.py           # Older/generic stream launch
    scenario_demo.launch.py            # Backup simulator launch

scripts/
  host_webcam_streamer.py              # Runs outside Docker and streams laptop camera

requirements.txt
README.md
```

Note: `cctv_stream_node.py` has a CCTV-style name, but it is currently used as a generic video stream reader. For the laptop demo, it reads:

```text
http://host.docker.internal:8090/video
```

## Main live pipeline

```text
scripts/host_webcam_streamer.py
        ↓
http://host.docker.internal:8090/video
        ↓
cctv_stream_node
        ↓
/camera/image_raw
        ↓
pose_estimator_node
        ↓
/poses/raw
        ↓
tracker_node
        ↓
/poses/tracked
        ↓
interaction_analyzer_node
        ↓
/interaction/features
        ↓
decision_node
        ↓
/suspicion_event
        ↓
alert_console_node + alarm_node
```

## Prerequisites

You need:

```text
Docker Desktop or Docker Engine
Python 3 on the host laptop
A working laptop camera
```

On macOS, run the webcam streamer from an app that has camera permission, such as VS Code if it is enabled in:

```text
System Settings -> Privacy & Security -> Camera
```

## Running the live laptop-camera demo

Use four terminals.

### Terminal 1: start the laptop webcam streamer

Run this outside Docker, preferably in VS Code terminal on macOS:

```bash
cd "/Users/samadi/Master AI/Robotics/ROS2/ros2-child-safety-monitoring"

python3 -m pip install --user "opencv-python==4.10.0.84" "numpy==1.26.4"
python3 scripts/host_webcam_streamer.py --camera 0 --port 8090
```

Keep this terminal open.

Check the raw laptop camera stream in a browser:

```text
http://127.0.0.1:8090/video
```

### Terminal 2: start Docker and run the live ROS pipeline

Run this from the parent folder of the repo:

```bash
cd "/Users/samadi/Master AI/Robotics/ROS2"

docker rm -f ros2-child-safety-dev 2>/dev/null || true

docker run -it --rm \
  --name ros2-child-safety-dev \
  -p 8080:8080 \
  -v "$PWD/ros2-child-safety-monitoring:/root/ros2_ws/src/ros2-child-safety-monitoring" \
  osrf/ros:humble-desktop \
  bash
```

Inside Docker:

```bash
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

pip3 install --no-cache-dir -r src/ros2-child-safety-monitoring/requirements.txt
pip3 install --no-cache-dir --force-reinstall "numpy==1.26.4" "opencv-python==4.10.0.84"

rm -rf build install log
colcon build --symlink-install
source install/setup.bash

ros2 launch child_safety_monitoring live_laptop_camera_demo.launch.py \
  stream_url:='http://host.docker.internal:8090/video'
```

Keep this terminal running.

### Terminal 3: start the browser pose-overlay server

Open a new terminal and enter the same Docker container:

```bash
docker exec -it ros2-child-safety-dev bash
```

Inside Docker:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run web_video_server web_video_server
```

Now open this in a browser:

```text
http://localhost:8080/stream?topic=/camera/pose_overlay
```

This shows the camera with YOLO pose overlay.

Raw camera view:

```text
http://localhost:8080/stream?topic=/camera/image_raw
```

### Terminal 4: check ROS topics

Open another terminal:

```bash
docker exec -it ros2-child-safety-dev bash
```

Inside Docker:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 node list
```

Expected important nodes:

```text
/cctv_stream_node
/pose_estimator_node
/tracker_node
/interaction_analyzer_node
/decision_node
/alert_console_node
/alarm_node
```

Check pose and feature topics:

```bash
ros2 topic hz /poses/raw
ros2 topic hz /poses/tracked
ros2 topic hz /interaction/features
```

Print one interaction feature message:

```bash
ros2 topic echo /interaction/features --once
```

For normal standing, a safe result looks like:

```text
suspicion_score: 0.0
state: observing
```

Check that normal behavior does not trigger alerts:

```bash
timeout 10 ros2 topic echo /suspicion_event
```

Expected for normal behavior: no output.

## Simulator demo

The simulator is useful for class demonstration because it shows the decision and alarm behavior without needing people to perform risky actions.

Inside Docker:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch child_safety_monitoring scenario_demo.launch.py
```

Expected output:

```text
[NORMAL] score=0.10 | No suspicious interaction pattern detected
[WARNING] score=0.62 | Suspicious interaction pattern detected
[HIGH ALERT] score=0.92 | Suspicious child-lifting pattern detected
```

## What the live feature values mean

The interaction analyzer publishes:

```text
torso_distance_norm       # smaller value means people are closer
wrap_score                # arm/body wrap-like posture estimate
lift_score                # upward body motion estimate
feet_off_ground_score     # feet/ankle visibility and off-ground proxy
limb_speed_score          # rapid limb movement estimate
limb_accel_score          # sudden limb acceleration estimate
co_motion_score           # people moving together
suspicion_score           # combined score
state                     # observing, watch, warning, high_alert, etc.
```

During safe normal testing, we expect low scores and no alert.

## Current known limitations

This is a prototype. It is not proof of kidnapping or harm.

Current limitations:

```text
- Laptop camera angle affects keypoint quality.
- YOLO sometimes loses ankles or wrists if the body is partly out of frame.
- Smaller/larger person role is estimated from bounding box size, not age.
- Live high-alert behavior needs more tuning.
- NNPACK warnings may appear on Apple Silicon Docker; they are annoying but not fatal.
```

The safest live demo is:

```text
1. show YOLO pose overlay in browser
2. show /interaction/features publishing
3. show normal behavior does not trigger false alarm
4. use simulator demo for warning/high-alert behavior
```

## Troubleshooting

### Browser camera stream does not open

Check Terminal 1. It should be running:

```bash
python3 scripts/host_webcam_streamer.py --camera 0 --port 8090
```

Then test:

```text
http://127.0.0.1:8090/video
```

On macOS, run from VS Code if Terminal does not have camera permission.

### Docker cannot read the camera stream

Inside Docker, the stream URL should be:

```text
http://host.docker.internal:8090/video
```

### `/interaction_analyzer_node` is missing

Rebuild and relaunch:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash

ros2 launch child_safety_monitoring live_laptop_camera_demo.launch.py \
  stream_url:='http://host.docker.internal:8090/video'
```

### NumPy / cv_bridge error

Use the pinned versions:

```bash
pip3 install --no-cache-dir --force-reinstall "numpy==1.26.4" "opencv-python==4.10.0.84"
```

### NNPACK warning

You may see many lines like:

```text
Could not initialize NNPACK! Reason: Unsupported hardware.
```

This warning is common on the current Mac + Docker setup and does not mean the pipeline failed.

## Windows and Linux notes

The project runs inside a Linux ROS 2 Docker container even if the host laptop is Windows or macOS.

For Windows teammates:

```text
- use Docker Desktop
- run the webcam streamer on Windows host
- use http://host.docker.internal:8090/video inside Docker
- adjust the volume mount path if needed
```

For Linux teammates:

```text
- Docker Engine can run the ROS container directly
- host.docker.internal may need:
  --add-host=host.docker.internal:host-gateway
- Linux can also pass a webcam directly with /dev/video0, but the host-streamer method keeps the workflow similar across machines
```

## Recommended class demo flow

```text
1. Start host webcam streamer.
2. Start live_laptop_camera_demo.launch.py in Docker.
3. Open /camera/pose_overlay in browser.
4. Show two people detected and tracked.
5. Show /interaction/features publishing.
6. Show no false alarm during normal behavior.
7. Run scenario_demo.launch.py to show warning/high-alert behavior.
```

## Git hygiene

Do not commit:

```text
build/
install/
log/
__pycache__/
.DS_Store
real passwords or private camera URLs
```

These are covered by `.gitignore`.
