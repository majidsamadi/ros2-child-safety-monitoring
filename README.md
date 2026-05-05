# ROS 2 Child Safety Monitoring Prototype

This project is a ROS 2 prototype for detecting suspicious child-safety interaction patterns using a **live laptop camera**.

The current project pipeline is:

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

The project also includes a simulator demo for showing `NORMAL`, `WARNING`, and `HIGH ALERT` behavior without unsafe acting.

## Current status

Working:

```text
✅ Docker + ROS 2 Humble environment
✅ Laptop camera stream from the host machine
✅ Docker reads the laptop camera stream
✅ YOLO pose overlay in browser
✅ /poses/raw publishes pose detections
✅ /poses/tracked publishes tracked people
✅ /interaction/features publishes interaction features
✅ Normal standing gives suspicion_score: 0.0
✅ Normal standing does not trigger /suspicion_event
✅ Simulator demo produces NORMAL -> WARNING -> HIGH ALERT
```

## Safety note

Do **not** test by lifting a real child or doing risky acting.

Safe live tests:

```text
- two people standing far apart
- two people standing closer
- one person raising arms near another person without touching or lifting
```

Use the simulator demo to show warning and high-alert behavior.

## Repository structure

```text
child_safety_msgs/
  msg/                                  # Custom ROS messages

child_safety_monitoring/
  child_safety_monitoring/nodes/
    cctv_stream_node.py                 # Generic stream reader node
    pose_estimator_node.py              # YOLO pose node
    tracker_node.py                     # Person tracking node
    interaction_analyzer_node.py        # Calculates interaction features
    decision_node.py                    # Converts features to warning/high events
    alert_console_node.py               # Clean terminal alert output
    alarm_node.py                       # Alarm state output
    scenario_simulator_node.py          # Backup simulator demo

  launch/
    live_laptop_camera_demo.launch.py   # Main live laptop-camera demo
    scenario_demo.launch.py             # Backup simulator demo

scripts/
  host_webcam_streamer.py               # Runs outside Docker
  run_webcam_streamer.sh                # Host helper script
  run_docker_dev.sh                     # Starts Docker container
  docker_setup_workspace.sh             # Installs deps and builds inside Docker
  run_laptop_camera_demo.sh             # Runs live laptop camera launch
  run_pose_overlay_server.sh            # Starts browser overlay server
  check_live_pipeline.sh                # Quick ROS topic checks
  run_simulator_demo.sh                 # Runs simulator demo
```

Note: `cctv_stream_node.py` has an old CCTV-style name, but it is currently used as a **generic video stream reader**. For laptop camera demo it reads:

```text
http://host.docker.internal:8090/video
```

## Quick run: live laptop-camera demo

Use 4 terminals.

### Terminal 1: host laptop webcam stream

Run this **outside Docker**.

On macOS, use a terminal app that has camera permission, such as VS Code terminal.

```bash
cd path/to/ros2-child-safety-monitoring
./scripts/run_webcam_streamer.sh
```

Open this in browser:

```text
http://127.0.0.1:8090/video
```

Keep Terminal 1 running.

### Terminal 2: Docker + ROS pipeline

Run this from the repo root, outside Docker:

```bash
cd path/to/ros2-child-safety-monitoring
./scripts/run_docker_dev.sh
```

Inside Docker:

```bash
cd /root/ros2_ws
bash src/ros2-child-safety-monitoring/scripts/docker_setup_workspace.sh
bash src/ros2-child-safety-monitoring/scripts/run_laptop_camera_demo.sh
```

Keep Terminal 2 running.

### Terminal 3: browser pose overlay

Open a new terminal:

```bash
docker exec -it ros2-child-safety-dev bash
```

Inside Docker:

```bash
bash /root/ros2_ws/src/ros2-child-safety-monitoring/scripts/run_pose_overlay_server.sh
```

Open this in browser:

```text
http://localhost:8080/stream?topic=/camera/pose_overlay
```

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
bash /root/ros2_ws/src/ros2-child-safety-monitoring/scripts/check_live_pipeline.sh
```

Important topics:

```text
/poses/raw
/poses/tracked
/interaction/features
/suspicion_event
/alarm/state
```

For normal safe behavior, expected result:

```text
suspicion_score: 0.0
state: observing
no /suspicion_event output
```

## Manual topic checks

Inside Docker:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 node list
ros2 topic hz /poses/raw
ros2 topic hz /poses/tracked
ros2 topic hz /interaction/features
ros2 topic echo /interaction/features --once
timeout 10 ros2 topic echo /suspicion_event
```

## Simulator demo

Use this when you want to demonstrate warning/high-alert behavior safely.

Inside Docker:

```bash
bash /root/ros2_ws/src/ros2-child-safety-monitoring/scripts/run_simulator_demo.sh
```

Expected output:

```text
[NORMAL] score=0.10 | No suspicious interaction pattern detected
[WARNING] score=0.62 | Suspicious interaction pattern detected
[HIGH ALERT] score=0.92 | Suspicious child-lifting pattern detected
```

## What interaction features mean

```text
torso_distance_norm       # smaller value means people are closer
wrap_score                # arm/body wrap-like posture estimate
lift_score                # upward body motion estimate
feet_off_ground_score     # feet/ankle off-ground proxy
limb_speed_score          # rapid limb movement estimate
limb_accel_score          # sudden limb acceleration estimate
co_motion_score           # people moving together
suspicion_score           # combined score
state                     # observing, watch, warning, high_alert
```

## Current limitations

This is a prototype. It is not proof of kidnapping or harm.

Known limitations:

```text
- Laptop camera angle affects keypoint quality.
- YOLO may lose wrists/ankles if people are partly out of frame.
- Smaller/larger role is estimated from bounding box size, not real age.
- Live warning/high-alert thresholds still need tuning.
- NNPACK warnings may appear on Apple Silicon Docker. They are annoying but not fatal.
```

Recommended class demo flow:

```text
1. Show laptop camera stream.
2. Show YOLO pose overlay in browser.
3. Show two people tracked.
4. Show /interaction/features publishing.
5. Show normal behavior does not trigger false alert.
6. Use simulator demo for warning/high-alert.
```

## Troubleshooting

### Camera stream does not open

Run from a terminal app with camera permission.

On macOS:

```text
System Settings -> Privacy & Security -> Camera
```

Then test:

```text
http://127.0.0.1:8090/video
```

### Docker cannot read host camera stream

Inside Docker, the stream URL should be:

```text
http://host.docker.internal:8090/video
```

### Browser pose overlay does not open

Make sure Docker was started with:

```bash
-p 8080:8080
```

Then run:

```bash
bash /root/ros2_ws/src/ros2-child-safety-monitoring/scripts/run_pose_overlay_server.sh
```

Open:

```text
http://localhost:8080/stream?topic=/camera/pose_overlay
```

### `/interaction_analyzer_node` is missing

Rebuild:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

Then relaunch the live demo.

### NumPy / cv_bridge error

Use pinned versions:

```bash
pip3 install --no-cache-dir --force-reinstall "numpy==1.26.4" "opencv-python==4.10.0.84"
```

### NNPACK warnings

You may see repeated warnings like:

```text
Could not initialize NNPACK! Reason: Unsupported hardware.
```

These are common on the current Mac + Docker setup and do not mean the pipeline failed.

## Windows and Linux notes

The project runs inside a Linux ROS 2 Docker container even if the host laptop is Windows or macOS.

For Windows teammates:

```text
- use Docker Desktop
- run the webcam streamer on the Windows host
- inside Docker, use http://host.docker.internal:8090/video
- use PowerShell/Git Bash/WSL depending on preference
```

For Linux teammates:

```text
- Docker Engine can run the ROS container directly
- if host.docker.internal does not work, start Docker with:
  --add-host=host.docker.internal:host-gateway
- direct webcam passthrough with /dev/video0 is possible, but the host-streamer method keeps the workflow similar
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
