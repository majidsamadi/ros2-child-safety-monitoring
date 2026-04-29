# README Demo Section

Copy this section into your main `README.md` file.

---

## Demo: ROS 2 Suspicious Child-Lifting Detection Prototype

This demo runs the ROS 2 child-safety monitoring pipeline using a simulated scenario input. It does **not** require a camera yet. The simulator publishes three situations:

```text
NORMAL  → no suspicious pattern
WARNING → suspicious interaction pattern
HIGH    → suspicious child-lifting pattern
```

The purpose of this demo is to show that the ROS 2 communication pipeline works:

```text
scenario_simulator_node
        ↓ /interaction/features
 decision_node
        ↓ /suspicion_event
 alert_console_node
        ↓
 clean terminal alert output
```

> Important: This project is a prototype for detecting suspicious movement patterns. It does not identify people, prove kidnapping, or infer criminal intent. Any real-world safety system would require human review, privacy controls, and much more testing.

---

## Requirements

This project is tested using Docker with ROS 2 Humble.

You need:

- Docker Desktop installed and running
- Git
- Internet connection for the first Docker build/run
- The repository cloned locally

Repository:

```bash
git clone https://github.com/majidsamadi/ros2-child-safety-monitoring.git
cd ros2-child-safety-monitoring
```

---

## Running the Demo on macOS with Docker

From the parent folder that contains the repository, run:

```bash
cd "/Users/samadi/Master AI/Robotics/ROS2"

docker run -it --rm \
  --name ros2-child-safety-dev \
  -v "$PWD/ros2-child-safety-monitoring:/root/ros2_ws/src/ros2-child-safety-monitoring" \
  osrf/ros:humble-desktop \
  bash
```

Inside the Docker container, run:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
apt update
apt install -y python3-pip python3-colcon-common-extensions ros-humble-vision-msgs
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
ros2 launch child_safety_monitoring scenario_demo.launch.py
```

---

## Expected Demo Output

When the demo runs successfully, you should see three ROS 2 nodes start:

```text
decision_node
scenario_simulator_node
alert_console_node
```

Then the simulator cycles through `NORMAL`, `WARNING`, and `HIGH` scenarios.

Expected output example:

```text
Changed scenario → NORMAL | score=0.10, state=normal
[NORMAL] score=0.10 | No suspicious interaction pattern detected

Changed scenario → WARNING | score=0.62, state=warning
[WARNING] score=0.62 | Suspicious interaction pattern detected

Changed scenario → HIGH | score=0.92, state=high_alert
[HIGH ALERT] score=0.92 | Suspicious child-lifting pattern detected

Changed scenario → NORMAL | score=0.10, state=normal
[NORMAL] score=0.10 | No suspicious interaction pattern detected
```

This proves that the following flow is working:

```text
Simulated interaction features → decision logic → readable safety alert
```

---

## Stopping the Demo

Press:

```text
CTRL + C
```

A clean shutdown should show messages like:

```text
process has finished cleanly
```

Then exit Docker:

```bash
exit
```

---

## What Each Node Does

### `scenario_simulator_node`

Publishes fake `InteractionFeatures` messages. This allows the team to test the project without a camera.

Scenarios:

- `normal`: low score, no alert
- `warning`: medium score, warning alert
- `high`: high score, high alert

### `decision_node`

Subscribes to:

```text
/interaction/features
```

Publishes:

```text
/suspicion_event
```

It applies threshold and persistence logic so the system does not jump to a high alert instantly from one frame.

### `alert_console_node`

Subscribes to:

```text
/interaction/features
/suspicion_event
```

It prints clean demo messages such as:

```text
[NORMAL]
[WARNING]
[HIGH ALERT]
```

This makes the class presentation easier to understand than using raw `ros2 topic echo` output.

---

## Useful Debug Commands

After sourcing the workspace inside Docker:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Check packages:

```bash
ros2 pkg list | grep child_safety
```

Expected:

```text
child_safety_monitoring
child_safety_msgs
```

Check custom interfaces:

```bash
ros2 interface list | grep child_safety
```

Expected:

```text
child_safety_msgs/msg/InteractionFeatures
child_safety_msgs/msg/PersonPose2D
child_safety_msgs/msg/PersonPose2DArray
child_safety_msgs/msg/SuspicionEvent
child_safety_msgs/action/StartMonitoring
```

Check running nodes while the demo is active:

```bash
ros2 node list
```

Expected:

```text
/decision_node
/scenario_simulator_node
/alert_console_node
```

Check topics:

```bash
ros2 topic list
```

Important topics:

```text
/interaction/features
/suspicion_event
```

View raw alerts:

```bash
ros2 topic echo /suspicion_event
```

---

## Running Individual Scenarios

You can run only one simulated scenario by passing a parameter.

Example: normal only

```bash
ros2 run child_safety_monitoring scenario_simulator_node --ros-args -p scenario:=normal
```

Example: warning only

```bash
ros2 run child_safety_monitoring scenario_simulator_node --ros-args -p scenario:=warning
```

Example: high alert only

```bash
ros2 run child_safety_monitoring scenario_simulator_node --ros-args -p scenario:=high
```

The full demo launch file uses:

```text
scenario = all
```

which cycles through:

```text
normal → warning → high → normal
```

---

## Troubleshooting

### `docker: command not found`

Docker Desktop is not installed or not running. Install Docker Desktop and open it before running the demo.

### `colcon: command not found`

Inside Docker, install colcon:

```bash
apt update
apt install -y python3-colcon-common-extensions
```

### `Could not find a package configuration file provided by ament_cmake`

ROS 2 is not sourced. Run:

```bash
source /opt/ros/humble/setup.bash
```

### `ros2 pkg list | grep child_safety` shows nothing

Build and source the workspace again:

```bash
cd /root/ros2_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

### The demo prints too many messages

Use the provided launch file:

```bash
ros2 launch child_safety_monitoring scenario_demo.launch.py
```

The simulator and alert console are configured to print only important state changes.

---

## Demo Talking Points for Presentation

Use these points when explaining the demo:

1. The project is a ROS 2 safety-monitoring prototype.
2. The current demo uses simulated interaction features instead of a camera.
3. The simulator creates normal, warning, and high-risk movement patterns.
4. The decision node converts movement features into safety events.
5. The alert console shows clean, human-readable warnings.
6. The output is a suspicion level, not proof of intent or identity.
7. The next development step is connecting real camera pose-estimation output to `/interaction/features`.

---

## Current Demo Status

The current repository demo confirms:

```text
✅ ROS 2 packages build successfully
✅ Custom message interfaces are generated
✅ Scenario simulator publishes feature messages
✅ Decision node publishes warning/high events
✅ Alert console displays clean messages
✅ CTRL + C shutdown is clean
```
