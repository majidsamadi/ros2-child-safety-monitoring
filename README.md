# README Demo Section — Cross-Platform Setup

Paste this section into the project `README.md` under a heading such as **Demo / Quick Start**.

This project can be demonstrated by all team members on **macOS**, **Windows**, and **Linux**. The recommended team-wide method is **Docker**, because it gives everyone the same Ubuntu + ROS 2 Humble environment.

---

## 1. What the demo does

The demo runs three ROS 2 nodes:

| Node | Purpose |
|---|---|
| `scenario_simulator_node` | Simulates three detection states: normal, warning, and high alert |
| `decision_node` | Converts interaction features into warning/high suspicion events |
| `alert_console_node` | Prints clean demo messages for presentation |

Expected demo cycle:

```text
NORMAL → WARNING → HIGH ALERT → NORMAL
```

Expected console style:

```text
Changed scenario → NORMAL | score=0.10, state=normal
[NORMAL] score=0.10 | No suspicious interaction pattern detected

Changed scenario → WARNING | score=0.62, state=warning
[WARNING] score=0.62 | Suspicious interaction pattern detected

Changed scenario → HIGH | score=0.92, state=high_alert
[HIGH ALERT] score=0.92 | Suspicious child-lifting pattern detected
```

This proves that the current ROS 2 pipeline works:

```text
Simulated interaction features → Decision node → Suspicion event → Console alert
```

---

## 2. Recommended setup for the whole team: Docker

Use this option for:

- macOS
- Windows
- Linux

Docker avoids differences between operating systems because the project runs inside an Ubuntu ROS 2 Humble container.

### Required software

Install Docker first:

- macOS: install Docker Desktop for Mac.
- Windows: install Docker Desktop for Windows and use the WSL 2 backend.
- Linux: install Docker Engine or Docker Desktop for Linux.

Official references:

- [ROS 2 Humble installation documentation](https://docs.ros.org/en/humble/Installation.html)
- [Docker Desktop for Mac](https://docs.docker.com/installation/mac/)
- [Docker Desktop for Windows](https://docs.docker.com/desktop/setup/install/windows-install/)
- [Docker Desktop WSL 2 backend](https://docs.docker.com/desktop/features/wsl/)
- [Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/)

---

## 3. Clone the repository

### macOS / Linux terminal

```bash
git clone https://github.com/majidsamadi/ros2-child-safety-monitoring.git
cd ros2-child-safety-monitoring
```

Then move one folder above the repository before starting Docker:

```bash
cd ..
```

You should now be in the parent folder containing:

```text
ros2-child-safety-monitoring
```

### Windows PowerShell

Choose a simple folder such as `Documents`:

```powershell
cd $env:USERPROFILE\Documents
git clone https://github.com/majidsamadi/ros2-child-safety-monitoring.git
```

Stay in the parent folder:

```powershell
cd $env:USERPROFILE\Documents
```

You should now be in the folder containing:

```text
ros2-child-safety-monitoring
```

### Windows WSL 2 Ubuntu terminal

If using WSL 2 Ubuntu, clone the repository inside the WSL home directory:

```bash
cd ~
git clone https://github.com/majidsamadi/ros2-child-safety-monitoring.git
cd ~
```

Then use the Linux/macOS Docker command below.

---

## 4. Start the ROS 2 Docker container

### macOS / Linux / WSL 2 Ubuntu

Run this from the folder **above** `ros2-child-safety-monitoring`:

```bash
docker run -it --rm \
  --name ros2-child-safety-dev \
  -v "$PWD/ros2-child-safety-monitoring:/root/ros2_ws/src/ros2-child-safety-monitoring" \
  osrf/ros:humble-desktop \
  bash
```

On Apple Silicon Macs, Docker may show a warning like:

```text
The requested image's platform (linux/amd64) does not match the detected host platform
```

That warning is acceptable for this class demo. The container may run slower, but the demo should still work.

### Windows PowerShell

Run this from the folder **above** `ros2-child-safety-monitoring`:

```powershell
docker run -it --rm `
  --name ros2-child-safety-dev `
  -v "${PWD}\ros2-child-safety-monitoring:/root/ros2_ws/src/ros2-child-safety-monitoring" `
  osrf/ros:humble-desktop `
  bash
```

### Windows Command Prompt alternative

```cmd
docker run -it --rm ^
  --name ros2-child-safety-dev ^
  -v "%cd%\ros2-child-safety-monitoring:/root/ros2_ws/src/ros2-child-safety-monitoring" ^
  osrf/ros:humble-desktop ^
  bash
```

After this, the terminal should change to something like:

```text
root@container_id:/#
```

That means you are inside the ROS 2 Docker container.

---

## 5. Build the project inside Docker

Run these commands **inside the Docker container**:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
apt update
apt install -y python3-pip python3-colcon-common-extensions ros-humble-vision-msgs
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

Expected build result:

```text
Summary: 2 packages finished
```

Check that ROS 2 can see the packages:

```bash
ros2 pkg list | grep child_safety
```

Expected output:

```text
child_safety_monitoring
child_safety_msgs
```

---

## 6. Run the scenario demo

Inside Docker, run:

```bash
ros2 launch child_safety_monitoring scenario_demo.launch.py
```

Expected output:

```text
Changed scenario → NORMAL | score=0.10, state=normal
[NORMAL] score=0.10 | No suspicious interaction pattern detected

Changed scenario → WARNING | score=0.62, state=warning
[WARNING] score=0.62 | Suspicious interaction pattern detected

Changed scenario → HIGH | score=0.92, state=high_alert
[HIGH ALERT] score=0.92 | Suspicious child-lifting pattern detected
```

Stop the demo with:

```text
CTRL + C
```

Expected clean shutdown:

```text
process has finished cleanly
```

---

## 7. Native Linux option: Ubuntu 22.04 with ROS 2 Humble

Use this only if the Linux teammate already has Ubuntu 22.04 and wants to run ROS 2 directly without Docker.

### Install ROS 2 Humble dependencies

Follow the official ROS 2 Humble Ubuntu deb installation guide first:

- [ROS 2 Humble Ubuntu deb packages](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

Then install the project build dependencies:

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions ros-humble-vision-msgs
```

### Create a ROS 2 workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/majidsamadi/ros2-child-safety-monitoring.git
```

### Build and run

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
ros2 launch child_safety_monitoring scenario_demo.launch.py
```

---

## 8. Troubleshooting

### Problem: `docker: command not found`

Docker is not installed, or the terminal was opened before Docker was added to PATH.

Fix:

- Install Docker Desktop or Docker Engine.
- Restart the terminal.
- Check:

```bash
docker --version
```

### Problem: Docker is installed but not running

Check:

```bash
docker ps
```

If it fails:

- macOS/Windows: open Docker Desktop and wait until it fully starts.
- Linux: start Docker service if needed.

### Problem: container name already exists

If you see:

```text
The container name "ros2-child-safety-dev" is already in use
```

Run:

```bash
docker rm -f ros2-child-safety-dev
```

Then start the container again.

### Problem: `colcon: command not found` inside Docker

Run:

```bash
apt update
apt install -y python3-colcon-common-extensions
```

### Problem: `ament_cmake` or ROS packages not found

Usually ROS 2 was not sourced.

Run:

```bash
source /opt/ros/humble/setup.bash
```

Then rebuild:

```bash
cd /root/ros2_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

### Problem: `ros2 launch` cannot find the package

You probably forgot to source the workspace:

```bash
cd /root/ros2_ws
source install/setup.bash
```

Then try again:

```bash
ros2 launch child_safety_monitoring scenario_demo.launch.py
```

### Problem: Windows path or volume mount does not work

Use PowerShell and run Docker from the parent folder of the repository.

Good example:

```powershell
cd $env:USERPROFILE\Documents
```

Then run the Windows PowerShell Docker command from this README section.

Avoid very complex paths while debugging, especially paths with unusual characters. Spaces are usually okay when the path is quoted, but simple paths are easier for the first demo.

---

## 9. Short explanation for presentation

Say this during the demo:

> This is our ROS 2 child-safety monitoring prototype. For the class demo, we are not using a real camera yet. Instead, a scenario simulator publishes fake interaction features for three states: normal, warning, and high alert. The decision node applies threshold and persistence logic, then publishes a suspicion event. The alert console node prints a clean output showing when the system detects normal, warning, or high-alert behavior.

Important framing:

> The system does not prove kidnapping or criminal intent. It only detects suspicious movement patterns such as close contact, lift-like motion, feet-off-ground cues, and rapid limb movement. A real-world system would always require human verification.

---

## 10. Current demo command summary

### macOS / Linux / WSL 2 Ubuntu

```bash
cd /path/to/parent/folder

docker run -it --rm \
  --name ros2-child-safety-dev \
  -v "$PWD/ros2-child-safety-monitoring:/root/ros2_ws/src/ros2-child-safety-monitoring" \
  osrf/ros:humble-desktop \
  bash
```

### Windows PowerShell

```powershell
cd C:\path\to\parent\folder

docker run -it --rm `
  --name ros2-child-safety-dev `
  -v "${PWD}\ros2-child-safety-monitoring:/root/ros2_ws/src/ros2-child-safety-monitoring" `
  osrf/ros:humble-desktop `
  bash
```

### Inside Docker

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

## Laptop camera live demo

For the current class demo, the laptop webcam is streamed from the host machine and read by Docker:

```text
Laptop webcam -> scripts/host_webcam_streamer.py -> http://host.docker.internal:8090/video -> ROS pipeline
```

Run the webcam streamer on the host laptop first:

```bash
python3 scripts/host_webcam_streamer.py --camera 0 --port 8090
```

Then run the live ROS pipeline inside Docker:

```bash
ros2 launch child_safety_monitoring live_laptop_camera_demo.launch.py \
  stream_url:='http://host.docker.internal:8090/video'
```

The old `live_cctv_demo.launch.py` file is kept as a compatibility alias, but the laptop-camera launch name is clearer for our project.
