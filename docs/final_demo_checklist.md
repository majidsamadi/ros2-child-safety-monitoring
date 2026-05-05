# Final Demo Checklist

## Safe live camera demo

1. Start webcam streamer.
2. Open `http://127.0.0.1:8090/video`.
3. Start Docker.
4. Run workspace setup.
5. Run `live_laptop_camera_demo.launch.py`.
6. Start `web_video_server`.
7. Open `http://localhost:8080/stream?topic=/camera/pose_overlay`.
8. Confirm two people are detected.
9. Run `check_live_pipeline.sh`.
10. Confirm normal behavior does not produce `/suspicion_event`.

## Simulator demo

Run:

```bash
bash /root/ros2_ws/src/ros2-child-safety-monitoring/scripts/run_simulator_demo.sh
```

Use this to demonstrate:

```text
NORMAL -> WARNING -> HIGH ALERT
```

## What to say in presentation

The live camera part proves the system can:

```text
- read laptop camera
- detect human pose
- track people
- calculate interaction features
- avoid false alarms during normal standing
```

The simulator proves the decision logic can:

```text
- classify warning and high-risk feature patterns
- publish alert messages
- trigger alarm state
```
