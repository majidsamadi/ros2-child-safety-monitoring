# Architecture

The prototype uses a modular ROS 2 perception pipeline:

```text
Camera / Video
    ↓ /camera/image_raw
Pose Estimator Node
    ↓ /poses/raw
Tracker Node
    ↓ /poses/tracked
Interaction Analyzer Node
    ↓ /interaction/features
Decision Node
    ↓ /suspicion_event
Visualization Node
    ↓ /annotated_image
```

## Design principle

The system does not infer intent. It only detects a combination of explainable motion cues that may justify human review.

## Detection state machine

```text
OBSERVING → CONTACT → LIFT_OR_ELEVATION → STRUGGLE_OR_CARRY → WARNING/HIGH_ALERT
```

## Topics

| Topic | Message | Description |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | Input frame |
| `/poses/raw` | `child_safety_msgs/PersonPose2DArray` | Pose detector output |
| `/poses/tracked` | `child_safety_msgs/PersonPose2DArray` | Pose output with track IDs and size roles |
| `/interaction/features` | `child_safety_msgs/InteractionFeatures` | Detection features and current score |
| `/suspicion_event` | `child_safety_msgs/SuspicionEvent` | Warning/high alert event |
| `/annotated_image` | `sensor_msgs/Image` | Debug visualization |

## Initial scoring formula

```text
score = 0.20 * contact
      + 0.15 * wrap
      + 0.25 * lift
      + 0.15 * feet_off_ground
      + 0.15 * struggle
      + 0.10 * co_motion
```

All thresholds are configured in YAML and must be tuned using staged safe test clips.
