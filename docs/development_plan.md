# Development Plan

## Phase 1 — Repository and build

- Create ROS 2 packages.
- Add custom messages and action.
- Add launch files and configuration.
- Confirm `colcon build` passes.

## Phase 2 — Camera/video input

- Publish webcam frames to `/camera/image_raw`.
- Support video-file playback.
- Confirm frames can be opened with `rqt_image_view`.

## Phase 3 — Pose estimation

- Use YOLO pose as the first multi-person backend.
- Publish person boxes and COCO keypoints.
- Confirm two-person detection in staged videos.

## Phase 4 — Tracking and role assignment

- Assign stable track IDs using centroid matching.
- Assign smaller/larger candidate roles based on bounding-box height.
- Add smoothing to avoid role flickering.

## Phase 5 — Interaction analysis

- Detect sudden close contact.
- Detect vertical lift/elevation.
- Detect rapid wrist and ankle motion.
- Calculate suspicion score.

## Phase 6 — Demo and evaluation

- Create safe staged test clips.
- Evaluate false positives on normal hug/play clips.
- Record alert latency.
- Prepare final demo and report.
