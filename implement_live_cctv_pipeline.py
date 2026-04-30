#!/usr/bin/env python3
"""
Patch script for ros2-child-safety-monitoring.

Run this from the repository root:

    python3 implement_live_cctv_pipeline.py

It adds a live CCTV/IP-camera pipeline:
    cctv_stream_node -> pose_estimator_node -> tracker_node -> interaction_analyzer_node
    -> decision_node -> alert_console_node/alarm_node

It keeps the existing scenario simulator demo as a backup.
"""

from __future__ import annotations

from pathlib import Path

ROOT = Path.cwd()
PKG = ROOT / "child_safety_monitoring"
PY_PKG = PKG / "child_safety_monitoring"
NODES = PY_PKG / "nodes"
LAUNCH = PKG / "launch"
CONFIG = PKG / "config"

required = [PKG / "setup.py", PKG / "package.xml", NODES]
missing = [str(p) for p in required if not p.exists()]
if missing:
    raise SystemExit(
        "This script must be run from the ros2-child-safety-monitoring repository root.\n"
        f"Missing: {missing}"
    )

NODES.mkdir(parents=True, exist_ok=True)
LAUNCH.mkdir(parents=True, exist_ok=True)
CONFIG.mkdir(parents=True, exist_ok=True)


def write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    print(f"wrote {path}")


write(
    NODES / "cctv_stream_node.py",
    r'''from __future__ import annotations

import time
from typing import Optional, Union

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CCTVStreamNode(Node):
    """
    Reads a live CCTV/IP-camera stream and publishes ROS Image messages.

    Supported stream_url examples:
      - rtsp://username:password@192.168.1.50:554/stream1
      - http://192.168.1.50/video
      - 0                     # local webcam, usually Linux only inside Docker

    For the final project, use an RTSP/HTTP CCTV URL instead of the simulator.
    """

    def __init__(self) -> None:
        super().__init__('cctv_stream_node')

        self.declare_parameter('stream_url', '')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_frame_id', 'cctv_camera')
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('resize_width', 0)
        self.declare_parameter('resize_height', 0)
        self.declare_parameter('reconnect_delay_seconds', 2.0)
        self.declare_parameter('buffer_size', 1)

        self.stream_url = str(self.get_parameter('stream_url').value).strip()
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_frame_id = str(self.get_parameter('camera_frame_id').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.resize_width = int(self.get_parameter('resize_width').value)
        self.resize_height = int(self.get_parameter('resize_height').value)
        self.reconnect_delay = float(self.get_parameter('reconnect_delay_seconds').value)
        self.buffer_size = int(self.get_parameter('buffer_size').value)

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, self.image_topic, 10)
        self.cap: Optional[cv2.VideoCapture] = None
        self.last_connect_attempt = 0.0

        timer_period = 1.0 / max(self.publish_rate_hz, 0.1)
        self.timer = self.create_timer(timer_period, self.on_timer)

        if not self.stream_url:
            self.get_logger().error(
                'No CCTV stream_url was provided. Launch with: '
                "stream_url:='rtsp://user:password@camera-ip/path'"
            )
        else:
            self.get_logger().info(f'CCTV stream node starting. Publishing to {self.image_topic}')
            self.get_logger().info('Tip: do not commit real CCTV usernames/passwords to GitHub.')

    def _source(self) -> Union[str, int]:
        # Numeric strings are treated as local camera indexes.
        if self.stream_url.isdigit():
            return int(self.stream_url)
        return self.stream_url

    def _connect(self) -> bool:
        now = time.time()
        if now - self.last_connect_attempt < self.reconnect_delay:
            return False
        self.last_connect_attempt = now

        self._release()
        source = self._source()
        self.get_logger().info(f'Connecting to video source: {source}')
        self.cap = cv2.VideoCapture(source)

        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, self.buffer_size)
        except Exception:
            pass

        if not self.cap.isOpened():
            self.get_logger().warn('Could not open CCTV/video source. Will retry...')
            self._release()
            return False

        self.get_logger().info('CCTV/video source connected.')
        return True

    def _release(self) -> None:
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        self.cap = None

    def on_timer(self) -> None:
        if not self.stream_url:
            return

        if self.cap is None or not self.cap.isOpened():
            self._connect()
            return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn('Frame read failed. Reconnecting...')
            self._release()
            return

        if self.resize_width > 0 and self.resize_height > 0:
            frame = cv2.resize(frame, (self.resize_width, self.resize_height))

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame_id
        self.publisher.publish(msg)

    def destroy_node(self) -> bool:
        self._release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CCTVStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
''',
)

write(
    NODES / "pose_estimator_node.py",
    r'''from __future__ import annotations

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point32
from rclpy.node import Node
from sensor_msgs.msg import Image, RegionOfInterest

from child_safety_msgs.msg import PersonPose2D, PersonPose2DArray
from child_safety_monitoring.core.keypoints import LEFT_HIP, LEFT_SHOULDER, RIGHT_HIP, RIGHT_SHOULDER


class PoseEstimatorNode(Node):
    """
    YOLO pose-estimation node.

    Subscribes:
      /camera/image_raw

    Publishes:
      /poses/raw
      /camera/pose_overlay  optional annotated image
    """

    def __init__(self) -> None:
        super().__init__('pose_estimator_node')

        self.declare_parameter('model_path', 'yolo11n-pose.pt')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('raw_pose_topic', '/poses/raw')
        self.declare_parameter('annotated_image_topic', '/camera/pose_overlay')
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('frame_skip', 0)
        self.declare_parameter('keypoint_confidence_threshold', 0.25)

        self.model_path = str(self.get_parameter('model_path').value)
        self.conf = float(self.get_parameter('confidence_threshold').value)
        self.publish_annotated = bool(self.get_parameter('publish_annotated').value)
        self.device = str(self.get_parameter('device').value)
        self.frame_skip = int(self.get_parameter('frame_skip').value)
        self.kpt_conf_threshold = float(self.get_parameter('keypoint_confidence_threshold').value)
        self.frame_count = 0

        self.bridge = CvBridge()
        self.pose_pub = self.create_publisher(
            PersonPose2DArray,
            str(self.get_parameter('raw_pose_topic').value),
            10,
        )
        self.overlay_pub = self.create_publisher(
            Image,
            str(self.get_parameter('annotated_image_topic').value),
            10,
        )
        self.subscription = self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self.on_image,
            10,
        )

        try:
            from ultralytics import YOLO
            self.model = YOLO(self.model_path)
            self.get_logger().info(f'Loaded YOLO pose model: {self.model_path} on device={self.device}')
        except Exception as exc:
            self.model = None
            self.get_logger().error(
                f'Failed to load YOLO pose model: {exc}. '
                'Install dependencies with: pip3 install -r requirements.txt'
            )

    def on_image(self, image_msg: Image) -> None:
        if self.model is None:
            return

        self.frame_count += 1
        if self.frame_skip > 0 and (self.frame_count % (self.frame_skip + 1)) != 1:
            return

        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        try:
            result = self.model.predict(
                frame,
                conf=self.conf,
                verbose=False,
                device=self.device,
            )[0]
        except Exception as exc:
            self.get_logger().error(f'YOLO prediction failed: {exc}')
            return

        out = PersonPose2DArray()
        out.header = image_msg.header

        boxes = result.boxes.xyxy.cpu().numpy() if result.boxes is not None else np.empty((0, 4))
        kxy = result.keypoints.xy.cpu().numpy() if result.keypoints is not None else np.empty((0, 17, 2))
        kcf = result.keypoints.conf.cpu().numpy() if result.keypoints is not None else np.empty((0, 17))

        for i, box in enumerate(boxes):
            if i >= len(kxy):
                continue

            x1, y1, x2, y2 = box
            width = max(int(x2 - x1), 0)
            height = max(int(y2 - y1), 0)
            if width <= 0 or height <= 0:
                continue

            person = PersonPose2D()
            person.header = image_msg.header
            person.track_id = ''
            person.size_role = 'unknown'
            person.size_confidence = 0.0
            person.bbox = RegionOfInterest(
                x_offset=max(int(x1), 0),
                y_offset=max(int(y1), 0),
                width=width,
                height=height,
                do_rectify=False,
            )
            person.keypoints_xy = [Point32(x=float(x), y=float(y), z=0.0) for x, y in kxy[i]]
            person.keypoint_confidence = [float(c) for c in kcf[i]]
            person.visible = [float(c) > self.kpt_conf_threshold for c in kcf[i]]
            person.center_depth_m = 0.0
            person.torso_length_px = self._torso_len(person)
            out.poses.append(person)

        self.pose_pub.publish(out)

        if self.publish_annotated:
            try:
                overlay = result.plot()
                overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
                overlay_msg.header = image_msg.header
                self.overlay_pub.publish(overlay_msg)
            except Exception as exc:
                self.get_logger().warn(f'Could not publish annotated image: {exc}')

    @staticmethod
    def _torso_len(person: PersonPose2D) -> float:
        try:
            sy = (person.keypoints_xy[LEFT_SHOULDER].y + person.keypoints_xy[RIGHT_SHOULDER].y) / 2.0
            hy = (person.keypoints_xy[LEFT_HIP].y + person.keypoints_xy[RIGHT_HIP].y) / 2.0
            return max(abs(hy - sy), 1.0)
        except Exception:
            return max(float(person.bbox.height) * 0.35, 1.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
''',
)

write(
    NODES / "tracker_node.py",
    r'''from __future__ import annotations

import rclpy
from rclpy.node import Node

from child_safety_msgs.msg import PersonPose2DArray
from child_safety_monitoring.core.centroid_tracker import CentroidTracker
from child_safety_monitoring.core.geometry import bbox_center


class TrackerNode(Node):
    """
    Adds stable track IDs and assigns simple size roles.

    For the prototype, the smaller bounding box is treated as smaller_candidate
    and the larger bounding box as larger_candidate. This is not real age recognition.
    """

    def __init__(self) -> None:
        super().__init__('tracker_node')
        self.declare_parameter('raw_pose_topic', '/poses/raw')
        self.declare_parameter('tracked_pose_topic', '/poses/tracked')
        self.declare_parameter('max_distance_px', 140.0)
        self.declare_parameter('max_missed_frames', 10)
        self.declare_parameter('min_people_required', 2)

        self.tracker = CentroidTracker(
            max_distance_px=float(self.get_parameter('max_distance_px').value),
            max_missed=int(self.get_parameter('max_missed_frames').value),
        )
        self.min_people_required = int(self.get_parameter('min_people_required').value)

        self.subscription = self.create_subscription(
            PersonPose2DArray,
            str(self.get_parameter('raw_pose_topic').value),
            self.on_raw_poses,
            10,
        )
        self.publisher = self.create_publisher(
            PersonPose2DArray,
            str(self.get_parameter('tracked_pose_topic').value),
            10,
        )

    def on_raw_poses(self, msg: PersonPose2DArray) -> None:
        detections = []
        for pose in msg.poses:
            center = bbox_center(pose.bbox.x_offset, pose.bbox.y_offset, pose.bbox.width, pose.bbox.height)
            detections.append((center, float(pose.bbox.height)))

        assignments = self.tracker.update(detections)
        for det_idx, track_id in assignments.items():
            if det_idx < len(msg.poses):
                msg.poses[det_idx].track_id = track_id

        for pose in msg.poses:
            pose.size_role = 'unknown'
            pose.size_confidence = 0.0

        if len(msg.poses) >= self.min_people_required:
            sorted_indices = sorted(range(len(msg.poses)), key=lambda i: msg.poses[i].bbox.height)
            smaller_idx = sorted_indices[0]
            larger_idx = sorted_indices[-1]

            for i, pose in enumerate(msg.poses):
                if i == smaller_idx:
                    pose.size_role = 'smaller_candidate'
                    pose.size_confidence = 0.75
                elif i == larger_idx:
                    pose.size_role = 'larger_candidate'
                    pose.size_confidence = 0.75
                else:
                    pose.size_role = 'bystander'
                    pose.size_confidence = 0.50

        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
''',
)

write(
    NODES / "interaction_analyzer_node.py",
    r'''from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point32
from rclpy.node import Node

from child_safety_msgs.msg import InteractionFeatures, PersonPose2D, PersonPose2DArray
from child_safety_monitoring.core.geometry import Point2, bbox_center, distance, ramp_score, safe_mean
from child_safety_monitoring.core.keypoints import (
    LEFT_ANKLE, LEFT_HIP, LEFT_SHOULDER, LIMB_MOTION_JOINTS,
    RIGHT_ANKLE, RIGHT_HIP, RIGHT_SHOULDER,
)
from child_safety_monitoring.core.scoring import FeatureScores, calculate_suspicion_score, state_from_score


@dataclass
class PoseSnapshot:
    stamp_sec: float
    pose: PersonPose2D


class InteractionAnalyzerNode(Node):
    """
    Converts tracked human poses into suspicious-interaction features.

    This is a rule-based prototype. It does not infer intent or prove a crime.
    It only measures pose/motion cues such as close contact, lift, limb motion,
    and adult-child co-movement.
    """

    def __init__(self) -> None:
        super().__init__('interaction_analyzer_node')

        self.declare_parameter('tracked_pose_topic', '/poses/tracked')
        self.declare_parameter('features_topic', '/interaction/features')
        self.declare_parameter('history_window_seconds', 1.25)
        self.declare_parameter('contact_distance_threshold_norm', 1.0)
        self.declare_parameter('lift_velocity_threshold_norm_per_sec', 0.45)
        self.declare_parameter('feet_off_ground_threshold_norm', 0.18)
        self.declare_parameter('limb_speed_low_norm_per_sec', 1.20)
        self.declare_parameter('limb_speed_high_norm_per_sec', 2.30)
        self.declare_parameter('limb_accel_low_norm_per_sec2', 3.50)
        self.declare_parameter('limb_accel_high_norm_per_sec2', 7.50)
        self.declare_parameter('co_motion_threshold', 0.30)
        self.declare_parameter('score_warning_threshold', 0.55)
        self.declare_parameter('score_high_threshold', 0.75)

        self.history_window = float(self.get_parameter('history_window_seconds').value)
        self.contact_threshold = float(self.get_parameter('contact_distance_threshold_norm').value)
        self.lift_velocity_threshold = float(self.get_parameter('lift_velocity_threshold_norm_per_sec').value)
        self.feet_off_ground_threshold = float(self.get_parameter('feet_off_ground_threshold_norm').value)
        self.limb_speed_low = float(self.get_parameter('limb_speed_low_norm_per_sec').value)
        self.limb_speed_high = float(self.get_parameter('limb_speed_high_norm_per_sec').value)
        self.limb_accel_low = float(self.get_parameter('limb_accel_low_norm_per_sec2').value)
        self.limb_accel_high = float(self.get_parameter('limb_accel_high_norm_per_sec2').value)
        self.co_motion_threshold = float(self.get_parameter('co_motion_threshold').value)
        self.warning_threshold = float(self.get_parameter('score_warning_threshold').value)
        self.high_threshold = float(self.get_parameter('score_high_threshold').value)

        self.history: Dict[str, Deque[PoseSnapshot]] = defaultdict(lambda: deque(maxlen=90))
        self.floor_y_by_track: Dict[str, float] = {}

        self.subscription = self.create_subscription(
            PersonPose2DArray,
            str(self.get_parameter('tracked_pose_topic').value),
            self.on_tracked_poses,
            10,
        )
        self.publisher = self.create_publisher(
            InteractionFeatures,
            str(self.get_parameter('features_topic').value),
            10,
        )

    def on_tracked_poses(self, msg: PersonPose2DArray) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9

        for pose in msg.poses:
            if pose.track_id:
                self.history[pose.track_id].append(PoseSnapshot(now_sec, pose))
                self._update_floor_proxy(pose)
                self._trim_history(pose.track_id, now_sec)

        smaller = self._find_role(msg.poses, 'smaller_candidate')
        larger = self._find_role(msg.poses, 'larger_candidate')
        if smaller is None or larger is None:
            return

        values = self._calculate_features(smaller, larger)
        out = InteractionFeatures()
        out.header = msg.header
        out.smaller_track_id = smaller.track_id
        out.larger_track_id = larger.track_id
        out.torso_distance_norm = float(values['torso_distance_norm'])
        out.wrap_score = float(values['wrap_score'])
        out.lift_score = float(values['lift_score'])
        out.feet_off_ground_score = float(values['feet_off_ground_score'])
        out.limb_speed_score = float(values['limb_speed_score'])
        out.limb_accel_score = float(values['limb_accel_score'])
        out.co_motion_score = float(values['co_motion_score'])
        out.suspicion_score = float(values['suspicion_score'])
        out.state = str(values['state'])
        self.publisher.publish(out)

    def _trim_history(self, track_id: str, now_sec: float) -> None:
        h = self.history[track_id]
        while h and now_sec - h[0].stamp_sec > self.history_window:
            h.popleft()

    @staticmethod
    def _find_role(poses: List[PersonPose2D], role: str) -> Optional[PersonPose2D]:
        matches = [p for p in poses if p.size_role == role]
        return matches[0] if matches else None

    def _calculate_features(self, child: PersonPose2D, adult: PersonPose2D) -> Dict[str, float | str]:
        child_center = self._torso_center(child)
        adult_center = self._torso_center(adult)
        norm = max(child.torso_length_px, child.bbox.height * 0.25, 1.0)
        dist_norm = distance(child_center, adult_center) / norm

        contact = 1.0 if dist_norm <= self.contact_threshold else 0.0
        wrap = 1.0 if dist_norm < 0.7 else 0.5 if dist_norm < 1.2 else 0.0
        lift = self._lift_score(child)
        feet = self._feet_off_ground_proxy(child)
        limb_speed, limb_accel = self._limb_motion_scores(child)
        comotion = self._co_motion_score(child, adult)

        score = calculate_suspicion_score(
            FeatureScores(contact, wrap, lift, feet, limb_speed, limb_accel, comotion)
        )

        return {
            'torso_distance_norm': float(dist_norm),
            'wrap_score': float(wrap),
            'lift_score': float(lift),
            'feet_off_ground_score': float(feet),
            'limb_speed_score': float(limb_speed),
            'limb_accel_score': float(limb_accel),
            'co_motion_score': float(comotion),
            'suspicion_score': float(score),
            'state': state_from_score(score, self.warning_threshold, self.high_threshold),
        }

    @staticmethod
    def _point(pose: PersonPose2D, index: int) -> Point2:
        if index >= len(pose.keypoints_xy):
            return bbox_center(pose.bbox.x_offset, pose.bbox.y_offset, pose.bbox.width, pose.bbox.height)
        p: Point32 = pose.keypoints_xy[index]
        return Point2(p.x, p.y)

    def _torso_center(self, pose: PersonPose2D) -> Point2:
        pts = [self._point(pose, i) for i in [LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_HIP, RIGHT_HIP]]
        return Point2(safe_mean([p.x for p in pts]), safe_mean([p.y for p in pts]))

    def _ankle_y(self, pose: PersonPose2D) -> Optional[float]:
        if len(pose.keypoints_xy) <= RIGHT_ANKLE:
            return None
        return max(self._point(pose, LEFT_ANKLE).y, self._point(pose, RIGHT_ANKLE).y)

    def _update_floor_proxy(self, pose: PersonPose2D) -> None:
        if not pose.track_id:
            return
        ankle_y = self._ankle_y(pose)
        if ankle_y is None:
            return
        previous = self.floor_y_by_track.get(pose.track_id, ankle_y)
        # In image coordinates, larger y is lower in the image. The largest observed
        # ankle y approximates the standing/floor level for that track.
        self.floor_y_by_track[pose.track_id] = max(previous, ankle_y)

    def _lift_score(self, child: PersonPose2D) -> float:
        h = list(self.history.get(child.track_id, []))
        if len(h) < 3:
            return 0.0
        first, last = h[0], h[-1]
        dt = max(last.stamp_sec - first.stamp_sec, 1e-3)
        upward_velocity_norm = (
            self._torso_center(first.pose).y - self._torso_center(last.pose).y
        ) / max(child.torso_length_px, 1.0) / dt
        return ramp_score(
            upward_velocity_norm,
            self.lift_velocity_threshold,
            self.lift_velocity_threshold * 1.8,
        )

    def _feet_off_ground_proxy(self, child: PersonPose2D) -> float:
        ankle_y = self._ankle_y(child)
        if ankle_y is None:
            return 0.0
        fallback_floor = float(child.bbox.y_offset + child.bbox.height)
        floor_y = self.floor_y_by_track.get(child.track_id, fallback_floor)
        gap_norm = (floor_y - ankle_y) / max(child.bbox.height, 1.0)
        return ramp_score(gap_norm, self.feet_off_ground_threshold, self.feet_off_ground_threshold * 1.8)

    def _limb_motion_scores(self, child: PersonPose2D) -> Tuple[float, float]:
        h = list(self.history.get(child.track_id, []))
        if len(h) < 3:
            return 0.0, 0.0

        speeds, accels = [], []
        for idx in LIMB_MOTION_JOINTS:
            samples = [(s.stamp_sec, self._point(s.pose, idx)) for s in h]
            joint_speeds = []
            for (t1, p1), (t2, p2) in zip(samples, samples[1:]):
                joint_speeds.append(
                    distance(p1, p2) / max(child.torso_length_px, 1.0) / max(t2 - t1, 1e-3)
                )
            if joint_speeds:
                speeds.append(safe_mean(joint_speeds))
            if len(joint_speeds) >= 2:
                accels.append(abs(joint_speeds[-1] - joint_speeds[0]) / max(samples[-1][0] - samples[0][0], 1e-3))

        return (
            ramp_score(safe_mean(speeds), self.limb_speed_low, self.limb_speed_high),
            ramp_score(safe_mean(accels), self.limb_accel_low, self.limb_accel_high),
        )

    def _co_motion_score(self, child: PersonPose2D, adult: PersonPose2D) -> float:
        ch = list(self.history.get(child.track_id, []))
        ah = list(self.history.get(adult.track_id, []))
        if len(ch) < 2 or len(ah) < 2:
            return 0.0

        child_dx = self._torso_center(ch[-1].pose).x - self._torso_center(ch[0].pose).x
        adult_dx = self._torso_center(ah[-1].pose).x - self._torso_center(ah[0].pose).x
        child_dy = self._torso_center(ch[-1].pose).y - self._torso_center(ch[0].pose).y
        adult_dy = self._torso_center(ah[-1].pose).y - self._torso_center(ah[0].pose).y
        same_direction = 1.0 if (child_dx * adult_dx + child_dy * adult_dy) > 0 else 0.0
        movement = min(
            (child_dx ** 2 + child_dy ** 2) ** 0.5,
            (adult_dx ** 2 + adult_dy ** 2) ** 0.5,
        ) / max(child.torso_length_px, 1.0)
        return same_direction * ramp_score(movement, self.co_motion_threshold, self.co_motion_threshold * 2.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InteractionAnalyzerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
''',
)

write(
    NODES / "alarm_node.py",
    r'''from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from child_safety_msgs.msg import InteractionFeatures, SuspicionEvent


class AlarmNode(Node):
    """
    Simple alarm state node.

    Publishes /alarm/state:
      OFF, WARNING, ON

    In a real deployment, this node is where a buzzer, dashboard, webhook,
    or security-room notification would be connected. For the class prototype,
    it publishes a ROS topic and logs the state change.
    """

    def __init__(self) -> None:
        super().__init__('alarm_node')
        self.declare_parameter('normal_threshold', 0.55)
        self.normal_threshold = float(self.get_parameter('normal_threshold').value)
        self.state = 'OFF'

        self.publisher = self.create_publisher(String, '/alarm/state', 10)
        self.feature_subscription = self.create_subscription(
            InteractionFeatures,
            '/interaction/features',
            self.on_features,
            10,
        )
        self.event_subscription = self.create_subscription(
            SuspicionEvent,
            '/suspicion_event',
            self.on_event,
            10,
        )
        self.get_logger().info('Alarm node started. Publishing /alarm/state.')
        self._set_state('OFF')

    def on_features(self, msg: InteractionFeatures) -> None:
        if msg.suspicion_score < self.normal_threshold:
            self._set_state('OFF')

    def on_event(self, msg: SuspicionEvent) -> None:
        if msg.level.lower() == 'high':
            self._set_state('ON')
        elif msg.level.lower() == 'warning' and self.state != 'ON':
            self._set_state('WARNING')

    def _set_state(self, new_state: str) -> None:
        if new_state == self.state:
            return
        self.state = new_state
        msg = String()
        msg.data = new_state
        self.publisher.publish(msg)

        if new_state == 'ON':
            self.get_logger().error('[ALARM ON] High-risk suspicious lifting pattern detected')
        elif new_state == 'WARNING':
            self.get_logger().warn('[ALARM WARNING] Suspicious interaction pattern detected')
        else:
            self.get_logger().info('[ALARM OFF] Monitoring normally')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AlarmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
''',
)

write(
    LAUNCH / "live_cctv_demo.launch.py",
    r'''from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    stream_url = LaunchConfiguration('stream_url')
    model_path = LaunchConfiguration('model_path')
    device = LaunchConfiguration('device')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'stream_url',
            default_value='',
            description='CCTV/IP-camera stream URL, usually RTSP. Example: rtsp://user:pass@192.168.1.50:554/stream1',
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value='yolo11n-pose.pt',
            description='YOLO pose model path/name.',
        ),
        DeclareLaunchArgument(
            'device',
            default_value='cpu',
            description='YOLO device, e.g. cpu, cuda, cuda:0.',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='12.0',
            description='CCTV frame publish rate.',
        ),

        Node(
            package='child_safety_monitoring',
            executable='cctv_stream_node',
            name='cctv_stream_node',
            output='screen',
            parameters=[{
                'stream_url': stream_url,
                'publish_rate_hz': publish_rate_hz,
                'image_topic': '/camera/image_raw',
                'resize_width': 960,
                'resize_height': 540,
                'buffer_size': 1,
            }],
        ),
        Node(
            package='child_safety_monitoring',
            executable='pose_estimator_node',
            name='pose_estimator_node',
            output='screen',
            parameters=[{
                'model_path': model_path,
                'device': device,
                'confidence_threshold': 0.35,
                'image_topic': '/camera/image_raw',
                'raw_pose_topic': '/poses/raw',
                'publish_annotated': True,
                'annotated_image_topic': '/camera/pose_overlay',
                'frame_skip': 0,
            }],
        ),
        Node(
            package='child_safety_monitoring',
            executable='tracker_node',
            name='tracker_node',
            output='screen',
            parameters=[{
                'raw_pose_topic': '/poses/raw',
                'tracked_pose_topic': '/poses/tracked',
                'max_distance_px': 140.0,
                'max_missed_frames': 10,
            }],
        ),
        Node(
            package='child_safety_monitoring',
            executable='interaction_analyzer_node',
            name='interaction_analyzer_node',
            output='screen',
        ),
        Node(
            package='child_safety_monitoring',
            executable='decision_node',
            name='decision_node',
            output='screen',
        ),
        Node(
            package='child_safety_monitoring',
            executable='alert_console_node',
            name='alert_console_node',
            output='screen',
        ),
        Node(
            package='child_safety_monitoring',
            executable='alarm_node',
            name='alarm_node',
            output='screen',
        ),
    ])
''',
)

write(
    CONFIG / "live_cctv_params.yaml",
    r'''cctv_stream_node:
  ros__parameters:
    image_topic: /camera/image_raw
    camera_frame_id: cctv_camera
    publish_rate_hz: 12.0
    resize_width: 960
    resize_height: 540
    reconnect_delay_seconds: 2.0
    buffer_size: 1

pose_estimator_node:
  ros__parameters:
    model_path: yolo11n-pose.pt
    confidence_threshold: 0.35
    keypoint_confidence_threshold: 0.25
    image_topic: /camera/image_raw
    raw_pose_topic: /poses/raw
    publish_annotated: true
    annotated_image_topic: /camera/pose_overlay
    device: cpu
    frame_skip: 0

tracker_node:
  ros__parameters:
    raw_pose_topic: /poses/raw
    tracked_pose_topic: /poses/tracked
    max_distance_px: 140.0
    max_missed_frames: 10
    min_people_required: 2

interaction_analyzer_node:
  ros__parameters:
    history_window_seconds: 1.25
    contact_distance_threshold_norm: 1.0
    lift_velocity_threshold_norm_per_sec: 0.45
    feet_off_ground_threshold_norm: 0.18
    limb_speed_low_norm_per_sec: 1.20
    limb_speed_high_norm_per_sec: 2.30
    limb_accel_low_norm_per_sec2: 3.50
    limb_accel_high_norm_per_sec2: 7.50
    co_motion_threshold: 0.30
    score_warning_threshold: 0.55
    score_high_threshold: 0.75
''',
)

# Update setup.py entry points.
setup_path = PKG / "setup.py"
setup_text = setup_path.read_text(encoding="utf-8")
entries = [
    "            'cctv_stream_node = child_safety_monitoring.nodes.cctv_stream_node:main',",
    "            'alarm_node = child_safety_monitoring.nodes.alarm_node:main',",
]

for entry in entries:
    if entry not in setup_text:
        marker = "            'video_source_node = child_safety_monitoring.nodes.video_source_node:main',"
        if marker in setup_text and "cctv_stream_node" in entry:
            setup_text = setup_text.replace(marker, entry + "\n" + marker)
        else:
            # Put remaining new entries before the closing console_scripts list.
            setup_text = setup_text.replace("        ],\n    },", entry + "\n        ],\n    },")

setup_path.write_text(setup_text, encoding="utf-8")
print(f"updated {setup_path}")

# Update requirements.txt without pinning too aggressively.
req_path = ROOT / "requirements.txt"
existing = req_path.read_text(encoding="utf-8").splitlines() if req_path.exists() else []
need = [
    "ultralytics>=8.3.0",
    "numpy>=1.23.0",
]
merged = existing[:]
for dep in need:
    name = dep.split(">=")[0].lower()
    if not any(line.strip().lower().startswith(name) for line in merged):
        merged.append(dep)
req_path.write_text("\n".join([line for line in merged if line.strip()]) + "\n", encoding="utf-8")
print(f"updated {req_path}")

# Add a docs file.
write(
    ROOT / "docs" / "live_cctv_pipeline.md",
    r'''# Live CCTV Pipeline

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
''',
)

print("\nLive CCTV implementation files added. Next steps:")
print("  git status")
print("  git add .")
print("  git commit -m 'Add live CCTV detection pipeline'")
print("  git push")
