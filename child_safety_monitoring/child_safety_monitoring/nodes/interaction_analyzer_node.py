from __future__ import annotations

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
    def __init__(self) -> None:
        super().__init__('interaction_analyzer_node')
        self.declare_parameter('history_window_seconds', 0.75)
        self.declare_parameter('contact_distance_threshold_norm', 1.0)
        self.declare_parameter('lift_velocity_threshold_norm_per_sec', 0.60)
        self.declare_parameter('feet_off_ground_threshold_norm', 0.25)
        self.declare_parameter('limb_speed_low_norm_per_sec', 1.50)
        self.declare_parameter('limb_speed_high_norm_per_sec', 2.50)
        self.declare_parameter('limb_accel_low_norm_per_sec2', 4.00)
        self.declare_parameter('limb_accel_high_norm_per_sec2', 8.00)
        self.declare_parameter('co_motion_threshold', 0.35)
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
        self.history: Dict[str, Deque[PoseSnapshot]] = defaultdict(lambda: deque(maxlen=60))
        self.subscription = self.create_subscription(PersonPose2DArray, '/poses/tracked', self.on_tracked_poses, 10)
        self.publisher = self.create_publisher(InteractionFeatures, '/interaction/features', 10)

    def on_tracked_poses(self, msg: PersonPose2DArray) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        for pose in msg.poses:
            if pose.track_id:
                self.history[pose.track_id].append(PoseSnapshot(now_sec, pose))
        smaller = self._find_role(msg.poses, 'smaller_candidate')
        larger = self._find_role(msg.poses, 'larger_candidate')
        if smaller is None or larger is None:
            return
        values = self._calculate_features(smaller, larger)
        out = InteractionFeatures()
        out.header = msg.header
        out.smaller_track_id = smaller.track_id
        out.larger_track_id = larger.track_id
        out.torso_distance_norm = values['torso_distance_norm']
        out.wrap_score = values['wrap_score']
        out.lift_score = values['lift_score']
        out.feet_off_ground_score = values['feet_off_ground_score']
        out.limb_speed_score = values['limb_speed_score']
        out.limb_accel_score = values['limb_accel_score']
        out.co_motion_score = values['co_motion_score']
        out.suspicion_score = values['suspicion_score']
        out.state = values['state']
        self.publisher.publish(out)

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
        score = calculate_suspicion_score(FeatureScores(contact, wrap, lift, feet, limb_speed, limb_accel, comotion))
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

    def _lift_score(self, child: PersonPose2D) -> float:
        h = list(self.history.get(child.track_id, []))
        if len(h) < 3:
            return 0.0
        first, last = h[0], h[-1]
        dt = max(last.stamp_sec - first.stamp_sec, 1e-3)
        upward_velocity_norm = (self._torso_center(first.pose).y - self._torso_center(last.pose).y) / max(child.torso_length_px, 1.0) / dt
        return ramp_score(upward_velocity_norm, self.lift_velocity_threshold, self.lift_velocity_threshold * 1.8)

    def _feet_off_ground_proxy(self, child: PersonPose2D) -> float:
        if len(child.keypoints_xy) <= RIGHT_ANKLE:
            return 0.0
        ankle_y = max(self._point(child, LEFT_ANKLE).y, self._point(child, RIGHT_ANKLE).y)
        bbox_bottom = float(child.bbox.y_offset + child.bbox.height)
        gap_norm = (bbox_bottom - ankle_y) / max(child.bbox.height, 1.0)
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
                joint_speeds.append(distance(p1, p2) / max(child.torso_length_px, 1.0) / max(t2 - t1, 1e-3))
            if joint_speeds:
                speeds.append(safe_mean(joint_speeds))
            if len(joint_speeds) >= 2:
                accels.append(abs(joint_speeds[-1] - joint_speeds[0]) / max(samples[-1][0] - samples[0][0], 1e-3))
        return ramp_score(safe_mean(speeds), self.limb_speed_low, self.limb_speed_high), ramp_score(safe_mean(accels), self.limb_accel_low, self.limb_accel_high)

    def _co_motion_score(self, child: PersonPose2D, adult: PersonPose2D) -> float:
        ch = list(self.history.get(child.track_id, []))
        ah = list(self.history.get(adult.track_id, []))
        if len(ch) < 2 or len(ah) < 2:
            return 0.0
        child_dx = self._torso_center(ch[-1].pose).x - self._torso_center(ch[0].pose).x
        adult_dx = self._torso_center(ah[-1].pose).x - self._torso_center(ah[0].pose).x
        same_direction = 1.0 if child_dx * adult_dx > 0 else 0.0
        movement = min(abs(child_dx), abs(adult_dx)) / max(child.torso_length_px, 1.0)
        return same_direction * ramp_score(movement, self.co_motion_threshold, self.co_motion_threshold * 2.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InteractionAnalyzerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
