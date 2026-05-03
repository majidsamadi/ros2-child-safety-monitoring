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
        self.feet_requires_lift_score = float(self.get_parameter('feet_requires_lift_score').value)
        self.feet_without_lift_cap = float(self.get_parameter('feet_without_lift_cap').value)
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
        raw_feet = self._feet_off_ground_proxy(child)

        # Feet/ankle keypoints can be unreliable with laptop cameras when legs are
        # partly out of frame. Do not let feet_off_ground dominate unless the
        # child candidate is also moving upward.
        if lift >= self.feet_requires_lift_score:
            feet = raw_feet
        else:
            feet = min(raw_feet, self.feet_without_lift_cap)

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
    def _is_visible(pose: PersonPose2D, index: int) -> bool:
        return (
            index < len(pose.keypoints_xy)
            and index < len(pose.visible)
            and bool(pose.visible[index])
        )

    def _point(self, pose: PersonPose2D, index: int) -> Point2:
        if not self._is_visible(pose, index):
            return bbox_center(pose.bbox.x_offset, pose.bbox.y_offset, pose.bbox.width, pose.bbox.height)
        p: Point32 = pose.keypoints_xy[index]
        return Point2(p.x, p.y)

    def _torso_center(self, pose: PersonPose2D) -> Point2:
        pts = [
            self._point(pose, i)
            for i in [LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_HIP, RIGHT_HIP]
            if self._is_visible(pose, i)
        ]
        if not pts:
            return bbox_center(pose.bbox.x_offset, pose.bbox.y_offset, pose.bbox.width, pose.bbox.height)
        return Point2(safe_mean([p.x for p in pts]), safe_mean([p.y for p in pts]))

    def _ankle_y(self, pose: PersonPose2D) -> Optional[float]:
        ankle_values = [
            self._point(pose, idx).y
            for idx in [LEFT_ANKLE, RIGHT_ANKLE]
            if self._is_visible(pose, idx)
        ]
        if not ankle_values:
            return None
        return max(ankle_values)

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
            samples = [
                (s.stamp_sec, self._point(s.pose, idx))
                for s in h
                if self._is_visible(s.pose, idx)
            ]
            if len(samples) < 2:
                continue
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
