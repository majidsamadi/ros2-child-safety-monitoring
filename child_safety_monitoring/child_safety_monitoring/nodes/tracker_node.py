from __future__ import annotations

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
