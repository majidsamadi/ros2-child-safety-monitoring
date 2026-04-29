from __future__ import annotations

import rclpy
from rclpy.node import Node
from child_safety_msgs.msg import PersonPose2DArray
from child_safety_monitoring.core.centroid_tracker import CentroidTracker
from child_safety_monitoring.core.geometry import bbox_center


class TrackerNode(Node):
    def __init__(self) -> None:
        super().__init__('tracker_node')
        self.tracker = CentroidTracker()
        self.subscription = self.create_subscription(PersonPose2DArray, '/poses/raw', self.on_raw_poses, 10)
        self.publisher = self.create_publisher(PersonPose2DArray, '/poses/tracked', 10)

    def on_raw_poses(self, msg: PersonPose2DArray) -> None:
        detections = []
        for pose in msg.poses:
            center = bbox_center(pose.bbox.x_offset, pose.bbox.y_offset, pose.bbox.width, pose.bbox.height)
            detections.append((center, float(pose.bbox.height)))
        assignments = self.tracker.update(detections)
        for det_idx, track_id in assignments.items():
            msg.poses[det_idx].track_id = track_id
        if len(msg.poses) >= 2:
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
    finally:
        node.destroy_node()
        rclpy.shutdown()
