from __future__ import annotations

from typing import Optional
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from child_safety_msgs.msg import PersonPose2DArray, SuspicionEvent
from child_safety_monitoring.core.keypoints import SKELETON_EDGES


class VisualizationNode(Node):
    def __init__(self) -> None:
        super().__init__('visualization_node')
        self.bridge = CvBridge()
        self.latest_poses: Optional[PersonPose2DArray] = None
        self.latest_event: Optional[SuspicionEvent] = None
        self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)
        self.create_subscription(PersonPose2DArray, '/poses/tracked', self.on_poses, 10)
        self.create_subscription(SuspicionEvent, '/suspicion_event', self.on_event, 10)
        self.publisher = self.create_publisher(Image, '/annotated_image', 10)

    def on_poses(self, msg: PersonPose2DArray) -> None:
        self.latest_poses = msg

    def on_event(self, msg: SuspicionEvent) -> None:
        self.latest_event = msg
        self.get_logger().warning(f'{msg.level.upper()}: {msg.explanation}')

    def on_image(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.latest_poses is not None:
            for pose in self.latest_poses.poses:
                self._draw_pose(frame, pose)
        if self.latest_event is not None:
            label = f'{self.latest_event.level.upper()} {self.latest_event.suspicion_score:.2f}'
            cv2.putText(frame, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
        else:
            cv2.putText(frame, 'OBSERVING', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out.header = msg.header
        self.publisher.publish(out)

    @staticmethod
    def _draw_pose(frame, pose) -> None:
        x, y, w, h = pose.bbox.x_offset, pose.bbox.y_offset, pose.bbox.width, pose.bbox.height
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 0), 2)
        label = f'{pose.track_id} {pose.size_role}'
        cv2.putText(frame, label, (x, max(0, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        pts = [(int(p.x), int(p.y)) for p in pose.keypoints_xy]
        for a, b in SKELETON_EDGES:
            if a < len(pts) and b < len(pts):
                cv2.line(frame, pts[a], pts[b], (0, 255, 255), 2)
        for p in pts:
            cv2.circle(frame, p, 3, (0, 255, 0), -1)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
