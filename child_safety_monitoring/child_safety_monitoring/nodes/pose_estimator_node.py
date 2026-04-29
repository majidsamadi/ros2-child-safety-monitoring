from __future__ import annotations

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point32
from rclpy.node import Node
from sensor_msgs.msg import Image, RegionOfInterest
from child_safety_msgs.msg import PersonPose2D, PersonPose2DArray
from child_safety_monitoring.core.keypoints import LEFT_HIP, LEFT_SHOULDER, RIGHT_HIP, RIGHT_SHOULDER


class PoseEstimatorNode(Node):
    def __init__(self) -> None:
        super().__init__('pose_estimator_node')
        self.declare_parameter('model_path', 'yolo11n-pose.pt')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('raw_pose_topic', '/poses/raw')
        self.model_path = str(self.get_parameter('model_path').value)
        self.conf = float(self.get_parameter('confidence_threshold').value)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(PersonPose2DArray, str(self.get_parameter('raw_pose_topic').value), 10)
        self.subscription = self.create_subscription(Image, str(self.get_parameter('image_topic').value), self.on_image, 10)
        try:
            from ultralytics import YOLO
            self.model = YOLO(self.model_path)
            self.get_logger().info(f'Loaded YOLO pose model: {self.model_path}')
        except Exception as exc:
            self.model = None
            self.get_logger().error(f'Failed to load YOLO pose model: {exc}')

    def on_image(self, image_msg: Image) -> None:
        if self.model is None:
            return
        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        result = self.model.predict(frame, conf=self.conf, verbose=False)[0]
        out = PersonPose2DArray()
        out.header = image_msg.header
        boxes = result.boxes.xyxy.cpu().numpy() if result.boxes is not None else np.empty((0, 4))
        kxy = result.keypoints.xy.cpu().numpy() if result.keypoints is not None else np.empty((0, 17, 2))
        kcf = result.keypoints.conf.cpu().numpy() if result.keypoints is not None else np.empty((0, 17))
        for i, box in enumerate(boxes):
            if i >= len(kxy):
                continue
            x1, y1, x2, y2 = box
            person = PersonPose2D()
            person.header = image_msg.header
            person.track_id = ''
            person.size_role = 'unknown'
            person.size_confidence = 0.0
            person.bbox = RegionOfInterest(x_offset=max(int(x1), 0), y_offset=max(int(y1), 0), width=max(int(x2 - x1), 0), height=max(int(y2 - y1), 0))
            person.keypoints_xy = [Point32(x=float(x), y=float(y), z=0.0) for x, y in kxy[i]]
            person.keypoint_confidence = [float(c) for c in kcf[i]]
            person.visible = [float(c) > 0.25 for c in kcf[i]]
            person.center_depth_m = 0.0
            person.torso_length_px = self._torso_len(person)
            out.poses.append(person)
        self.publisher.publish(out)

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
    finally:
        node.destroy_node()
        rclpy.shutdown()
