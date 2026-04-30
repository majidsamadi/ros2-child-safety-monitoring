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
