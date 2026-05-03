from __future__ import annotations

import time
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class VideoSourceNode(Node):
    def __init__(self) -> None:
        super().__init__('video_source_node')
        self.declare_parameter('source_type', 'webcam')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('video_path', '')
        self.declare_parameter('fps', 20.0)
        self.declare_parameter('loop_video', True)
        self.source_type = self.get_parameter('source_type').value
        source = int(self.get_parameter('camera_index').value) if self.source_type == 'webcam' else str(self.get_parameter('video_path').value)
        self.fps = float(self.get_parameter('fps').value)
        self.loop_video = bool(self.get_parameter('loop_video').value)
        self.cap = cv2.VideoCapture(source)
        if not self.cap.isOpened():
            raise RuntimeError(f'Could not open video source: {source}')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(1.0 / max(self.fps, 1.0), self.publish_frame)
        self.get_logger().info(f'Video source started: {source}')

    def publish_frame(self) -> None:
        ok, frame = self.cap.read()
        if not ok:
            if self.source_type == 'video' and self.loop_video:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.1)
                return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VideoSourceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cap.release()
        except Exception:
            pass
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
