from __future__ import annotations

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
