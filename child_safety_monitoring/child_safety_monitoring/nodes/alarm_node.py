from __future__ import annotations

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
