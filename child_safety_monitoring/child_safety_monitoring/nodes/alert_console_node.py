from __future__ import annotations

import rclpy
from rclpy.node import Node

from child_safety_msgs.msg import SuspicionEvent


class AlertConsoleNode(Node):
    """
    Prints clean alert messages for classroom demo.
    This is easier to understand than reading full YAML from ros2 topic echo.
    """

    def __init__(self) -> None:
        super().__init__('alert_console_node')

        self.subscription = self.create_subscription(
            SuspicionEvent,
            '/suspicion_event',
            self.on_suspicion_event,
            10,
        )

        self.get_logger().info('Alert console started. Waiting for suspicion events...')

    def on_suspicion_event(self, msg: SuspicionEvent) -> None:
        level = msg.level.upper()
        score = msg.suspicion_score
        explanation = msg.explanation

        if level == 'HIGH':
            self.get_logger().error(
                f'[HIGH ALERT] score={score:.2f} | Suspicious child-lifting pattern detected | {explanation}'
            )
        elif level == 'WARNING':
            self.get_logger().warn(
                f'[WARNING] score={score:.2f} | Suspicious interaction pattern detected | {explanation}'
            )
        else:
            self.get_logger().info(
                f'[INFO] score={score:.2f} | {explanation}'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AlertConsoleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
