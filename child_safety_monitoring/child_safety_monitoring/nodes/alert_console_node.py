from __future__ import annotations

import rclpy
from rclpy.node import Node

from child_safety_msgs.msg import InteractionFeatures, SuspicionEvent


class AlertConsoleNode(Node):
    """
    Prints clean state-change messages for classroom demo.
    Instead of repeating every alert, it only prints when the level changes.
    """

    def __init__(self) -> None:
        super().__init__('alert_console_node')

        self.declare_parameter('normal_threshold', 0.55)

        self.normal_threshold = float(self.get_parameter('normal_threshold').value)
        self.last_level = 'unknown'

        self.feature_subscription = self.create_subscription(
            InteractionFeatures,
            '/interaction/features',
            self.on_interaction_features,
            10,
        )

        self.alert_subscription = self.create_subscription(
            SuspicionEvent,
            '/suspicion_event',
            self.on_suspicion_event,
            10,
        )

        self.get_logger().info('Alert console started. Waiting for scenario state changes...')

    def on_interaction_features(self, msg: InteractionFeatures) -> None:
        if msg.suspicion_score < self.normal_threshold and self.last_level != 'normal':
            self.last_level = 'normal'
            self.get_logger().info(
                f'[NORMAL] score={msg.suspicion_score:.2f} | No suspicious interaction pattern detected'
            )

    def on_suspicion_event(self, msg: SuspicionEvent) -> None:
        level = msg.level.lower()
        score = msg.suspicion_score
        explanation = msg.explanation

        if level == self.last_level:
            return

        self.last_level = level

        if level == 'high':
            self.get_logger().error(
                f'[HIGH ALERT] score={score:.2f} | Suspicious child-lifting pattern detected | {explanation}'
            )
        elif level == 'warning':
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
