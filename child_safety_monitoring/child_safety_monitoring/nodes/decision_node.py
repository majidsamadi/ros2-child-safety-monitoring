from __future__ import annotations

import rclpy
from rclpy.node import Node
from child_safety_msgs.msg import InteractionFeatures, SuspicionEvent


class DecisionNode(Node):
    def __init__(self) -> None:
        super().__init__('decision_node')
        self.declare_parameter('warning_threshold', 0.55)
        self.declare_parameter('high_threshold', 0.75)
        self.declare_parameter('warning_persistence_seconds', 0.30)
        self.declare_parameter('high_persistence_seconds', 0.50)
        self.warning_threshold = float(self.get_parameter('warning_threshold').value)
        self.high_threshold = float(self.get_parameter('high_threshold').value)
        self.warning_persistence = float(self.get_parameter('warning_persistence_seconds').value)
        self.high_persistence = float(self.get_parameter('high_persistence_seconds').value)
        self.over_warning_since = None
        self.over_high_since = None
        self.subscription = self.create_subscription(InteractionFeatures, '/interaction/features', self.on_features, 10)
        self.publisher = self.create_publisher(SuspicionEvent, '/suspicion_event', 10)

    def on_features(self, features: InteractionFeatures) -> None:
        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1e9
        score = float(features.suspicion_score)
        self.over_warning_since = self.over_warning_since or now_sec if score >= self.warning_threshold else None
        self.over_high_since = self.over_high_since or now_sec if score >= self.high_threshold else None
        level = None
        if self.over_high_since is not None and (now_sec - self.over_high_since) >= self.high_persistence:
            level = 'high'
        elif self.over_warning_since is not None and (now_sec - self.over_warning_since) >= self.warning_persistence:
            level = 'warning'
        if level is None:
            return
        event = SuspicionEvent()
        event.header = features.header
        event.event_start = now.to_msg()
        event.current_time = now.to_msg()
        event.level = level
        event.suspicion_score = score
        event.explanation = (
            f'{level.upper()} score={score:.2f}; '
            f'contact_dist_norm={features.torso_distance_norm:.2f}; '
            f'wrap={features.wrap_score:.2f}; lift={features.lift_score:.2f}; '
            f'feet={features.feet_off_ground_score:.2f}; '
            f'limb_speed={features.limb_speed_score:.2f}; '
            f'limb_accel={features.limb_accel_score:.2f}; '
            f'co_motion={features.co_motion_score:.2f}'
        )
        self.publisher.publish(event)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
