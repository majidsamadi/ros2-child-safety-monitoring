from __future__ import annotations

import time
from typing import Dict

import rclpy
from rclpy.node import Node

from child_safety_msgs.msg import InteractionFeatures


class ScenarioSimulatorNode(Node):
    """
    Publishes fake InteractionFeatures messages so we can test the decision pipeline
    before connecting the real camera and pose-estimation nodes.
    """

    def __init__(self) -> None:
        super().__init__('scenario_simulator_node')

        self.declare_parameter('scenario', 'all')
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('scenario_duration_seconds', 4.0)
        self.declare_parameter('loop', True)

        self.scenario = str(self.get_parameter('scenario').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.scenario_duration = float(self.get_parameter('scenario_duration_seconds').value)
        self.loop = bool(self.get_parameter('loop').value)

        self.publisher = self.create_publisher(
            InteractionFeatures,
            '/interaction/features',
            10,
        )

        self.scenarios = self._build_scenarios()
        self.scenario_order = self._resolve_scenario_order()

        self.current_index = 0
        self.current_scenario_started_at = time.time()

        timer_period = 1.0 / max(self.rate_hz, 0.1)
        self.timer = self.create_timer(timer_period, self.on_timer)

        self.get_logger().info(
            f'Scenario simulator started. scenario={self.scenario}, '
            f'rate_hz={self.rate_hz}, duration={self.scenario_duration}, loop={self.loop}'
        )

    def _build_scenarios(self) -> Dict[str, Dict[str, float | str]]:
        return {
            'normal': {
                'smaller_track_id': 'child_candidate_1',
                'larger_track_id': 'adult_candidate_1',
                'torso_distance_norm': 2.50,
                'wrap_score': 0.05,
                'lift_score': 0.00,
                'feet_off_ground_score': 0.00,
                'limb_speed_score': 0.10,
                'limb_accel_score': 0.10,
                'co_motion_score': 0.05,
                'suspicion_score': 0.10,
                'state': 'normal',
            },
            'warning': {
                'smaller_track_id': 'child_candidate_1',
                'larger_track_id': 'adult_candidate_1',
                'torso_distance_norm': 0.85,
                'wrap_score': 0.55,
                'lift_score': 0.35,
                'feet_off_ground_score': 0.20,
                'limb_speed_score': 0.45,
                'limb_accel_score': 0.40,
                'co_motion_score': 0.35,
                'suspicion_score': 0.62,
                'state': 'warning',
            },
            'high': {
                'smaller_track_id': 'child_candidate_1',
                'larger_track_id': 'adult_candidate_1',
                'torso_distance_norm': 0.30,
                'wrap_score': 0.90,
                'lift_score': 0.95,
                'feet_off_ground_score': 0.90,
                'limb_speed_score': 0.95,
                'limb_accel_score': 0.90,
                'co_motion_score': 0.80,
                'suspicion_score': 0.92,
                'state': 'high_alert',
            },
        }

    def _resolve_scenario_order(self) -> list[str]:
        if self.scenario == 'all':
            return ['normal', 'warning', 'high']

        if self.scenario not in self.scenarios:
            valid = ', '.join(['all'] + list(self.scenarios.keys()))
            raise ValueError(f'Invalid scenario "{self.scenario}". Valid options: {valid}')

        return [self.scenario]

    def on_timer(self) -> None:
        now = time.time()

        if now - self.current_scenario_started_at >= self.scenario_duration:
            self.current_index += 1

            if self.current_index >= len(self.scenario_order):
                if not self.loop:
                    self.get_logger().info('Scenario simulation completed.')
                    rclpy.shutdown()
                    return
                self.current_index = 0

            self.current_scenario_started_at = now

        scenario_name = self.scenario_order[self.current_index]
        values = self.scenarios[scenario_name]

        msg = InteractionFeatures()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'simulated_camera'

        msg.smaller_track_id = str(values['smaller_track_id'])
        msg.larger_track_id = str(values['larger_track_id'])
        msg.torso_distance_norm = float(values['torso_distance_norm'])
        msg.wrap_score = float(values['wrap_score'])
        msg.lift_score = float(values['lift_score'])
        msg.feet_off_ground_score = float(values['feet_off_ground_score'])
        msg.limb_speed_score = float(values['limb_speed_score'])
        msg.limb_accel_score = float(values['limb_accel_score'])
        msg.co_motion_score = float(values['co_motion_score'])
        msg.suspicion_score = float(values['suspicion_score'])
        msg.state = str(values['state'])

        self.publisher.publish(msg)

        self.get_logger().info(
            f'Published scenario={scenario_name}, score={msg.suspicion_score:.2f}, state={msg.state}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScenarioSimulatorNode()

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
