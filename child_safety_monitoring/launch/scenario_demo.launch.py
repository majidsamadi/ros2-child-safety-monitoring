from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    decision_node = Node(
        package='child_safety_monitoring',
        executable='decision_node',
        name='decision_node',
        output='screen',
    )

    scenario_simulator_node = Node(
        package='child_safety_monitoring',
        executable='scenario_simulator_node',
        name='scenario_simulator_node',
        output='screen',
        parameters=[
            {
                'scenario': 'all',
                'rate_hz': 5.0,
                'scenario_duration_seconds': 4.0,
                'loop': True,
            }
        ],
    )

    return LaunchDescription([
        decision_node,
        scenario_simulator_node,
    ])
