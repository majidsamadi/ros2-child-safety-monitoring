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
                'log_each_message': False,
            }
        ],
    )

    alert_console_node = Node(
        package='child_safety_monitoring',
        executable='alert_console_node',
        name='alert_console_node',
        output='screen',
    )

    return LaunchDescription([
        decision_node,
        scenario_simulator_node,
        alert_console_node,
    ])
