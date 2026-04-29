from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('child_safety_monitoring')
    config = os.path.join(pkg_share, 'config', 'detection_params.yaml')
    video_path = LaunchConfiguration('video_path')

    return LaunchDescription([
        DeclareLaunchArgument('video_path', default_value='', description='Absolute path to a video file'),
        Node(package='child_safety_monitoring', executable='video_source_node', name='video_source_node', parameters=[config, {'source_type': 'video', 'video_path': video_path}], output='screen'),
        Node(package='child_safety_monitoring', executable='pose_estimator_node', name='pose_estimator_node', parameters=[config], output='screen'),
        Node(package='child_safety_monitoring', executable='tracker_node', name='tracker_node', output='screen'),
        Node(package='child_safety_monitoring', executable='interaction_analyzer_node', name='interaction_analyzer_node', parameters=[config], output='screen'),
        Node(package='child_safety_monitoring', executable='decision_node', name='decision_node', parameters=[config], output='screen'),
        Node(package='child_safety_monitoring', executable='visualization_node', name='visualization_node', output='screen'),
    ])
