from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    stream_url = LaunchConfiguration('stream_url')
    model_path = LaunchConfiguration('model_path')
    device = LaunchConfiguration('device')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'stream_url',
            default_value='',
            description='Camera stream URL. For laptop webcam streamer use: http://host.docker.internal:8090/video',
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value='yolo11n-pose.pt',
            description='YOLO pose model path/name.',
        ),
        DeclareLaunchArgument(
            'device',
            default_value='cpu',
            description='YOLO device, e.g. cpu, cuda, cuda:0.',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='12.0',
            description='Camera frame publish rate.',
        ),

        Node(
            package='child_safety_monitoring',
            executable='cctv_stream_node',
            name='cctv_stream_node',
            output='screen',
            parameters=[{
                'stream_url': stream_url,
                'publish_rate_hz': publish_rate_hz,
                'image_topic': '/camera/image_raw',
                'resize_width': 960,
                'resize_height': 540,
                'buffer_size': 1,
            }],
        ),
        Node(
            package='child_safety_monitoring',
            executable='pose_estimator_node',
            name='pose_estimator_node',
            output='screen',
            parameters=[{
                'model_path': model_path,
                'device': device,
                'confidence_threshold': 0.35,
                'image_topic': '/camera/image_raw',
                'raw_pose_topic': '/poses/raw',
                'publish_annotated': True,
                'annotated_image_topic': '/camera/pose_overlay',
                'frame_skip': 0,
            }],
        ),
        Node(
            package='child_safety_monitoring',
            executable='tracker_node',
            name='tracker_node',
            output='screen',
            parameters=[{
                'raw_pose_topic': '/poses/raw',
                'tracked_pose_topic': '/poses/tracked',
                'max_distance_px': 140.0,
                'max_missed_frames': 10,
            }],
        ),
        Node(
            package='child_safety_monitoring',
            executable='interaction_analyzer_node',
            name='interaction_analyzer_node',
            output='screen',
        ),
        Node(
            package='child_safety_monitoring',
            executable='decision_node',
            name='decision_node',
            output='screen',
        ),
        Node(
            package='child_safety_monitoring',
            executable='alert_console_node',
            name='alert_console_node',
            output='screen',
        ),
        Node(
            package='child_safety_monitoring',
            executable='alarm_node',
            name='alarm_node',
            output='screen',
        ),
    ])
