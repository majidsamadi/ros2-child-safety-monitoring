from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    stream_url = LaunchConfiguration('stream_url')

    return LaunchDescription([
        DeclareLaunchArgument(
            'stream_url',
            default_value='http://host.docker.internal:8090/video',
            description='Laptop webcam stream URL from host_webcam_streamer.py',
        ),

        Node(
            package='child_safety_monitoring',
            executable='cctv_stream_node',
            name='cctv_stream_node',
            output='screen',
            parameters=[{
                'stream_url': stream_url,
                'image_topic': '/camera/image_raw',
                'camera_frame_id': 'laptop_camera',
                'publish_rate_hz': 15.0,
            }],
        ),

        Node(
            package='child_safety_monitoring',
            executable='pose_estimator_node',
            name='pose_estimator_node',
            output='screen',
            parameters=[{
                'image_topic': '/camera/image_raw',
                'raw_pose_topic': '/poses/raw',
                'annotated_image_topic': '/camera/pose_overlay',
                'publish_annotated': True,
                'model_path': 'yolo11n-pose.pt',
                'confidence_threshold': 0.35,
                'device': 'cpu',
                'frame_skip': 0,
                'keypoint_confidence_threshold': 0.25,
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
                'min_people_required': 2,
            }],
        ),

        Node(
            package='child_safety_monitoring',
            executable='interaction_analyzer_node',
            name='interaction_analyzer_node',
            output='screen',
            parameters=[{
                'tracked_pose_topic': '/poses/tracked',
                'features_topic': '/interaction/features',
            }],
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
