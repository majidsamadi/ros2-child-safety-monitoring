from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'child_safety_monitoring'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student Team',
    maintainer_email='student@example.com',
    description='ROS 2 suspicious child-lifting pattern detection prototype.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_source_node = child_safety_monitoring.nodes.video_source_node:main',
            'pose_estimator_node = child_safety_monitoring.nodes.pose_estimator_node:main',
            'tracker_node = child_safety_monitoring.nodes.tracker_node:main',
            'interaction_analyzer_node = child_safety_monitoring.nodes.interaction_analyzer_node:main',
            'decision_node = child_safety_monitoring.nodes.decision_node:main',
            'visualization_node = child_safety_monitoring.nodes.visualization_node:main',
            'scenario_simulator_node = child_safety_monitoring.nodes.scenario_simulator_node:main',
            'alert_console_node = child_safety_monitoring.nodes.alert_console_node:main',
        ],
    },
)
