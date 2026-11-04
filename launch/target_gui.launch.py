import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('target_gui'),
        'config',
        'target_gui.yaml',
    )
    return LaunchDescription([
        Node(
            package='target_gui',
            executable='target_gui',
            name='target_gui',
            output='screen',
            parameters=[config],
        ),
    ])
