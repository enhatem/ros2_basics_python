from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='exercise42_pkg',
            executable='move_stop_service',
            output='screen'),
    ])
