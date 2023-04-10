from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_follower',
            executable='wall_finding',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']),

        Node(
            package='wall_follower',
            executable='wall_following_v2',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'])
    ])
