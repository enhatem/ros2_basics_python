from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='exercise31_pkg',
            executable='pub_sub_node',
            output='screen'),
    ])
