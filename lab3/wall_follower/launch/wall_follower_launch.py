from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_follower',
            executable='wall_follower',
            name='wall_follower_node',
            output='screen'
        ),
        Node(
            package='safety_controller',
            executable='safety_controller',
            name='safety_controller_node',
            output='screen'
        )
    ])
