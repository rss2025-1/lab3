from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the params file
    config_dir = os.path.join(get_package_share_directory('wall_follower'), 'config')
    params_file = os.path.join(config_dir, 'wall_follower_params.yaml')

    return LaunchDescription([
        Node(
            package='wall_follower',
            executable='wall_follower',
            name='wall_follower',
            parameters=[params_file]
        ),
        Node(
            package='safety_controller',
            executable='safety_controller',
            name='safety_controller',
            parameters=[params_file]
        )
    ])
