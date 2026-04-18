import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to your custom yaml file
    config_file = os.path.join(get_package_share_directory('storm_slam'), 'config', 'slam_toolbox_params.yaml')

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[config_file] # Passing your custom file here
        )
    ])