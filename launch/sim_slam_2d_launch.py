import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('storm_slam')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # pointcloud_to_laserscan: converts /velodyne_points → /scan for SLAM Toolbox
    # Identical params to master_launch.py; /velodyne_points now arrives via ros_gz_bridge
    pc_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'velodyne',
            'transform_tolerance': 0.1,
            'min_height': -0.1,
            'max_height': 0.1,
            'angle_min': -3.1415,
            'angle_max': 3.1415,
            'angle_increment': 0.017453,
            'range_min': 0.3,
            'range_max': 25.0,
            'use_inf': True,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/cloud_in', '/velodyne_points'),
            ('/scan', '/scan')
        ]
    )

    # SLAM Toolbox async mapping with sim-tuned params (use_sim_time: true,
    # non-zero travel thresholds since real wheel odom is available)
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': os.path.join(
                pkg, 'config', 'slam_toolbox_sim_params.yaml'
            ),
            'use_sim_time': use_sim_time
        }.items()
    )

    odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        odom_tf,
        pc_to_scan_node,
        slam_toolbox_launch,
    ])
