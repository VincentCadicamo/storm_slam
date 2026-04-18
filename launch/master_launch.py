import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    # 1. Velodyne driver + pointcloud nodes.
    # Wrapped in a GroupAction that remaps /scan -> /velodyne_scan_raw so the built-in
    # velodyne_laserscan node doesn't publish on /scan and conflict with our
    # pointcloud_to_laserscan output below.
    velodyne_launch = GroupAction([
        SetRemap(src='/scan', dst='/velodyne_scan_raw'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('velodyne'), 'launch', 'velodyne-all-nodes-VLP16-launch.py')
            )
        )
    ])

    pc_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'velodyne', # Leave this as your lidar frame
            'transform_tolerance': 0.1,
            'min_height': -0.1,  # Adjust these to "slice" the 3D data
            'max_height': 0.1,   # only at the height of your LIDAR
            'angle_min': -3.1415,
            'angle_max': 3.1415,
            'angle_increment': 0.017453,  # 1 degree in radians (pi/180); gives exactly 360 readings
            'range_min': 0.3,
            'range_max': 25.0,
            'use_inf': True
        }],
        remappings=[
            ('/cloud_in', '/velodyne_points'), # Output from Velodyne driver
            ('/scan', '/scan')                 # Input to Slam Toolbox
        ]
    )

    # 2. Path to Slam Toolbox (Standard async launch)
    # We pass YOUR custom yaml file as an argument here
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': os.path.join(
                get_package_share_directory('storm_slam'), 'config', 'slam_toolbox_params.yaml'
            )
        }.items()
    )

    # 3. Static Transform: base_link -> velodyne
    # Publishes the fixed offset between robot base and the LIDAR sensor.
    # SLAM Toolbox owns the odom -> base_link transform; this covers base_link -> velodyne.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'velodyne']
    )

    # 4. Fake odometry: odom -> base_link (identity, handheld / no-encoder setup)
    # Without wheel encoders or an IMU there is no real odometry source. Publishing a static
    # identity transform here satisfies SLAM Toolbox's TF lookup on startup. All actual motion
    # is absorbed into the map -> odom transform that SLAM Toolbox publishes via scan matching.
    fake_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    return LaunchDescription([
        velodyne_launch,
        pc_to_scan_node,
        slam_toolbox_launch,
        static_tf,
        fake_odom_tf,
    ])