import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    # 1. Velodyne driver + pointcloud nodes.
    # SetRemap silences the built-in velodyne_laserscan on /scan so it doesn't interfere.
    velodyne_launch = GroupAction([
        SetRemap(src='/scan', dst='/velodyne_scan_raw'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('velodyne'), 'launch', 'velodyne-all-nodes-VLP16-launch.py')
            )
        )
    ])

    # 2. ICP Odometry — computes real odom -> base_link from consecutive point clouds.
    # Replaces the fake static transform used in master_launch.py, giving RTAB-Map
    # a real motion prior and dramatically reducing drift.
    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'subscribe_scan_cloud': True,
            'approx_sync': True,
            'wait_for_transform': 0.2,
            # ICP tuning for slow indoor cart with VLP16
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '30',            # more iterations = more accurate in corridors
            'Icp/VoxelSize': '0.1',           # downsample to 10cm voxels for speed
            'Icp/MaxCorrespondenceDistance': '0.2',  # slightly larger to handle corridor drift
            'Icp/MaxTranslation': '1.0',       # reject jumps > 1m between scans
            'Icp/CorrespondenceRatio': '0.01', # accept sparse correspondences in corridors
            'Odom/ScanKeyFrameThr': '0.9',     # add keyframe when 90% of points change
            'Odom/Strategy': '0',              # frame-to-map odometry
            'scan_normal_k': '20',             # use 20 neighbors for normal estimation (default 5)
            'scan_range_min': 0.3,             # match VLP16 minimum range
            'scan_range_max': 25.0,            # match indoor range limit
        }],
        remappings=[
            ('scan_cloud', '/velodyne_points'),
        ]
    )

    # 3. RTAB-Map — 3D mapping with loop closure.
    # Subscribes to point clouds and odometry, builds a 3D map and a 2D occupancy grid.
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'database_path': '~/.ros/storm_slam_map.db',  # explicit save location
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan_cloud': True,
            'subscribe_odom_info': False,   # only sync odom + scan_cloud (2-way), not odom_info
            'approx_sync': True,
            'queue_size': 10,
            'wait_for_transform': 0.2,
            # Loop closure and proximity detection
            'RGBD/NeighborLinkRefining': 'true',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '10', # search 10 neighbors for proximity detection
            'Icp/CorrespondenceRatio': '0.01',      # accept weak ICP matches for loop closure
            # Use ICP for scan-to-map registration
            'Reg/Strategy': '1',
            'Icp/VoxelSize': '0.1',
            'Icp/MaxCorrespondenceDistance': '0.1',
            # Range limits matching VLP16 / indoor environment
            'Grid/RangeMin': '0.3',
            'Grid/RangeMax': '25.0',
            'Grid/CellSize': '0.05',           # 5cm map resolution
            'cloud_min_height': '-0.1',        # filter points more than 10cm below ground
            'cloud_max_height': '3.0',         # filter points above 3m
            # Memory management for large building mapping
            'Mem/STMSize': '30',               # keep 30 nodes in short-term memory (default 10)
            'Mem/NotLinkedNodesKept': 'false', # discard unlinked nodes to save RAM
            'cloud_voxel_size': '0.05',        # 5cm voxels in saved cloud map (keeps file size down)
            # Map update rate
            'Rtabmap/DetectionRate': '1.0',    # loop closure check every 1s
            'Vis/MaxFeatures': '0',            # disable visual features (no camera)
            'Optimizer/GravitySigma': '0',     # disable IMU gravity constraint (no IMU)
        }],
        remappings=[
            ('scan_cloud', '/velodyne_points'),
        ]
    )

    # 4. Static Transform: base_link -> velodyne
    # Fixed sensor mounting offset. RTAB-Map and ICP odometry use this to
    # transform point clouds into the base_link frame.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'velodyne']
    )

    return LaunchDescription([
        velodyne_launch,
        static_tf,
        icp_odometry,
        rtabmap,
    ])
