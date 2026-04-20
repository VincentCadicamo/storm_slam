import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # ICP Odometry — computes odom->base_link from consecutive point clouds.
    # /velodyne_points now arrives via ros_gz_bridge instead of the real hardware driver.
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
            'use_sim_time': use_sim_time,
            # ICP tuning (same as real-hardware launch)
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '30',
            'Icp/VoxelSize': '0.1',
            'Icp/MaxCorrespondenceDistance': '0.2',
            'Icp/MaxTranslation': '1.0',
            'Icp/CorrespondenceRatio': '0.01',
            'Odom/ScanKeyFrameThr': '0.9',
            'Odom/Strategy': '0',
            'scan_normal_k': 20,
            'scan_range_min': 0.3,
            'scan_range_max': 25.0,
        }],
        remappings=[
            ('scan_cloud', '/velodyne_points'),
        ]
    )

    # RTAB-Map 3D mapping with loop closure
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'database_path': '~/.ros/storm_slam_map.db',
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan_cloud': True,
            'subscribe_odom_info': False,
            'approx_sync': True,
            'sync_queue_size': 10,
            'wait_for_transform': 0.2,
            'use_sim_time': use_sim_time,
            # Loop closure and proximity detection
            'RGBD/NeighborLinkRefining': 'true',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '10',
            'Icp/CorrespondenceRatio': '0.01',
            'Reg/Strategy': '1',
            'Icp/VoxelSize': '0.1',
            'Icp/MaxCorrespondenceDistance': '0.1',
            # Range and map resolution
            'Grid/RangeMin': '0.3',
            'Grid/RangeMax': '25.0',
            'Grid/CellSize': '0.05',
            'cloud_min_height': '-0.1',
            'cloud_max_height': '3.0',
            # Memory management
            'Mem/STMSize': '30',
            'Mem/NotLinkedNodesKept': 'false',
            'cloud_voxel_size': '0.05',
            'Rtabmap/DetectionRate': '1.0',
            'Vis/MaxFeatures': '0',
            'Optimizer/GravitySigma': '0',
        }],
        remappings=[
            ('scan_cloud', '/velodyne_points'),
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        icp_odometry,
        rtabmap,
    ])
