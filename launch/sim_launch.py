import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, EqualsSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('storm_slam')

    slam_mode = LaunchConfiguration('slam_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_slam_mode = DeclareLaunchArgument(
        'slam_mode',
        default_value='2d',
        description='SLAM pipeline: "2d" (SLAM Toolbox) or "3d" (RTAB-Map)'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # 1. Gazebo Harmonic with our world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': os.path.join(pkg, 'worlds', 'storm_world.sdf')
        }.items()
    )

    # 2. robot_state_publisher — publishes fixed TFs from URDF (base_link→velodyne etc.)
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg, 'urdf', 'storm_robot.urdf.xacro')]),
        value_type=str
    )
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # 3. Spawn robot into Gazebo at the world origin
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_storm_robot',
        arguments=[
            '-name', 'storm_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    # 4. ros_gz_bridge — converts Gazebo transport topics to/from ROS2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{
            'config_file': os.path.join(pkg, 'config', 'ros_gz_bridge.yaml'),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # 5. RViz with pre-configured displays (map, scan, point cloud, robot model, odom)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg, 'config', 'slam_viz.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 6a. 2D SLAM (SLAM Toolbox) — delayed 5s to ensure /clock is available
    #     before SLAM nodes with use_sim_time:true initialize
    slam_2d = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, 'launch', 'sim_slam_2d_launch.py')
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
                condition=IfCondition(EqualsSubstitution(slam_mode, '2d'))
            )
        ]
    )

    # 6b. 3D SLAM (RTAB-Map + ICP odometry) — same delay
    slam_3d = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, 'launch', 'sim_slam_3d_launch.py')
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
                condition=IfCondition(EqualsSubstitution(slam_mode, '3d'))
            )
        ]
    )

    return LaunchDescription([
        declare_slam_mode,
        declare_use_sim_time,
        gz_sim,
        rsp,
        spawn_robot,
        bridge,
        rviz,
        slam_2d,
        slam_3d,
    ])
