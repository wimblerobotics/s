import os
import sys
import xacro
import yaml

# import launch
import launch_ros.actions
# from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


s_sub_launch_path = os.path.join(get_package_share_directory('s_base'),
                                    'launch', 'sub_launch')
sys.path.append(s_sub_launch_path)
import common

def generate_launch_description():

    use_sim_time = True
    
    do_mapping = LaunchConfiguration('do_mapping')
    common.ld.add_action(DeclareLaunchArgument(
        name='do_mapping', 
        default_value='false',
        description='true=>use slam_toolbox to map, false=>use AMCL to localize'))

    world_path = PathJoinSubstitution(
        [FindPackageShare("s_description"), "worlds", "home.world"]
    )

    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',  '-s', 'libgazebo_ros_init.so', world_path],
            output='screen'
    )
    common.ld.add_action(gazebo)

    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        arguments=['-topic', 'robot_description',
                    '-entity', 's_bot',
                    '-x' , '8.80', 
                    '-y', '3.0',
        ],
        output='screen'
    )
    common.ld.add_action(spawn_entity)

    # Bring of the EKF node.
    ekf_filter_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ 
            {'use_sim_time': use_sim_time},
            common.ekf_config_path,
        ],
        remappings=[('odometry/filtered', 'odom')]
    )
    common.ld.add_action(ekf_filter_node)

    # Bring up the robot description (URDF).
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            common.s_description_directory_path, '/launch/description.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'publish_joints': 'false'
        }.items()
    )
    common.ld.add_action(description_launch)

    # Bring up the LIDAR multiplexer
    lidar_multiplexer = launch_ros.actions.Node(
            package='ira_laser_tools',
            executable='laserscan_multi_merger',
            name='laserscan_multi_merger',
            parameters=[{
                    "destination_frame": "base_link",
                    "cloud_destination_topic": "/merged_lidar_cloud",
                    "scan_destination_topic": "/scan",
                    "laserscan_topics": "/scan_left_front /scan_right_rear",
                    "angle_min": -3.14159,
                    "angle_max": 3.14159,
                    "angle_increment": 0.013935472816228867,
                    "scan_time": 0.010,
                    "range_min": 0.0,
                    "range_max": 10.0,
                    "max_merge_time_diff": 1000000000.0,
                    # "allow_scan_delay": use_sim_time, # -- code does not read this properly
            }],
            # prefix=["xterm -geometry 200x30 -e gdb -ex run --args"],
            output='screen'
    )
    common.ld.add_action(lidar_multiplexer)

    # Bring up the navigation stack.
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    nav2_config_path =os.path.join(common.s_base_directory_path, 'config', 'navigation.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            'map': common.map_path,
            'use_sim_time': str(use_sim_time),
            'params_file': nav2_config_path
        }.items(),
        condition=UnlessCondition(do_mapping)
    )
    common.ld.add_action(nav2_launch)

    # Bring up SLAM.
    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    slam_config_path = os.path.join(common.s_base_directory_path, 'config', 'slam.yaml')        
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'slam_params_file': slam_config_path
        }.items(),
        condition=IfCondition(do_mapping)
    )
    common.ld.add_action(slam_launch)

    return common.ld
